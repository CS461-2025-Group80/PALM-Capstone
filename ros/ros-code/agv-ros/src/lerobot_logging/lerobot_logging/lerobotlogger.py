import rclpy
from rclpy.node import Node
import cv2
from geometry_msgs.msg import Twist
from um982_gps_manual.msg import UM982Fix
import json
import os
import subprocess
import threading
import argparse
from concurrent.futures import ThreadPoolExecutor
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from datetime import datetime

def open_camera(camera_index, fps, logger, width, height):
    pipeline = (
        f"nvarguscamerasrc sensor-id={camera_index} ! "
        f"video/x-raw(memory:NVMM),width={width},height={height},framerate={fps}/1 ! "
        f"nvvidconv flip-method=2 ! "
        f"video/x-raw,format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw,format=BGR ! "
        f"appsink drop=true max-buffers=1"
    )
    camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not camera.isOpened():
        logger.error("Camera not opened.")
        exit(1)
    logger.info(f"Camera opened via GStreamer at {width}x{height} @ {fps}fps.")
    return camera

class LeRobotLogger(Node):
    def __init__(self, session_name=None, use_camera_subscription=False, session_interval=0.0):
        super().__init__("LeRobotLogger")
        self.get_logger().info("LeRobotLogger initialized.")

        videowriter_file = self.setup_logging(session_name=session_name, session_interval=session_interval)
        self.setup_subscriptions()
        self.setup_camera(use_camera_subscription=use_camera_subscription, filename=videowriter_file)

    ###############################################################################################
    ############################################ SETUP ############################################
    ###############################################################################################

    # preferably, this should be set up first.
    def setup_logging(self, session_name, session_interval):
        # if the user didn't provide a session name,
        if session_name is None:
            # nanoseconds after 1970
            now_ns = self.get_clock().now().nanoseconds
            # datetime (based on seconds after 1970)
            now_dt = datetime.fromtimestamp(now_ns / 1e9)
            # session name follows "day month year hour minute"
            session_name = now_dt.strftime("%d%m%Y%H%M")        

        self.working_dir = os.path.join(get_package_share_directory("lerobot_logging"), f"session_{session_name}")

        # make the directories if they don't exist
        os.makedirs(self.working_dir, exist_ok=True)

        # just default to using the working directory as the output directory
        output_dir = self.working_dir
        content_name = session_name
        
        if session_interval != 0:
            self.session_interval = session_interval

            # current ID of the session (from 0 to infinity)
            self.current_session_id = 0

            content_name = f"session_{self.current_session_id}"

            # the current output directory will change based on the interval
            output_dir = os.path.join(self.working_dir, content_name)

        
        # make all needed directories
        os.makedirs(output_dir, exist_ok=True)
        # the JSON
        output_file_name = os.path.join(output_dir, f"{content_name}.json")

        # open the file
        self.open_file(output_file_name)
        # for marking that the first log entry doesn't need a leading ','
        self.first_log = True
        self._rotating = False

        # return the location of the MP4 (for initializing the camera's video writer)
        return os.path.join(output_dir, f"{content_name}.mp4")
    
    def setup_subscriptions(self):
        # cmd_vel variable (filled by a subscription to cmd_vel)
        self.cmd_vel = None
        # subscribe to cmd_vel
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_intervaller, 1)
        # gps_data variable, filled by the subscrption to gps/data
        self.gps_data = None
        # subscribe to gps
        self.create_subscription(UM982Fix, "gps", self.gps_data_intervaller, 1)
    
    # must occur after logging has been set up.
    def setup_camera(self, use_camera_subscription, filename):
        self.cvbridge = CvBridge()

        # the total count of frames that have been logged
        self.frame_count = 0
        # the FPS the camera will be requested to run at
        self.fps = 30
        self.width = 640
        self.height = 480
        # the latest frame captured by the camera capture thread
        self.frame = None
        # whether we're currently capturing the camera view or not
        self.is_capturing_camera = True
        # if we shouldn't use a subscription for updating the frame,
        if not use_camera_subscription:
            # creating the camera object
            self.camera = open_camera(camera_index=0, fps=self.fps, logger=self.get_logger(), width=self.width, height=self.height)
            # create a thread for specifically capturing camera output (spams camera.read() to keep latest_frame fresh)
            self.camera_capture_thread = threading.Thread(target=self.rect_img_raw_thread_intervaller, daemon=True)
            # start that thread
            self.camera_capture_thread.start()
        else:
            # otherwise, use a subscription
            self.create_subscription(Image, "/camera/image_raw", self.rect_img_raw_subscription_intervaller, 1)
        # for storing the starting time of the first frame
        self.starting_time = None

        # for writing frames
        self.create_video_writer(filename=filename)

        # every 1/FPS seconds, get the latest frame and cmd_vel and dump a log of it
        self.create_timer(1.0 / self.fps, self.dump_intervaller)
        self.get_logger().info(f"Started logging at a {1.0 / self.fps} second(s) interval.")

    ###############################################################################################
    ############################################ UTIL #############################################
    ###############################################################################################
    ################ some useful functions.
    
    def open_file(self, name):
        if hasattr(self, "output_file"):
            self.close_file()

        # open the file in append mode
        self.output_file = open(name, 'a')
        # start the JSON with a leading bracket
        self.output_file.write('[')

        self.get_logger().info(f"Output file set as {name}")

    def close_file(self):
        # write the last bracket (for valid JSON syntax)
        self.output_file.write(']')
        # close the file
        self.output_file.close()
        # null the file out
        self.output_file = None
    
    def try_increment_session(self):
        if not hasattr(self, "session_interval"):
            return

        elapsed_seconds = self.frame_count / self.fps
        interval_seconds = self.session_interval * 60

        if elapsed_seconds >= interval_seconds:
            self.get_logger().info(f"session_{self.current_session_id} reached limit, rotating sessions.")

            # increment the session id
            self.current_session_id += 1
            # content name (name of the items o the files in output directories)
            content_name = f"session_{self.current_session_id}"
            # output dir is named identically to the items, just lacks an extension
            output_dir = os.path.join(self.working_dir, content_name)
            # make the directories
            os.makedirs(output_dir, exist_ok=True)

            # JSON
            json_file_name = os.path.join(output_dir, f"{content_name}.json")
            self.open_file(json_file_name)
            self.first_log = True

            # MP4
            video_file_name = os.path.join(output_dir, f"{content_name}.mp4")
            self.create_video_writer(video_file_name)

            # reset the frame counter
            self.frame_count = 0
    
    def create_video_writer(self, filename):
        if hasattr(self, "video_writer"):
            self.video_writer.release()

        self.video_writer = cv2.VideoWriter(
            filename,
            cv2.VideoWriter_fourcc(*"mp4v"),
            self.fps,
            (self.width, self.height)
        )

    ###############################################################################################
    ######################################### INTERVALLERS ########################################
    ###############################################################################################
    ################ functions that are periodically called.

    def cmd_vel_intervaller(self, msg):
        self.cmd_vel = msg
    
    def gps_data_intervaller(self, msg):
        self.gps_data = msg
    
    def rect_img_raw_subscription_intervaller(self, msg):
        if self.is_capturing_camera:
            self.frame = self.cvbridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def rect_img_raw_thread_intervaller(self):
        # spam camera.read() so frame is always as fresh as possible
        while self.is_capturing_camera:
            ret, frame = self.camera.read()
            if ret:
                self.frame = frame
    
    def dump_intervaller(self):
        # don't dump anything if we're not capturing camera frames
        if not self.is_capturing_camera:
            self.get_logger().warn("is_capturing_camera is False.")
            return

        # try to increment the file log if needed (nothing done if self.session_interval is undefined)
        self.try_increment_session()

        relative_time = 0
        linear_x = 0
        angular_z = 0
        lat = None
        lon = None
        heading = None
        frame_index = self.frame_count

        # if we have a starting time,
        if self.starting_time is not None:
            # calculate the difference between now and then, in seconds
            relative_time =  (self.get_clock().now() - self.starting_time).nanoseconds / 1e9
        else:
            # leave relative time as 0 since this is the first frame. just store the current time for future frames.
            self.starting_time = self.get_clock().now()
        
        if self.cmd_vel is not None:
            linear_x = self.cmd_vel.linear.x
            angular_z = self.cmd_vel.angular.z

        if self.frame is not None:
            # write the frame to the video we're making
            self.video_writer.write(self.frame)
            # increment the frame count (we got a frame)
            self.frame_count += 1
        
        if self.gps_data is not None:
            lat = self.gps_data.latitude
            lon = self.gps_data.longitude
            heading = self.gps_data.heading
        
        data = {
            "relative_time": relative_time,
            "linear_x": linear_x,
            "angular_z": angular_z,
            "frame_index": frame_index,
            "lat": lat,
            "lon": lon,
            "heading": heading
        }

        # write this data into the file. this MUST be synchronous.
        if self.first_log:
            # no leading comma for the first log.
            self.output_file.write(json.dumps(data) + '\n')
            self.first_log = False
        else:
            # leading commas for logs that aren't the first.
            self.output_file.write(',' + json.dumps(data) + '\n')

        self.get_logger().info(f"Caught frame {frame_index}: {data}")
        
    ###############################################################################################
    ########################################### CLEANUP ###########################################
    ###############################################################################################
    ################ a single function for cleaning up the class. highly recommended to call this when done.

    # class cleanup
    def finish(self):
        self.get_logger().info("Cleaning up LeRobotLogger...")

        # stop the looping thread and the timer
        self.is_capturing_camera = False

        # if the thread was initialized,
        if hasattr(self, "camera_capture_thread"):
            # join the thread
            self.camera_capture_thread.join(timeout=2.0)
            self.camera_capture_thread = None
        
        # if we have an output file,
        if hasattr(self, "output_file"):
            self.close_file()

        # if the camera was initialized,
        if hasattr(self, "camera"):
            # try to release ownership of it.
            try:
                self.camera.release()
                self.camera = None
                cv2.destroyAllWindows()
            except:
                pass
        
        if hasattr(self, "video_writer"):
            self.video_writer.release()
            self.video_writer = None
        
        self.get_logger().info("Finished cleaning LeRobotLogger.")

def main(args=None):
    parser = argparse.ArgumentParser(description="LeRobotLogger node")
    parser.add_argument("--session-name", type=str, default=None, help="The name of the session of this run of logging. Reflected in the log file name and the session directory.")
    parser.add_argument("--run-cam-ros", type=int, default=0, help="0 if we're running the camera with GStreamer, 1 if it's with ROS.")
    parser.add_argument("--session-interval", type=float, default=0.0, help="If nonzero, this will break the session into having subsessions that are all at maximum the time in minutes you specify.")
    
    parsed_args, remaining_args = parser.parse_known_args(args)
    
    rclpy.init(args=remaining_args)
    node = LeRobotLogger(session_name=parsed_args.session_name, use_camera_subscription=(parsed_args.run_cam_ros == 1), session_interval=parsed_args.session_interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()