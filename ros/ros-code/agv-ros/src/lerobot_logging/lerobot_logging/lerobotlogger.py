import rclpy
from rclpy.node import Node
import cv2
from geometry_msgs.msg import Twist
import json
import os
import subprocess
import threading
import argparse
from concurrent.futures import ThreadPoolExecutor
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def open_camera(camera_index, fps, logger, width, height):
    pipeline = (
        f"nvarguscamerasrc sensor-id={camera_index} ! "
        f"video/x-raw(memory:NVMM),width={width},height={height},framerate={fps}/1 ! "
        f"nvvidconv flip-method=2 ! "
        f"video/x-raw,format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw,format=BGR ! "
        f"appsink"
    )
    # open the camera
    camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    # if it didn't open,
    if not camera.isOpened():
        # error out
        logger.error("Camera not opened.")
        exit(1)
    logger.info(f"Camera opened via GStreamer at {width}x{height} @ {fps}fps.")

    return camera

def write_frame(destination, frame):
    cv2.imwrite(destination, frame)

class LeRobotLogger(Node):
    def __init__(self, session_name=None, use_camera_subscription=False):
        super().__init__("LeRobotLogger")
        self.get_logger().info("LeRobotLogger initialized.")
        self.cvbridge = CvBridge()

        ######################################################################## LOGGING
        # if the user didn't provide a session name,
        if session_name is None:
            # default to just the current nanosecond after 1970
            session_name = f"{self.get_clock().now().nanoseconds}"
        session_name = f"session_{session_name}"
        # output directory is under share for the lerobot_logging ROS package and under a specific directory name
        self.output_dir = os.path.join(get_package_share_directory("lerobot_logging"), f"{session_name}")
        # make the directories if they don't exist
        os.makedirs(self.output_dir, exist_ok=True)
        # the output file name 
        self.output_file_name = os.path.join(self.output_dir, f"{session_name}.json")

        # output file object
        self.output_file = open(self.output_file_name, 'a')
        # for formatting. REQUIRES that we append ']' later.
        self.output_file.write('[')
        # for marking that the first log entry doesn't need a leading ','
        self.first_log = True
        # for deferring the work of writing MJPEGs
        self.write_executor = ThreadPoolExecutor(max_workers=5)

        self.get_logger().info(f"Output file set as {self.output_file_name}")

        ######################################################################## SUBSCRIPTION TO cmd_vel
        # cmd_vel variable (filled by a subscription to cmd_vel)
        self.cmd_vel = None
        # subscribe to cmd_vel
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_intervaller, 1)

        ######################################################################## CAMERA INITIALIZATION & CAPTURING
        # the total count of frames that have been logged
        self.frame_count = 0
        # the FPS the camera will be requested to run at
        self.fps = 30
        # the latest frame captured by the camera capture thread
        self.frame = None
        # whether we're currently capturing the camera view or not
        self.is_capturing_camera = True
        # if we shouldn't use a subscription for updating the frame,
        if not use_camera_subscription:
            # creating the camera object
            self.camera = open_camera(camera_index=0, fps=self.fps, logger=self.get_logger(), width=640, height=480)
            # create a thread for specifically capturing camera output (spams camera.read() to keep latest_frame fresh)
            self.camera_capture_thread = threading.Thread(target=self.rect_img_raw_thread_intervaller, daemon=True)
            # start that thread
            self.camera_capture_thread.start()
        else:
            # otherwise, use a subscription
            self.create_subscription(Image, "/camera/image_raw", self.rect_img_raw_subscription_intervaller, 1)
        # for storing the starting time of the first frame
        self.starting_time = None
        # every 1/FPS seconds, get the latest frame and cmd_vel and dump a log of it
        self.create_timer(1.0 / self.fps, self.dump_intervaller)
        self.get_logger().info(f"Started logging at a {1.0 / self.fps} second(s) interval.")

    def cmd_vel_intervaller(self, msg):
        self.cmd_vel = msg
    
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

        relative_time = 0
        linear_x = 0
        angular_z = 0
        frame_name = None

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
            frame_name = f"frame_{self.frame_count}.jpg"
            # defer writing the frame to another thread.
            self.write_executor.submit(write_frame, os.path.join(self.output_dir, frame_name), self.frame)
        
        data = {
            "relative_time": relative_time,
            "linear_x": linear_x,
            "angular_z": angular_z,
            "frame_name": frame_name
        }

        # write this data into the file. this MUST be synchronous.
        if self.first_log:
            # no leading comma for the first log.
            self.output_file.write(json.dumps(data) + '\n')
            self.first_log = False
        else:
            # leading commas for logs that aren't the first.
            self.output_file.write(',' + json.dumps(data) + '\n')

        self.get_logger().info(f"Caught frame {self.frame_count}: {data}")
        self.frame_count += 1
        
    
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
            # write the last bracket (for valid JSON syntax)
            self.output_file.write(']')
            # close the file
            self.output_file.close()
            # null the file out
            self.output_file = None

        # if the camera was initialized,
        if hasattr(self, "camera"):
            # try to release ownership of it.
            try:
                self.camera.release()
                self.camera = None
                cv2.destroyAllWindows()
            except:
                pass
        
        # if the write executor was initialized,
        if hasattr(self, "write_executor"):
            # shut it down
            self.write_executor.shutdown(wait=True)
            # null it out
            self.write_executor = None
        
        self.get_logger().info("Finished cleaning LeRobotLogger.")

def main(args=None):
    parser = argparse.ArgumentParser(description="LeRobotLogger node")
    parser.add_argument("--session-name", type=str, default=None, help="The name of the session of this run of logging. Reflected in the log file name and the session directory.")
    parser.add_argument("--run-cam-ros", type=int, default=0, help="0 if we're running the camera with GStreamer, 1 if it's with ROS.")
    
    parsed_args, remaining_args = parser.parse_known_args(args)
    
    rclpy.init(args=remaining_args)
    node = LeRobotLogger(session_name=parsed_args.session_name, use_camera_subscription=(parsed_args.run_cam_ros == 1))
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