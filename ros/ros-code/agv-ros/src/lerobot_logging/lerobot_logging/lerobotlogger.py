import rclpy
from rclpy.node import Node
import cv2
from geometry_msgs.msg import Twist
import json
import base64
import os
import subprocess
import threading
from concurrent.futures import ThreadPoolExecutor
from ament_index_python.packages import get_package_share_directory

def open_camera(dev, fps, logger, width, height, force=False):
    # if we're forcefully starting,
    if force:
        try:
            # kill gstreamer
            subprocess.run(['killall', 'gst-launch-1.0'], stderr=subprocess.DEVNULL)
        except:
            pass
    # open the camera
    camera = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    # if it's not opened,
    if not camera.isOpened():
        # error out.
        logger.error("Camera not opened.")
        exit(1)

    # set dimensions
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    # set encoding (MJPEG)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    # set FPS
    camera.set(cv2.CAP_PROP_FPS, fps)

    # get the actual FPS (may return -1, cameras may not always work with this)
    actual_fps = camera.get(cv2.CAP_PROP_FPS)
    logger.info(f"Camera opened at FPS {actual_fps} ({fps} specified).")

    return camera

# create a directory for logging and a specific directory for a specified session of logging
def create_logging_directory(parent_dir, session_dir):
    output_dir = os.path.join(parent_dir, session_dir)
    os.makedirs(output_dir, exist_ok=True)
    return output_dir

# write log data to a specific file under a specified directory
def write_log(data, output_dir, unique_name):
    file_path = os.path.join(output_dir, f"{unique_name}.json")
    with open(file_path, 'w') as f:
        json.dump(data, f)
    return file_path

# for running in parallel
def encode_and_write(frame, cmd_vel, relative_time, frame_index, output_dir, logger):
    _, buffer = cv2.imencode(".jpg", frame)
    frame_encoded = base64.b64encode(buffer).decode("utf-8")

    data = {
        "relative_time": relative_time,
        "linear_x": cmd_vel.linear.x,
        "angular_z": cmd_vel.angular.z,
        "frame": frame_encoded
    }

    log_location = write_log(data=data, output_dir=output_dir, unique_name=f"frame_{frame_index}")
    logger.info(f"{frame_index} wrote at {log_location}.")

class LeRobotLogger(Node):
    def __init__(self):
        super().__init__("LeRobotLogger")
        self.get_logger().info("LeRobotLogger initialized")

        ######################################################################## LOGGING
        # output directory we'll be logging to
        self.output_dir = create_logging_directory(parent_dir=get_package_share_directory("lerobot_logging"), session_dir=f"session_{self.get_clock().now().nanoseconds}")
        self.get_logger().info(f"Output directory set as {self.output_dir}")
        # a thread we can use for handling writes
        self.write_executor = ThreadPoolExecutor(max_workers=5)

        ######################################################################## SUBSCRIPTION TO cmd_vel
        # cmd_vel variable (filled by a subscription to cmd_vel)
        self.cmd_vel = None
        # subscribe to cmd_vel
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 1)

        ######################################################################## CAMERA INITIALIZATION & CAPTURING
        # the total count of frames that have been logged
        self.frame_count = 0
        # the FPS the camera will be requested to run at
        self.fps = 30
        # creating the camera object
        self.camera = open_camera(dev="/dev/video0", fps=self.fps, logger=self.get_logger(), width=640, height=480, force=True)
        # the latest frame captured by the camera capture thread
        self.frame = None
        # whether we're currently capturing the camera view or not
        self.is_capturing_camera = True
        # create a thread for specifically capturing camera output (spams camera.read() to keep latest_frame fresh)
        self.camera_capture_thread = threading.Thread(target=self.camera_capture_thread_function, daemon=True)
        # start that thread
        self.camera_capture_thread.start()
        # every 1/FPS seconds, get the latest frame and cmd_vel and dump a log of it
        self.create_timer(1.0 / self.fps, self.dump_intervaller)

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def camera_capture_thread_function(self):
        # spam camera.read() so latest_frame is always as fresh as possible
        while self.is_capturing_camera:
            ret, frame = self.camera.read()
            if ret:
                self.frame = frame

    def dump_intervaller(self):
        if self.cmd_vel is None:
            self.get_logger().warn("cmd_vel is None.")
            return

        # snapshot the latest frame from the capture thread
        frame = self.frame
        if frame is None:
            self.get_logger().warn("frame is None.")
            return

        self.get_logger().info(f"{self.frame_count} frame received.")
        relative_time = self.frame_count / self.fps
        self.frame_count += 1

        self.write_executor.submit(encode_and_write, frame, self.cmd_vel, relative_time, self.frame_count - 1, self.output_dir, self.get_logger())

def main(args=None):
    rclpy.init(args=args)
    node = LeRobotLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_executor.shutdown(wait=True)
        # stop the looping thread
        node.is_capturing_camera = False
        if hasattr(node, "camera_capture_thread"):
            # join the thread
            node.camera_capture_thread.join(timeout=2.0)
        if hasattr(node, "camera"):
            try:
                # release the camera from out possession
                node.camera.release()
                node.camera = None
                cv2.destroyAllWindows()
            except:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()