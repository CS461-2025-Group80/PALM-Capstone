import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

from . import mjpeg_server


class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('camera_stream_node')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('use_compressed', False)  # set True if topic is CompressedImage
        self.declare_parameter('jpeg_quality', 80)       # 0-100; lower = faster, smaller
        self.declare_parameter('http_port', 8080)

        topic    = self.get_parameter('image_topic').value
        use_comp = self.get_parameter('use_compressed').value
        self._quality = int(self.get_parameter('jpeg_quality').value)
        port     = int(self.get_parameter('http_port').value)

        self._bridge = CvBridge()

        # ── HTTP server ──────────────────────────────────────────────
        mjpeg_server.start_server(port=port)
        self.get_logger().info(f'MJPEG stream at http://<robot-ip>:{port}/stream')

        # ── Subscriber ──────────────────────────────────────────────
        if use_comp:
            self.create_subscription(
                CompressedImage, topic, self._compressed_cb, 1)  # QoS depth=1 drops stale frames
            self.get_logger().info(f'Subscribed (CompressedImage): {topic}')
        else:
            self.create_subscription(
                Image, topic, self._raw_cb, 1)
            self.get_logger().info(f'Subscribed (Image): {topic}')

    # ── Callbacks ────────────────────────────────────────────────────

    def _raw_cb(self, msg: Image):
        """Convert raw ROS Image → OpenCV → JPEG bytes."""
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._push(cv_img)
        except Exception as e:
            self.get_logger().warn(f'raw_cb error: {e}')

    def _compressed_cb(self, msg: CompressedImage):
        """Convert CompressedImage → OpenCV → JPEG bytes.
        
        If the source is already JPEG, we can re-use the bytes directly
        (zero re-encoding cost).
        """
        try:
            if msg.format.lower().startswith('jpeg') or msg.format.lower().startswith('jpg'):
                # Already JPEG — push raw bytes, no re-encode needed
                mjpeg_server.update_frame(bytes(msg.data))
            else:
                cv_img = self._bridge.compressed_imgmsg_to_cv2(msg)
                self._push(cv_img)
        except Exception as e:
            self.get_logger().warn(f'compressed_cb error: {e}')

    def _push(self, cv_img):
        """JPEG-encode and push to the HTTP server."""
        ok, buf = cv2.imencode(
            '.jpg', cv_img,
            [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        if ok:
            mjpeg_server.update_frame(buf.tobytes())


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()