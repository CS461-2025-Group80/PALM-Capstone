import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from azure.storage.blob import BlobServiceClient
import cv2
import time


class AzureImageUploader(Node):
    def __init__(self):
        super().__init__("azure_image_uploader")

        # Create the CvBridge for converting ROS Image → OpenCV
        self.bridge = CvBridge()

        # Azure setup
        self.blob_service = BlobServiceClient.from_connection_string(
            "<your-connection-string>"
        )
        self.container = self.blob_service.get_container_client("images")

        # Subscriber for RealSense camera topic
        self.sub = self.create_subscription(
            # data type
            Image,
            # topic name
            "/camera/color/image_raw",
            # callback function
            self.image_received,
            # queue size
            10
        )

    def image_received(self, message):
        try:
            # Convert ROS Image → CV2
            cv_image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")

            # Encode as JPEG bytes
            success, jpg_bytes = cv2.imencode(".jpg", cv_image)
            if not success:
                self.get_logger().error("Failed to encode image.")
                return

            data = jpg_bytes.tobytes()

            # Upload to Azure Blob
            blob_name = f"frame_{int(time.time())}.jpg"
            self.container.upload_blob(name=blob_name, data=data)

            self.get_logger().info(f"Uploaded {blob_name}")

        except Exception as e:
            self.get_logger().error(f"Upload failed: {e}")


def main():
    rclpy.init()
    uploader = AzureImageUploader()
    rclpy.spin(uploader)
    uploader.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
