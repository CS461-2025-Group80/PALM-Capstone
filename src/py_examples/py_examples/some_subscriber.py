import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class some_subscriber(Node):
    def __init__(self):
        super().__init__("some_node_subscriber_name")
        self.sub = self.create_subscription(
            # data type
            String,
            # topic name
            "sometopic",
            # callback function upon a message being sent to the topic
            self.message_written,
            # the queue size (number of callbacks to hold)
            10
        )
    
    def message_written(self, message):
        self.get_logger().info(f"got message {message}")


def main():
    rclpy.init()
    some_sub = some_subscriber()
    rclpy.spin(some_sub)
    some_sub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()