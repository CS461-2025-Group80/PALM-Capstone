# specify dependencies in package.xml.
# specify that main must be ran in setup.py

# note that packages can't contain "-"


# ROS2 Python Client Library
import rclpy

# Node module for doing publisher-subscriber communication
from rclpy.node import Node

# for Node types
from std_msgs.msg import String

class some_publisher(Node):
    # called on class initialization
    def __init__(self):
        # pass the name of this publisher-subscriber node.
        super().__init__("some_node_publisher_name")

        self.pub = self.create_publisher(
            String, # type of the value that will be written to this topic. Use std_msgs types.
            "sometopic", # the name of the topic
            10 # queue size for subscribers (listeners) for this topic. listeners are responsible for popping these elements.
        )

        # udemy guy likes to count the number of messages sent in total and the frequency of message sending
        self.counter = 0
        self.frequency = 1.0


        # ROS-standardized logging system. ".info" is just an informational log function.
        self.get_logger().info(f"publishing messages per {self.frequency} seconds")

        # udemy guy wants to call a function every "frequency" seconds.
        self.timer = self.create_timer(self.frequency, self.timerCallback)

    # called every "self.frequency" seconds, as specified by init
    def timerCallback(self):
        # "msg" is a String object (odd standard)
        msg = String()

        # modify the content of the String object
        msg.data = f"hello there. counter is: {self.counter}"
        
        # as specified by init, .pub is a publisher class. .publish actually writes that message to the topic in question.
        self.pub.publish(msg)
        self.counter += 1


# you need a static function. this can be called by a script.
def main():
    # initialize the ROS2 Python Client Library
    rclpy.init()

    # do whatever you want next. in this case, I initialize my publisher class.
    some_pub = some_publisher()

    # rclpy does process handling. it requires any and all nodes to be given to it via .add_node(node) or being passed directly to spin.
    # having more nodes makes your entire file a multithreaded one. you can specify for it to hop between the nodes (and thus being single-threaded)
    # via SingleThreadedExecutor. MultiThreadedExecutor is obviously with multiple threads (MultiThreadedExecutor(num_threads=n)).
    rclpy.spin(some_pub)

    # if spin stops, that means ROS2 has stopped its execution of nodes (or this node died)

    # cleanup for the nodes
    some_pub.destroy_node()

    # shutting off the ROS2 Python Client Library
    rclpy.shutdown()


# typical python main stuff
if __name__ == "__main__":
    main()
