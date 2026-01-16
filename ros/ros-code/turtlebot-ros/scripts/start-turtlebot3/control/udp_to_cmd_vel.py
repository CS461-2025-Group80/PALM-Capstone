#!/usr/bin/env python3
import json
import socket
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ==========================
# Configuration
# ==========================
UDP_IP   = "0.0.0.0"
UDP_PORT = 55055

WATCHDOG_TIMEOUT = 0.3     # seconds
PRINT_EVERY_SEC  = 2.0

# Uniform-magnitude parameters
MAX_LINEAR_SPEED  = 0.3    # m/s
MAX_ANGULAR_SPEED = 1.2    # rad/s
EPS = 1e-3

# ==========================
# Node
# ==========================
class UdpCmdVelBridge(Node):
    def __init__(self):
        super().__init__("udp_cmdvel_bridge")

        # QoS
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        self.pub = self.create_publisher(Twist, "/cmd_vel", qos)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.settimeout(0.05)

        self.last_rx = 0.0
        self.last_log = 0.0
        self.latest_cmd = Twist()

        # Receiver thread
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        # 50 Hz publisher + watchdog
        self.timer = self.create_timer(1.0 / 50.0, self.publish_loop)

        self.get_logger().info(
            f"UDP → /cmd_vel bridge listening on {UDP_IP}:{UDP_PORT}"
        )

    # ==========================
    # UDP receive loop
    # ==========================
    def rx_loop(self):
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            now = time.time()
            self.last_rx = now

            try:
                msg = json.loads(data.decode("utf-8"))

                lin = float(msg.get("lin_x", 0.0))
                ang = float(msg.get("ang_z", 0.0))

                # --------------------------
                # Uniform magnitude control
                # --------------------------
                mag = (lin ** 2 + ang ** 2) ** 0.5
                t = Twist()

                if mag > EPS:
                    t.linear.x  = MAX_LINEAR_SPEED  * (lin / mag)
                    t.angular.z = MAX_ANGULAR_SPEED * (ang / mag)
                else:
                    t.linear.x  = 0.0
                    t.angular.z = 0.0

                self.latest_cmd = t

                # Periodic debug logging
                if now - self.last_log > PRINT_EVERY_SEC:
                    self.get_logger().info(
                        f"RX {addr[0]} seq={msg.get('seq')} "
                        f"→ lin={t.linear.x:.2f} ang={t.angular.z:.2f}"
                    )
                    self.last_log = now

            except (ValueError, KeyError, json.JSONDecodeError) as e:
                self.get_logger().warn(f"Bad packet: {e}")

    # ==========================
    # Publisher + watchdog
    # ==========================
    def publish_loop(self):
        now = time.time()

        if now - self.last_rx > WATCHDOG_TIMEOUT:
            # Stop robot if input is stale
            twist = Twist()
        else:
            twist = self.latest_cmd

        self.pub.publish(twist)


# ==========================
# Main
# ==========================
def main():
    rclpy.init()
    node = UdpCmdVelBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down bridge.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
