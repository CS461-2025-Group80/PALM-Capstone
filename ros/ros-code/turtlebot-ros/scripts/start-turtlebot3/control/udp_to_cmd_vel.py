#!/usr/bin/env python3
import json, socket, time, threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

UDP_IP   = "0.0.0.0"   # listen on all interfaces
UDP_PORT = 55055
WATCHDOG_TIMEOUT = 0.3  # seconds: stop if no packets for this long
PRINT_EVERY_SEC = 2.0

class UdpCmdVelBridge(Node):
    def __init__(self):
        super().__init__("udp_cmdvel_bridge")

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST
        self.pub = self.create_publisher(Twist, "/cmd_vel", qos)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.settimeout(0.05)  # non-blocking-ish

        self.last_rx = 0.0
        self.last_log = 0.0
        self.latest_cmd = Twist()

        # threads
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        # 50 Hz publisher + watchdog
        self.timer = self.create_timer(1.0/50.0, self.publish_loop)

        self.get_logger().info(f"UDP → /cmd_vel bridge listening on {UDP_IP}:{UDP_PORT}")

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
                t = Twist()
                t.linear.x  = lin
                t.angular.z = ang
                self.latest_cmd = t

                # periodic debug
                if now - self.last_log > PRINT_EVERY_SEC:
                    self.get_logger().info(
                        f"RX from {addr[0]} seq={msg.get('seq')} lin_x={lin:.2f} ang_z={ang:.2f}"
                    )
                    self.last_log = now

            except (ValueError, KeyError, json.JSONDecodeError) as e:
                self.get_logger().warn(f"bad packet: {e}")

    def publish_loop(self):
        now = time.time()
        # watchdog: if stale, publish zero
        if now - self.last_rx > WATCHDOG_TIMEOUT:
            twist = Twist()  # zeros
        else:
            twist = self.latest_cmd
        self.pub.publish(twist)

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

