#!/usr/bin/env python3
"""
um982_node.py

Minimal ROS2 node for the MJ RTK GPS (Unicorecomm UM982).
Reads raw NMEA sentences from a serial port and publishes
everything on a single topic:

  /gps     um982_gps_manual/UM982Fix

Parsed sentences:
  $GNGGA  — position, altitude, fix quality, HDOP, satellites
  $GNRMC  — ground speed, course over ground
  $GNHPR  — heading, pitch, roll  (UM982 proprietary)

No external GPS libraries required — pure Python parsing.
"""

import math
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Header
from um982_gps_manual.msg import UM982Fix


# ---------------------------------------------------------------------------
# NMEA helpers
# ---------------------------------------------------------------------------

def nmea_checksum_valid(sentence: str) -> bool:
    try:
        data, checksum = sentence.strip().lstrip('$').rsplit('*', 1)
        computed = 0
        for ch in data:
            computed ^= ord(ch)
        return computed == int(checksum, 16)
    except Exception:
        return False


def nmea_dms_to_decimal(value: str, direction: str) -> float:
    if not value:
        return float('nan')
    dot = value.index('.')
    degrees = float(value[:dot - 2])
    minutes = float(value[dot - 2:])
    decimal = degrees + minutes / 60.0
    if direction in ('S', 'W'):
        decimal = -decimal
    return decimal


# ---------------------------------------------------------------------------
# Sentence parsers
# ---------------------------------------------------------------------------

def parse_gga(fields):
    try:
        return {
            'lat':        nmea_dms_to_decimal(fields[2], fields[3]),
            'lon':        nmea_dms_to_decimal(fields[4], fields[5]),
            'quality':    int(fields[6]),
            'satellites': int(fields[7]),
            'hdop':       float(fields[8]) if fields[8] else float('nan'),
            'altitude':   float(fields[9]) if fields[9] else float('nan'),
        }
    except Exception:
        return None


def parse_rmc(fields):
    try:
        if fields[2] != 'A':   # A=active, V=void
            return None
        return {
            'speed':  float(fields[7]) * 0.514444 if fields[7] else 0.0,  # knots → m/s
            'course': float(fields[8]) if fields[8] else float('nan'),
        }
    except Exception:
        return None


def parse_hpr(fields):
    try:
        roll_raw = fields[4].split('*')[0] if '*' in fields[4] else fields[4]
        return {
            'heading': float(fields[2]) if fields[2] else float('nan'),
            'pitch':   float(fields[3]) if fields[3] else float('nan'),
            'roll':    float(roll_raw)  if roll_raw  else float('nan'),
        }
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class UM982Node(Node):

    def __init__(self):
        super().__init__('um982_gps_manual')

        self.declare_parameter('port',     '/dev/ttyUSB0')
        self.declare_parameter('baud',     115200)
        self.declare_parameter('frame_id', 'gps')

        port          = self.get_parameter('port').get_parameter_value().string_value
        baud          = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.pub = self.create_publisher(UM982Fix, 'gps', 10)

        # Accumulated state — publish whenever GGA arrives (it comes most often)
        self._msg = UM982Fix()
        self._msg.fix_quality  = 0
        self._msg.heading      = float('nan')
        self._msg.pitch        = float('nan')
        self._msg.roll         = float('nan')
        self._msg.speed        = float('nan')
        self._msg.course       = float('nan')

        try:
            self.serial = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'Opened {port} @ {baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open {port}: {e}')
            raise

        self.create_timer(0.0, self.read_loop)

    def read_loop(self):
        try:
            raw = self.serial.readline()
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read error: {e}')
            return

        try:
            line = raw.decode('ascii', errors='ignore').strip()
        except Exception:
            return

        if not line.startswith('$') or not nmea_checksum_valid(line):
            return

        sentence = line.split('*')[0]
        fields   = sentence.split(',')
        msg_type = fields[0].lstrip('$')

        if msg_type in ('GNGGA', 'GPGGA'):
            data = parse_gga(fields)
            if data:
                self._msg.latitude   = data['lat']
                self._msg.longitude  = data['lon']
                self._msg.altitude   = data['altitude']
                self._msg.fix_quality  = data['quality']
                self._msg.satellites   = data['satellites']
                self._msg.hdop         = data['hdop']
                self._publish()

        elif msg_type in ('GNRMC', 'GPRMC'):
            data = parse_rmc(fields)
            if data:
                self._msg.speed  = data['speed']
                self._msg.course = data['course']

        elif msg_type in ('GNHPR', 'GPHPR'):
            data = parse_hpr(fields)
            if data:
                self._msg.heading = data['heading']
                self._msg.pitch   = data['pitch']
                self._msg.roll    = data['roll']

    def _publish(self):
        self._msg.header.stamp    = self.get_clock().now().to_msg()
        self._msg.header.frame_id = self.frame_id
        self.pub.publish(self._msg)

    def destroy_node(self):
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UM982Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
