#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import serial
import struct
import math
import zmq
import time


# ================================================================
#  Constants
# ================================================================
HDR1 = 0xAA
HDR2 = 0x55


# ================================================================
#  LiDAR Driver Node
# ================================================================
class LidarDriverNode(Node):
    """
    SAFRS LiDAR Driver for YDLIDAR X4 / X4 Pro
    Parses raw serial packets → publishes LaserScan + ZMQ JSON.
    """

    def __init__(self):
        super().__init__("lidar_driver")

        # ------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 128000)
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("zmq_pub_ip", "0.0.0.0")
        self.declare_parameter("zmq_pub_port", 5000)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.frame_id = self.get_parameter("frame_id").value
        self.zmq_pub_ip = self.get_parameter("zmq_pub_ip").value
        self.zmq_pub_port = self.get_parameter("zmq_pub_port").value

        # ------------------------------------------------------------
        # Serial Connection
        # ------------------------------------------------------------
        self.ser = None
        self.connect_serial()

        # ------------------------------------------------------------
        # ROS2 Publisher
        # ------------------------------------------------------------
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)

        # ------------------------------------------------------------
        # ZMQ Publisher (Main Pi)
        # ------------------------------------------------------------
        ctx = zmq.Context.instance()
        self.zmq_pub = ctx.socket(zmq.PUB)
        self.zmq_pub.bind(f"tcp://{self.zmq_pub_ip}:{self.zmq_pub_port}")
        self.get_logger().info(
            f"[ZMQ] Publishing LiDAR scan → tcp://{self.zmq_pub_ip}:{self.zmq_pub_port}"
        )

        # ------------------------------------------------------------
        # Internal State
        # ------------------------------------------------------------
        self.buffer = bytearray()

        # Timer for continuous serial reading
        self.create_timer(0.001, self.read_serial)

        self.get_logger().info("[LiDAR] LidarDriverNode initialized.")

    # ================================================================
    #  Serial Connection Handling
    # ================================================================
    def connect_serial(self):
        """Attempt to open the serial port with retries."""
        for attempt in range(5):
            try:
                self.get_logger().info(
                    f"[LiDAR] Opening {self.port} @ {self.baudrate} (attempt {attempt+1})"
                )
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0.01)
                time.sleep(0.2)
                self.get_logger().info("[LiDAR] Serial connection established.")
                return
            except Exception as e:
                self.get_logger().error(
                    f"[LiDAR] Serial connection failed: {e}"
                )
                time.sleep(0.5)

        self.get_logger().fatal("[LiDAR] Failed to open serial port after retries.")
        raise RuntimeError("Serial port unavailable.")

    # ================================================================
    #  Serial Reader
    # ================================================================
    def read_serial(self):
        """Continuously read bytes from serial and feed packet parser."""
        try:
            data = self.ser.read(1024)
            if data:
                self.buffer.extend(data)
                self.parse_packets()
        except Exception as e:
            self.get_logger().error(f"[Serial Error] {e}")

    # ================================================================
    #  Packet Parser for YDLIDAR X4 / X4 Pro
    # ================================================================
    def parse_packets(self):
        """Parse LiDAR packets following YDLidar X4 Pro protocol."""

        MIN_LEN = 10

        while len(self.buffer) >= MIN_LEN:

            # ------------------------------------------------------------
            # Header Check
            # ------------------------------------------------------------
            if not (self.buffer[0] == HDR1 and self.buffer[1] == HDR2):
                self.buffer.pop(0)
                continue

            ct = self.buffer[2]     # Packet type
            lsn = self.buffer[3]    # Number of samples

            # Only support distance packets (CT = 0x66)
            if ct != 0x66:
                self.buffer.pop(0)
                continue

            packet_length = 10 + lsn * 2
            if len(self.buffer) < packet_length:
                return  # Not enough data yet

            # Extract full packet
            packet = self.buffer[:packet_length]
            del self.buffer[:packet_length]

            # ------------------------------------------------------------
            # Parse angle block
            # ------------------------------------------------------------
            fsa_raw = struct.unpack("<H", packet[4:6])[0]
            lsa_raw = struct.unpack("<H", packet[6:8])[0]

            fsa = (fsa_raw >> 1) / 64.0  # deg
            lsa = (lsa_raw >> 1) / 64.0  # deg

            # ------------------------------------------------------------
            # Parse distance array
            # ------------------------------------------------------------
            ranges = []
            for i in range(lsn):
                d_raw = struct.unpack(
                    "<H", packet[8 + 2 * i: 10 + 2 * i]
                )[0]

                if 80 <= d_raw <= 12000:
                    ranges.append(d_raw / 1000.0)  # mm → m
                else:
                    ranges.append(float("inf"))

            # ------------------------------------------------------------
            # Publish ROS LaserScan
            # ------------------------------------------------------------
            self.publish_scan(fsa, lsa, ranges)

            # ------------------------------------------------------------
            # Publish via ZMQ
            # ------------------------------------------------------------
            try:
                self.zmq_pub.send_json({
                    "start_angle": fsa,
                    "end_angle": lsa,
                    "lsn": lsn,
                    "distances": ranges,
                })
            except Exception as e:
                self.get_logger().warn(f"[ZMQ] send error: {e}")

    # ================================================================
    #  LaserScan Publisher
    # ================================================================
    def publish_scan(self, fsa_deg, lsa_deg, ranges):
        """Publish sensor_msgs/LaserScan from extracted LiDAR data."""

        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id

        # Range specifications
        scan_msg.range_min = 0.08
        scan_msg.range_max = 12.0

        # Convert degrees → radians
        angle_min = math.radians(fsa_deg)
        angle_max = math.radians(lsa_deg)

        scan_msg.angle_min = angle_min
        scan_msg.angle_max = angle_max
        scan_msg.angle_increment = (angle_max - angle_min) / max(len(ranges) - 1, 1)

        scan_msg.ranges = ranges
        scan_msg.intensities = [0.0] * len(ranges)

        self.scan_pub.publish(scan_msg)


# ================================================================
#  Main Entry
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = LidarDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
