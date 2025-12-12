#!/usr/bin/env python3
"""
SAFRS Main Pi - ZMQ Bridge Module
Module: LidarZMQSubscriber
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from main_pi.scripts.utils.zmq_utils import create_subscriber
from main_pi.scripts.utils.param_utils import declare_and_get


class LidarZMQSubscriber(Node):

    def __init__(self):
        super().__init__("lidar_zmq_subscriber")

        # ------------------------------
        # Parameters
        # ------------------------------
        ip = declare_and_get(self, "lidar_ip", "172.30.1.14")
        port = declare_and_get(self, "lidar_port", 6000)
        topic = declare_and_get(self, "topic_name", "/scan")
        self.frame_id = declare_and_get(self, "frame_id", "laser")

        # ------------------------------
        # ROS Publisher
        # ------------------------------
        self.pub = self.create_publisher(LaserScan, topic, 10)

        # ------------------------------
        # ZMQ Subscriber
        # ------------------------------
        self.sub = create_subscriber(ip, port)
        self.get_logger().info(f"[LidarZMQ] Listening → tcp://{ip}:{port}")

        self.timer = self.create_timer(0.01, self.receive)

    # ------------------------------------------------------------------
    def receive(self):
        try:
            data = self.sub.recv_json(flags=1)
        except Exception:
            return

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id

        start = math.radians(data["start_angle"])
        end = math.radians(data["end_angle"])
        count = data["lsn"]

        scan.angle_min = start
        scan.angle_max = end
        scan.angle_increment = (end - start) / max(count - 1, 1)
        scan.range_min = 0.05
        scan.range_max = 12.0
        scan.ranges = data["distances"]

        self.pub.publish(scan)


def main():
    rclpy.init()
    node = LidarZMQSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
