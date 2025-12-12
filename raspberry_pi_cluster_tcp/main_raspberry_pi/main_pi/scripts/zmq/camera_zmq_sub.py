#!/usr/bin/env python3
"""
SAFRS Main Pi - ZMQ Bridge Module
Module: CameraZMQSubscriber

Description:
    Receives camera frames from Camera Raspberry Pi over ZeroMQ.
    Decodes JPEG/Base64 frames and publishes ROS2 Image messages.
"""

import base64
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from main_pi.scripts.utils.param_utils import declare_and_get
from main_pi.scripts.utils.zmq_utils import create_subscriber


class CameraZMQSubscriber(Node):
    """
    SAFRS Camera ZMQ Subscriber
    Receives frames → publishes /camera/image_raw
    """

    def __init__(self):
        super().__init__("camera_zmq_subscriber")

        # ------------------------------
        # Parameters
        # ------------------------------
        ip = declare_and_get(self, "camera_ip", "172.30.1.5")
        port = declare_and_get(self, "camera_port", 7000)
        self.topic = declare_and_get(self, "topic_name", "/camera/image_raw")

        # ------------------------------
        # ROS Publisher
        # ------------------------------
        self.pub = self.create_publisher(Image, self.topic, 10)

        # ------------------------------
        # ZMQ Subscriber
        # ------------------------------
        self.sub = create_subscriber(ip, port)
        self.get_logger().info(
            f"[CameraZMQ] Listening → tcp://{ip}:{port}"
        )

        # Timer
        self.timer = self.create_timer(0.01, self.recv_frame)

    # ------------------------------------------------------------------
    def recv_frame(self):
        try:
            raw = self.sub.recv_json(flags=1)
        except Exception:
            return
        
        if "frame" not in raw:
            self.get_logger().warn("[CameraZMQ] Invalid JSON data")
            return

        # Base64 decode
        try:
            jpg = base64.b64decode(raw["frame"])
            arr = np.frombuffer(jpg, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"[CameraZMQ] Decode error: {e}")
            return

        # Convert to ROS image
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width, _ = frame.shape
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = frame.tobytes()

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CameraZMQSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
