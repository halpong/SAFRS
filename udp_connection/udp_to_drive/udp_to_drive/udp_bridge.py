#!/usr/bin/env python3
"""
SAFRS - UDP → ROS2 Bridge
Listens for UDP packets from PC → Publishes trigger message to ROS2.
"""

import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import yaml
import os


class UDPToROS2Bridge(Node):
    """
    SAFRS Standard:
    - Load settings from config.yaml
    - Listen on UDP (non-blocking)
    - Publish a ROS2 trigger message when a specific word is received
    """

    def __init__(self):
        super().__init__("udp_to_ros2_bridge")

        # ----------------------------------------------------
        # Load configuration file
        # ----------------------------------------------------
        pkg_dir = get_package_share_directory("udp_to_drive")
        config_path = os.path.join(pkg_dir, "udp_to_drive", "config.yaml")

        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        udp_cfg = config["udp"]
        topic_cfg = config["topic"]
        msg_cfg = config["message"]

        self.udp_ip = udp_cfg["ip"]
        self.udp_port = udp_cfg["port"]
        self.topic_name = topic_cfg["start_drive"]
        self.trigger_word = msg_cfg["trigger_word"]
        self.publish_word = msg_cfg["publish_word"]

        # ----------------------------------------------------
        # ROS2 Publisher
        # ----------------------------------------------------
        self.publisher = self.create_publisher(String, self.topic_name, 10)
        self.get_logger().info(f"[ROS2] Publisher ready → {self.topic_name}")

        # ----------------------------------------------------
        # UDP Socket Setup
        # ----------------------------------------------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.setblocking(False)

        self.get_logger().info(
            f"[UDP] Listening on {self.udp_ip}:{self.udp_port}"
        )

        # 10ms timer loop
        self.timer = self.create_timer(0.01, self.read_udp)

    # --------------------------------------------------------
    # UDP Receive Loop
    # --------------------------------------------------------
    def read_udp(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            msg = data.decode().strip()

            self.get_logger().info(f"[UDP] Received from {addr}: {msg}")

            # Trigger matched
            if msg == self.trigger_word:
                ros_msg = String()
                ros_msg.data = self.publish_word

                self.publisher.publish(ros_msg)
                self.get_logger().info(
                    f"[ROS2] Published '{self.publish_word}' → {self.topic_name}"
                )

        except BlockingIOError:
            # No data received → Ignore
            pass
        except Exception as e:
            self.get_logger().error(f"[UDP ERROR] {e}")



def main(args=None):
    rclpy.init(args=args)

    node = UDPToROS2Bridge()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
