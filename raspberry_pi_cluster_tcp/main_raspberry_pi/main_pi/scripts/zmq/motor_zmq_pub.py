#!/usr/bin/env python3
"""
SAFRS Main Pi - ZMQ Bridge Module
Module: MotorZMQPublisher
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from main_pi.scripts.utils.zmq_utils import create_publisher
from main_pi.scripts.utils.param_utils import declare_and_get


class MotorZMQPublisher(Node):

    def __init__(self):
        super().__init__("motor_zmq_publisher")

        port = declare_and_get(self, "motor_pub_port", 5000)
        self.pub = create_publisher(port)
        self.get_logger().info(f"[MotorZMQ] Publishing → tcp://*:{port}")

        self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_callback,
            10,
        )

    # --------------------------------------------------------------
    def cmd_callback(self, msg: Twist):
        data = {"lx": float(msg.linear.x), "az": float(msg.angular.z)}

        try:
            self.pub.send_json(data)
        except Exception as e:
            self.get_logger().error(f"[MotorZMQ] Send failed: {e}")
            return

        self.get_logger().debug(f"[MotorZMQ] Sent {data}")


def main():
    rclpy.init()
    node = MotorZMQPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
