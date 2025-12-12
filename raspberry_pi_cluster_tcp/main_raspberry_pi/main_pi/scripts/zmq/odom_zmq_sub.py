#!/usr/bin/env python3
"""
SAFRS Main Pi - ZMQ Bridge Module
Module: OdomZMQSubscriber
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

from main_pi.scripts.utils.zmq_utils import create_subscriber
from main_pi.scripts.utils.param_utils import declare_and_get
from main_pi.scripts.utils.math_utils import yaw_to_quaternion


class OdomZMQSubscriber(Node):

    def __init__(self):
        super().__init__("odom_zmq_subscriber")

        ip = declare_and_get(self, "motor_ip", "172.30.1.133")
        port = declare_and_get(self, "motor_port", 5001)

        self.sub = create_subscriber(ip, port)
        self.get_logger().info(f"[OdomZMQ] Listening → tcp://{ip}:{port}")

        self.pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.02, self.receive)

    # --------------------------------------------------------------
    def receive(self):
        try:
            raw = self.sub.recv_string(flags=1)
        except Exception:
            return

        try:
            x, y, th = map(float, raw.split())
        except:
            return

        q = yaw_to_quaternion(th)

        # -------------------- Odometry Message --------------------
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = q

        self.pub.publish(odom)

        # -------------------- TF Transform ------------------------
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w

        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = OdomZMQSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
