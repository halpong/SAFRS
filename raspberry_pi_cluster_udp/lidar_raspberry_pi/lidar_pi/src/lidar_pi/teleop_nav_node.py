#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import sys
import tty
import termios


HELP = """
-------------------------------
     TELEOP NAVIGATION
-------------------------------
w : forward
x : backward
a : rotate left
d : rotate right
u : forward-left
o : forward-right
s : stop
q : quit
-------------------------------
"""


class TeleopNav(Node):
    def __init__(self):
        super().__init__('teleop_nav')

        # publisher to D (or C if remapped)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)

        # mode subscriber
        self.mode_sub = self.create_subscription(
            String, 'mode', self.mode_cb, 10
        )

        # ⭐ NEW: ODOM subscriber
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10
        )

        self.mode = "nav"

        print(HELP)

    # new
    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z

        self.get_logger().info(
            f"[B ODOM] x={x:.2f}, y={y:.2f}, vx={vx:.2f}, wz={wz:.2f}"
        )

    def mode_cb(self, msg):
        self.mode = msg.data
        self.get_logger().info(f"[B] Mode changed → {self.mode}")

    def get_key(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def teleop_loop(self):
        try:
            while rclpy.ok():
                key = self.get_key()

                if key == 'q':
                    self.publish_stop()
                    break

                if self.mode != "nav":
                    print("[B] NAV 모드 아님 → STOP")
                    self.publish_stop()
                    continue

                twist = Twist()

                if key == 'w':
                    twist.linear.x = 0.25
                elif key == 'x':
                    twist.linear.x = -0.25
                elif key == 'a':
                    twist.angular.z = 0.9
                elif key == 'd':
                    twist.angular.z = -0.9
                elif key == 's':
                    twist = Twist()
                elif key == 'u':
                    twist.linear.x = 0.25
                    twist.angular.z = 0.7
                elif key == 'o':
                    twist.linear.x = 0.25
                    twist.angular.z = -0.7
                else:
                    twist = Twist()

                self.cmd_pub.publish(twist)

        except Exception as e:
            print("[EXCEPTION]", e)
        finally:
            self.publish_stop()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNav()

    rclpy.spin_once(node, timeout_sec=0)
    node.teleop_loop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
