#!/usr/bin/env python3
"""
SAFRS Main Pi (D-Pi)
Main Bridge Node

Role:
- Central command relay
- Mode broadcast controller (NAV / STBY / TRACK)
- cmd_vel_nav → cmd_vel forwarding
- NAV inactivity timeout handling
"""

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MainBridgeNode(Node):
    """
    Main Bridge Node (D-Pi)

    - Receives cmd_vel_nav from B-Pi (teleop / navigation)
    - Forwards cmd_vel to C-Pi (motor controller)
    - Broadcasts mode changes to A/B/C
    - Forces STBY if NAV command timeout occurs
    """

    NAV_TIMEOUT_SEC = 15.0  # seconds

    def __init__(self):
        super().__init__('main_bridge')

        # ===============================
        # Internal State
        # ===============================
        self.mode = "nav"
        self.last_cmd_vel_time = time.time()

        # ===============================
        # Subscribers
        # ===============================
        # B-Pi → Main (teleop / nav command)
        self.sub_cmd_nav = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.cmd_nav_cb,
            10
        )

        # A/B/C → Main (mode input)
        self.sub_mode = self.create_subscription(
            String,
            '/mode',
            self.mode_cb,
            10
        )

        # ===============================
        # Publishers
        # ===============================
        # Main → C-Pi (motor command)
        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Main → A/B/C (mode broadcast)
        self.pub_mode = self.create_publisher(
            String,
            '/mode',
            10
        )

        # ===============================
        # Timers
        # ===============================
        self.timer = self.create_timer(
            0.5,
            self.check_nav_timeout
        )

        self.get_logger().info("[MAIN] Main Bridge Node started (mode=NAV)")

    # ======================================================
    # Mode Input Callback (A/B/C → Main)
    # ======================================================
    def mode_cb(self, msg: String):
        new_mode = msg.data.strip().lower()
        if new_mode != self.mode:
            self.set_mode(new_mode)

    # ======================================================
    # cmd_vel_nav Callback (B-Pi → Main)
    # ======================================================
    def cmd_nav_cb(self, msg: Twist):
        if self.mode == "nav":
            self.pub_cmd_vel.publish(msg)
        self.last_cmd_vel_time = time.time()

    # ======================================================
    # NAV Timeout Checker
    # ======================================================
    def check_nav_timeout(self):
        if self.mode == "nav":
            elapsed = time.time() - self.last_cmd_vel_time
            if elapsed > self.NAV_TIMEOUT_SEC:
                self.get_logger().warn(
                    f"[MAIN] NAV timeout ({elapsed:.1f}s) → STBY"
                )
                self.set_mode("stby")

    # ======================================================
    # Mode Setter (Centralized)
    # ======================================================
    def set_mode(self, new_mode: str):
        if new_mode != self.mode:
            self.mode = new_mode
            self.get_logger().info(f"[MAIN] Mode changed → {new_mode.upper()}")
            self.pub_mode.publish(String(data=new_mode))


# ==========================================================
# Main Entry
# ==========================================================
def main(args=None):
    rclpy.init(args=args)
    node = MainBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
