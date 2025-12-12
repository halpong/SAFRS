#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import zmq
import time
import math


class MotorOdomNode(Node):

    # =======================================================
    # Constructor
    # =======================================================
    def __init__(self):
        super().__init__("motor_odom_node")

        # ------------------------------
        # Load Parameters
        # ------------------------------
        self.declare_parameter("encoder_sub_port", 5002)
        self.declare_parameter("odom_pub_port", 5001)

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("ticks_per_rev", 1024)
        self.declare_parameter("wheel_base", 0.18)

        enc_port = self.get_parameter("encoder_sub_port").value
        odom_port = self.get_parameter("odom_pub_port").value

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.ticks_per_rev = self.get_parameter("ticks_per_rev").value
        self.wheel_base = self.get_parameter("wheel_base").value

        # ------------------------------
        # Internal odom state
        # ------------------------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left = None
        self.last_right = None

        # ------------------------------
        # ZMQ: Encoder SUB
        # ------------------------------
        ctx = zmq.Context.instance()
        self.enc_sub = ctx.socket(zmq.SUB)
        self.enc_sub.connect(f"tcp://127.0.0.1:{enc_port}")
        self.enc_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.get_logger().info(f"[ZMQ] ENC SUB {enc_port}")

        # ------------------------------
        # ZMQ: Odom PUB
        # ------------------------------
        self.odom_pub = ctx.socket(zmq.PUB)
        self.odom_pub.bind(f"tcp://*:{odom_port}")
        self.get_logger().info(f"[ZMQ] ODOM PUB {odom_port}")

        # Timer loop
        self.create_timer(0.02, self.process_encoder)

        self.get_logger().info("[MotorOdomNode] Node initialized")

    # =======================================================
    # Convert ticks → meters
    # =======================================================
    def ticks_to_distance(self, ticks):
        rev = ticks / self.ticks_per_rev
        return rev * (2 * math.pi * self.wheel_radius)

    # =======================================================
    # Main encoder → odometry loop
    # =======================================================
    def process_encoder(self):

        try:
            line = self.enc_sub.recv_string(flags=zmq.NOBLOCK)
        except zmq.Again:
            return

        # parse LF RF LR RR
        try:
            parts = line.split()
            lf = int(parts[0].split(":")[1])
            rf = int(parts[1].split(":")[1])
            lr = int(parts[2].split(":")[1])
            rr = int(parts[3].split(":")[1])
        except:
            return

        left = (lf + lr) / 2
        right = (rf + rr) / 2

        # Init only
        if self.last_left is None:
            self.last_left = left
            self.last_right = right
            return

        # Δticks
        dL = left - self.last_left
        dR = right - self.last_right
        self.last_left = left
        self.last_right = right

        # tick → meter
        dL_m = self.ticks_to_distance(dL)
        dR_m = self.ticks_to_distance(dR)

        # diff-drive motion
        dS = (dL_m + dR_m) / 2
        dTheta = (dR_m - dL_m) / self.wheel_base

        self.x += dS * math.cos(self.theta + dTheta / 2)
        self.y += dS * math.sin(self.theta + dTheta / 2)
        self.theta += dTheta

        msg = f"{self.x} {self.y} {self.theta}"
        self.odom_pub.send_string(msg)
        self.get_logger().info(f"[ODOM] {msg}")


def main(args=None):
    rclpy.init(args=args)
    node = MotorOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
