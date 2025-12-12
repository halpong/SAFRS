#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import zmq
import json


class MotorCmdNode(Node):

    # =======================================================
    # Constructor
    # =======================================================
    def __init__(self):
        super().__init__("motor_cmd_node")

        # ------------------------------
        # Load Parameters
        # ------------------------------
        self.declare_parameter("main_pi_ip", "172.30.1.78")
        self.declare_parameter("cmd_sub_port", 5000)
        self.declare_parameter("bridge_pub_port", 5003)

        main_pi_ip = self.get_parameter("main_pi_ip").value
        cmd_port = self.get_parameter("cmd_sub_port").value
        bridge_pub_port = self.get_parameter("bridge_pub_port").value

        # ------------------------------
        # ZMQ: MainPi → MotorPi SUB
        # ------------------------------
        ctx = zmq.Context.instance()
        self.cmd_sub = ctx.socket(zmq.SUB)
        self.cmd_sub.connect(f"tcp://{main_pi_ip}:{cmd_port}")
        self.cmd_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.get_logger().info(f"[ZMQ] SUB connected → tcp://{main_pi_ip}:{cmd_port}")

        # ------------------------------
        # ZMQ: MotorPi → Serial Bridge PUB
        # ------------------------------
        self.bridge_pub = ctx.socket(zmq.PUB)
        self.bridge_pub.bind(f"tcp://*:{bridge_pub_port}")
        self.get_logger().info(f"[ZMQ] CMD → Bridge PUB bind {bridge_pub_port}")

        # Timer loop for polling
        self.create_timer(0.01, self.poll_zmq)

        self.get_logger().info("[MotorCmdNode] Node initialized")

    # =======================================================
    # Poll ZMQ messages
    # =======================================================
    def poll_zmq(self):

        try:
            raw = self.cmd_sub.recv_string(flags=zmq.NOBLOCK)
        except zmq.Again:
            return

        self.get_logger().info(f"[MotorPi] CMD recv: {raw}")

        try:
            data = json.loads(raw)
            lx = data.get("lx", 0.0)
            az = data.get("az", 0.0)
        except Exception:
            return

        # ------------------------------
        # Velocity → motor command map
        # ------------------------------
        if lx > 0.1:
            cmd = "w"
        elif lx < -0.1:
            cmd = "s"
        elif az > 0.1:
            cmd = "a"
        elif az < -0.1:
            cmd = "d"
        else:
            cmd = "x"

        # Publish to Serial Bridge
        self.bridge_pub.send_string(cmd)
        self.get_logger().info(f"[MotorPi] SEND → BRIDGE: {cmd}")


def main(args=None):
    rclpy.init(args=args)
    node = MotorCmdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
