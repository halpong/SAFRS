#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import zmq
import serial
import time
import glob


class MotorSerialNode(Node):

    # =======================================================
    # Constructor
    # =======================================================
    def __init__(self):
        super().__init__("motor_serial_node")

        # ------------------------------
        # Load Parameters
        # ------------------------------
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("cmd_sub_port", 5003)
        self.declare_parameter("enc_pub_port", 5002)

        baud = self.get_parameter("baudrate").value
        cmd_port = self.get_parameter("cmd_sub_port").value
        enc_port = self.get_parameter("enc_pub_port").value

        # ------------------------------
        # Serial connection: auto-detect
        # ------------------------------
        self.ser = self.connect_arduino(baud)

        # ------------------------------
        # ZMQ: CMD SUB
        # ------------------------------
        ctx = zmq.Context.instance()
        self.cmd_sub = ctx.socket(zmq.SUB)
        self.cmd_sub.connect(f"tcp://127.0.0.1:{cmd_port}")
        self.cmd_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.get_logger().info(f"[ZMQ] CMD SUB {cmd_port}")

        # ------------------------------
        # ZMQ: Encoder PUB
        # ------------------------------
        self.enc_pub = ctx.socket(zmq.PUB)
        self.enc_pub.bind(f"tcp://*:{enc_port}")
        self.get_logger().info(f"[ZMQ] ENC PUB {enc_port}")

        # Timer
        self.create_timer(0.005, self.loop_once)

        self.get_logger().info("[MotorSerialNode] Node initialized")

    # =======================================================
    # Arduino connection auto-detect
    # =======================================================
    def connect_arduino(self, baud):
        while True:
            ports = glob.glob("/dev/ttyACM*")
            if not ports:
                self.get_logger().warn("[Serial] No Arduino detected")
                time.sleep(1)
                continue

            port = ports[0]
            try:
                ser = serial.Serial(port, baud, timeout=0.1)
                self.get_logger().info(f"[Serial] Connected → {port}")
                return ser
            except Exception as e:
                self.get_logger().error(f"[Serial] Connect failed: {e}")
                time.sleep(1)

    # =======================================================
    # Main loop: send CMD, read encoder
    # =======================================================
    def loop_once(self):

        # ----------------------
        # Read ZMQ command
        # ----------------------
        try:
            cmd = self.cmd_sub.recv_string(flags=zmq.NOBLOCK)
            self.ser.write((cmd + "\n").encode())
        except zmq.Again:
            pass
        except Exception as e:
            self.get_logger().warn(f"[Serial] Write error: {e}")
            self.ser = self.connect_arduino(self.ser.baudrate)

        # ----------------------
        # Read encoder
        # ----------------------
        try:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line.startswith("LF:") and "RF:" in line:
                self.enc_pub.send_string(line)
        except Exception as e:
            self.get_logger().warn(f"[Serial] Read error: {e}")
            self.ser = self.connect_arduino(self.ser.baudrate)


def main(args=None):
    rclpy.init(args=args)
    node = MotorSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
