#!/usr/bin/env python3
"""
SAFRS Motor Raspberry Pi (C-Pi)
STM32 Controller Node (cmd_vel → UART, ODOM/IMU → ROS2)

- Subscribes:
    /cmd_vel          (geometry_msgs/Twist)
    /mode             (std_msgs/String: NAV / STBY / TRACK)
    /c_error_angle    (geometry_msgs/Vector3: yaw/pitch error from A-Pi)
    /a_target_status  (std_msgs/String: "ally" / "enemy")

- Publishes:
    /odom             (nav_msgs/Odometry)
    /imu/data         (sensor_msgs/Imu)
    /status           (std_msgs/String)
    /servo_stabilized (std_msgs/String, e.g. "ally_stabilized")
    /c_trigger_done   (std_msgs/String, e.g. "enemy_done")
    /mode             (std_msgs/String, re-broadcast / override)

Parameters are loaded from:
- config/serial_params.yaml
- config/odom_params.yaml
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

import serial


class STMControllerNode(Node):
    """Motor + Gimbal + Target FSM controller for C-Pi."""

    def __init__(self) -> None:
        super().__init__("stm_controller_node")

        # ================================================================
        # 1) Parameters (loaded via YAML)
        # ================================================================
        # --- Serial settings ---
        self.declare_parameter("serial.port", "/dev/ttyACM0")
        self.declare_parameter("serial.baudrate", 115200)
        self.declare_parameter("serial.timeout_ms", 5)

        serial_port = self.get_parameter("serial.port").value
        serial_baud = int(self.get_parameter("serial.baudrate").value)
        timeout_ms = int(self.get_parameter("serial.timeout_ms").value)
        serial_timeout = timeout_ms / 1000.0

        # --- Motor mapping ---
        self.declare_parameter("motor.min_pwm", 70)
        self.declare_parameter("motor.max_pwm", 255)
        self.declare_parameter("motor.min_rpm", 10)
        self.declare_parameter("motor.max_rpm", 330)

        self.pwm_min = int(self.get_parameter("motor.min_pwm").value)
        self.pwm_max = int(self.get_parameter("motor.max_pwm").value)
        self.rpm_min = float(self.get_parameter("motor.min_rpm").value)
        self.rpm_max = float(self.get_parameter("motor.max_rpm").value)

        # --- Robot geometry (for cmd_vel → wheel velocity) ---
        # These should match odom_params.yaml on STM32 side.
        self.declare_parameter("odom.wheel_radius", 0.033)  # meters
        self.declare_parameter("odom.wheel_base", 0.19)     # meters

        self.wheel_radius = float(
            self.get_parameter("odom.wheel_radius").value
        )
        self.wheel_base = float(
            self.get_parameter("odom.wheel_base").value
        )

        # ================================================================
        # 2) Subscribers
        # ================================================================
        # Driving velocity command
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)

        # Mode control (NAV / STBY / TRACK)
        self.create_subscription(String, "/mode", self.mode_cb, 10)

        # Gimbal tracking error from A-Pi
        self.create_subscription(
            Vector3,
            "/c_error_angle",
            self.cam_error_cb,
            10,
        )

        # Target status from A-Pi: "ally" / "enemy"
        self.create_subscription(
            String,
            "/a_target_status",
            self.target_status_cb,
            10,
        )

        # ================================================================
        # 3) Publishers
        # ================================================================
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.status_pub = self.create_publisher(String, "/status", 10)

        # Ally stabilization notification (A-Pi subscribes)
        self.servo_pub = self.create_publisher(
            String,
            "/servo_stabilized",
            10,
        )

        # Enemy trigger completion (A/D-Pi subscribe)
        self.trigger_done_pub = self.create_publisher(
            String,
            "/c_trigger_done",
            10,
        )

        # Mode rebroadcast / override
        self.mode_pub = self.create_publisher(String, "/mode", 10)

        # ================================================================
        # 4) Internal state
        # ================================================================
        self.mode = "nav"
        self.motor_enabled = True
        self.gimbal_enabled = False
        self.tracking_enabled = False

        # Tracking error (A → C)
        self.yaw_err = 0.0
        self.pitch_err = 0.0

        # Encoder log throttling
        self.last_enc_log_time = time.time()
        self.enc_log_interval = 0.5

        # Current tracking target: "ally" / "enemy"
        self.target_class = None

        # ================================================================
        # 5) Serial port init
        # ================================================================
        try:
            self.ser = serial.Serial(
                serial_port,
                serial_baud,
                timeout=serial_timeout,
            )
            self.get_logger().info(
                f"[C] Serial connected to STM32 at {serial_port} "
                f"({serial_baud} bps)"
            )
        except Exception as e:
            self.get_logger().error(f"[C] Serial open error: {e}")
            self.ser = None

        # UART RX thread
        if self.ser is not None:
            self.rx_thread = threading.Thread(
                target=self.uart_rx_loop,
                daemon=True,
            )
            self.rx_thread.start()

        self.get_logger().info(
            f"[C] STMControllerNode started with mode='{self.mode}'"
        )

    # ===================================================================
    # Target status from A-Pi ("ally" / "enemy")
    # ===================================================================
    def target_status_cb(self, msg: String) -> None:
        data = msg.data.strip().lower()
        if data in ("ally", "enemy"):
            self.target_class = data
            self.get_logger().info(
                f"[C] Target status from A-Pi: {self.target_class}"
            )
        else:
            self.get_logger().warn(
                f"[C] Unknown target status received: {data}"
            )

    # ===================================================================
    # Mode callback: NAV / STBY / TRACK
    # ===================================================================
    def mode_cb(self, msg: String) -> None:
        new_mode = msg.data.strip().lower()
        prev_mode = self.mode
        self.mode = new_mode

        self.get_logger().info(f"[C] Mode: {prev_mode} → {self.mode}")

        if self.mode == "nav":
            self.motor_enabled = True
            self.gimbal_enabled = False
            self.tracking_enabled = False
            self.send_stop()
            self.get_logger().info(
                "[C] NAV: Motor ON | Gimbal OFF | Tracking OFF"
            )

        elif self.mode == "stby":
            self.motor_enabled = False
            self.gimbal_enabled = True
            self.tracking_enabled = False
            self.send_stop()
            self.get_logger().info(
                "[C] STBY: Motor OFF | Gimbal ON | Tracking OFF"
            )

        elif self.mode == "track":
            self.motor_enabled = False
            self.gimbal_enabled = False
            self.tracking_enabled = True
            self.send_stop()
            self.get_logger().info(
                "[C] TRACK: Motor OFF | Gimbal OFF | Tracking ON"
            )

            # Send current target info to STM32
            if self.target_class in ("ally", "enemy"):
                cmd = f"TARGET {self.target_class.upper()}"
                self.send(cmd)
                self.get_logger().info(f"[C] UART → {cmd}")
            else:
                self.get_logger().warn(
                    "[C] TRACK mode entered but target_class is not set"
                )

        else:
            self.get_logger().warn(
                f"[C] Unknown mode received: {self.mode}"
            )

    # ===================================================================
    # /cmd_vel callback → compute wheel commands → M p1 p2 p3 p4
    # ===================================================================
    def cmd_cb(self, msg: Twist) -> None:
        if not self.motor_enabled:
            return

        vx = msg.linear.x
        wz_raw = msg.angular.z

        # Reduce rotation gain while moving forward/backward
        if abs(vx) > 0.1:
            wz_gain = 1.2
        else:
            wz_gain = 3.0
        wz = wz_raw * wz_gain

        # Differential drive linear velocity per side
        r = self.wheel_radius
        l = self.wheel_base

        v_left = vx - (wz * l / 2.0)
        v_right = vx + (wz * l / 2.0)

        def ms_to_rpm(v: float) -> float:
            return (v / (2.0 * math.pi * r)) * 60.0

        rpm_left = ms_to_rpm(v_left)
        rpm_right = ms_to_rpm(v_right)

        # RPM → PWM mapping
        p_left = self._rpm_to_pwm_signed(rpm_left)
        p_right = self._rpm_to_pwm_signed(rpm_right)

        # 4WD: RR / LR / RF / LF
        p1 = p_left   # Rear Left or Right (depending on wiring)
        p2 = p_left
        p3 = p_right
        p4 = p_right

        cmd = f"M {p1} {p2} {p3} {p4}"
        self.send(cmd)
        self.get_logger().info(f"[C] UART → {cmd}")

    def _rpm_to_pwm_signed(self, rpm: float) -> int:
        """Convert signed RPM to signed PWM considering deadzone."""
        sign = 1 if rpm >= 0.0 else -1
        rpm_abs = abs(rpm)

        if rpm_abs < self.rpm_min:
            return 0

        span_rpm = max(self.rpm_max - self.rpm_min, 1.0)
        span_pwm = max(self.pwm_max - self.pwm_min, 1)

        pwm = self.pwm_min + (rpm_abs - self.rpm_min) * span_pwm / span_rpm
        pwm = int(min(pwm, self.pwm_max))

        return sign * pwm

    # ===================================================================
    # /c_error_angle callback: tracking error from A-Pi (yaw, pitch)
    # ===================================================================
    def cam_error_cb(self, msg: Vector3) -> None:
        if not self.tracking_enabled:
            return

        self.yaw_err = float(msg.x)
        self.pitch_err = float(msg.y)

        # Error evaluation is done on STM32 side
        cmd = f"TRACK {self.yaw_err:.2f} {self.pitch_err:.2f}"
        self.send(cmd)
        self.get_logger().info(f"[C] UART → {cmd}")

    # ===================================================================
    # UART send helpers
    # ===================================================================
    def send(self, text: str) -> None:
        if self.ser is None:
            return
        try:
            self.ser.write((text + "\n").encode("utf-8"))
        except Exception:
            self.get_logger().error("[C] UART TX error")

    def send_stop(self) -> None:
        self.send("M 0 0 0 0")

    # ===================================================================
    # UART RX loop (STM32 → C-Pi)
    # ===================================================================
    def uart_rx_loop(self) -> None:
        buffer = ""

        while rclpy.ok():
            try:
                ch = self.ser.read().decode("utf-8", errors="ignore")
                if not ch:
                    continue

                if ch == "\n":
                    line = buffer.strip()
                    buffer = ""
                    if line:
                        self.parse_stm32_msg(line)
                else:
                    buffer += ch

            except Exception as e:
                self.get_logger().error(f"[C] UART RX error: {e}")
                time.sleep(0.1)

    # ===================================================================
    # Parse STM32 messages
    # ===================================================================
    def parse_stm32_msg(self, line: str) -> None:
        # ------------------ ODOM ------------------
        if line.startswith("ODOM"):
            try:
                _, x, y, th, vx, wz = line.split(",")
                msg = Odometry()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "odom"
                msg.child_frame_id = "base_link"

                msg.pose.pose.position.x = float(x)
                msg.pose.pose.position.y = float(y)
                msg.pose.pose.orientation.z = float(th)  # optional: use yaw→quat

                msg.twist.twist.linear.x = float(vx)
                msg.twist.twist.angular.z = float(wz)

                self.odom_pub.publish(msg)
            except Exception:
                self.get_logger().warn(
                    f"[C] Failed to parse ODOM line: {line}"
                )

        # ------------------- IMU -------------------
        elif line.startswith("IMU"):
            try:
                _, qx, qy, qz, qw, gx, gy, gz = line.split(",")
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "base_link"

                msg.orientation.x = float(qx)
                msg.orientation.y = float(qy)
                msg.orientation.z = float(qz)
                msg.orientation.w = float(qw)

                msg.angular_velocity.x = float(gx)
                msg.angular_velocity.y = float(gy)
                msg.angular_velocity.z = float(gz)

                self.imu_pub.publish(msg)
            except Exception:
                self.get_logger().warn(
                    f"[C] Failed to parse IMU line: {line}"
                )

        # ---------------- STATUS MSG ---------------
        elif line.startswith("STATUS"):
            msg = String()
            msg.data = line[7:]
            self.status_pub.publish(msg)

        # ---------------- ENCODERS -----------------
        elif line.startswith("E1:"):
            try:
                parts = line.replace(" ", "").split(",")
                e1 = int(parts[0].split(":")[1])
                e2 = int(parts[1].split(":")[1])
                e3 = int(parts[2].split(":")[1])
                e4 = int(parts[3].split(":")[1])

                now = time.time()
                if now - self.last_enc_log_time >= self.enc_log_interval:
                    self.get_logger().info(
                        f"[C] Encoders: {e1}, {e2}, {e3}, {e4}"
                    )
                    self.last_enc_log_time = now
            except Exception:
                self.get_logger().warn(
                    f"[C] Failed to parse encoder line: {line}"
                )

        # ---------------- ENEMY 완료 ----------------
        elif line.startswith("TRIGGER_DONE"):
            self.get_logger().info(
                "[C] Enemy trigger complete (TRIGGER_DONE)"
            )
            self.trigger_done_pub.publish(String(data="enemy_done"))
            # Enemy done → switch to STBY to search for ally
            self.mode_pub.publish(String(data="stby"))

        # ---------------- ALLY 완료 -----------------
        elif line.startswith("ALLY_OK"):
            self.get_logger().info("[C] Ally stabilized (ALLY_OK)")
            self.servo_pub.publish(String(data="ally_stabilized"))
            # Ally done → NAV back
            self.mode_pub.publish(String(data="nav"))

        else:
            # Unknown line type → optional debug
            self.get_logger().debug(f"[C] RX: {line}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = STMControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
