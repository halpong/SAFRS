#!/usr/bin/env python3
"""
SAFRS A-Pi Camera (UDP Event Sender + ROS2 FSM)
"""


import platform
import time
import json
import socket

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

# ============================================================
# 0) Load model interpreter
# ============================================================
if platform.system() == "Windows":
    import tensorflow as tf
    Interpreter = tf.lite.Interpreter
else:
    import tflite_runtime.interpreter as tflite
    Interpreter = tflite.Interpreter

CLASS_MODEL_PATH = "model/mobilenetv2_int8_v2.tflite"
POSE_MODEL_PATH = "model/pose_landmarker_lite.task"
IMG_SIZE = (160, 160)

# Load classifier
interpreter = Interpreter(model_path=CLASS_MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
in_idx = input_details[0]['index']
out_idx = output_details[0]['index']
in_scale, in_zero = input_details[0]["quantization"]

# ============================================================
# 1) Utility Functions
# ============================================================
def softmax(x):
    x = x.astype(np.float32)
    e = np.exp(x - np.max(x))
    return e / np.sum(e)

def classify_color(roi):
    if roi is None or roi.size == 0:
        return "unknown", 0.0

    img = cv2.resize(roi, IMG_SIZE).astype(np.float32)

    if in_scale != 0:
        img = img / in_scale + in_zero
    img = np.expand_dims(img, axis=0).astype(input_details[0]["dtype"])

    interpreter.set_tensor(in_idx, img)
    interpreter.invoke()
    output = interpreter.get_tensor(out_idx)[0]

    # Quantized → float
    if output_details[0]["dtype"] == np.uint8:
        out_scale, out_zero = output_details[0]["quantization"]
        output = (output.astype(np.float32) - out_zero) * out_scale

    prob = softmax(output)

    # label: 0=ally, 1=enemy, 2=other
    if prob[0] > 0.5 and prob[0] > prob[1]:
        return "ally", prob[0]
    elif prob[1] > 0.5 and prob[1] > prob[0]:
        return "enemy", prob[1]
    return "unknown", max(prob)

# ============================================================
# 2) Mediapipe Pose (for ROI crop)
# ============================================================
BaseOptions = python.BaseOptions
PoseLandmarkerOptions = vision.PoseLandmarkerOptions
RunningMode = vision.RunningMode

pose_options = PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=POSE_MODEL_PATH),
    running_mode=RunningMode.IMAGE,
    num_poses=1
)
pose_landmarker = vision.PoseLandmarker.create_from_options(pose_options)

def crop_upper_body(frame, keypoints, margin=25):
    try:
        Ls, Rs, Lh, Rh = keypoints[11], keypoints[12], keypoints[23], keypoints[24]
        xs = np.array([Ls[0], Rs[0], Lh[0], Rh[0]])
        ys = np.array([Ls[1], Rs[1], Lh[1], Rh[1]])

        x1, y1 = int(xs.min() - margin), int(ys.min() - margin)
        x2, y2 = int(xs.max() + margin), int(ys.max() + margin)
        x1, y1 = max(0, x1), max(0, y1)
        x2 = min(x2, frame.shape[1])
        y2 = min(y2, frame.shape[0])

        if x2 <= x1 or y2 <= y1:
            return None, None

        return frame[y1:y2, x1:x2], (x1, y1, x2, y2)
    except:
        return None, None

# ============================================================
# 3) UDP Settings (A → D)
# ============================================================
UDP_IP = "172.30.1.50"   # D-Pi IP
UDP_PORT = 6006
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_udp_event(event_type, conf):
    """Send JSON event to D-Pi."""
    packet = {
        "event": event_type,
        "confidence": float(conf),
        "timestamp": time.time()
    }
    udp_sock.sendto(json.dumps(packet).encode(), (UDP_IP, UDP_PORT))

# ============================================================
# 4) Camera Client Node
# ============================================================
class CameraClientNode(Node):
    def __init__(self):
        super().__init__("camera_udp_client")

        self.mode = "nav"
        self.cap = None

        self.last_color = None
        self.detect_count = 0
        self.last_detect_time = 0.0

        # ROS2 Communication
        self.pub_error = self.create_publisher(Vector3, "/c_error_angle", 10)
        self.pub_status = self.create_publisher(String, "/a_target_status", 10)
        self.sub_mode = self.create_subscription(String, "/mode", self.cb_mode, 10)

        self.timer = self.create_timer(0.06, self.loop)

        self.get_logger().info("[INIT] Camera UDP Client Ready.")

    # ---------------- MODE Callback ----------------
    def cb_mode(self, msg):
        new_mode = msg.data.strip().lower()
        prev = self.mode
        self.mode = new_mode

        if new_mode == "nav":
            self.detect_count = 0
            self.last_color = None
            if self.cap:
                self.cap.release()
                self.cap = None
            cv2.destroyAllWindows()

        self.get_logger().info(f"[MODE] {prev} → {self.mode}")

    # ---------------- Main Loop ----------------
    def loop(self):
        if self.mode == "nav":
            return

        # Open camera if closed
        if self.cap is None:
            for idx in range(5):
                cap = cv2.VideoCapture(idx)
                if cap.isOpened():
                    self.cap = cap
                    break
            if self.cap is None:
                self.get_logger().warn("[CAM] No camera found.")
                return

        ok, frame = self.cap.read()
        if not ok:
            self.cap.release()
            self.cap = None
            return

        h, w = frame.shape[:2]

        # mediapipe
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        result = pose_landmarker.detect(mp_img)

        roi, box = None, None
        if result.pose_landmarks:
            keypoints = np.array([[lm.x * w, lm.y * h, lm.z]
                                  for lm in result.pose_landmarks[0]])
            roi, box = crop_upper_body(frame, keypoints)

        if roi is None:
            return

        color, conf = classify_color(roi)

        # ===================== STBY =====================
        if self.mode == "stby":
            now = time.time()
            if now - self.last_detect_time >= 1.0:
                self.last_detect_time = now

                if color == self.last_color:
                    self.detect_count += 1
                else:
                    self.detect_count = 1
                    self.last_color = color

                # stable detection
                if self.detect_count >= 3 and color in ("ally", "enemy"):
                    # Send UDP event
                    send_udp_event(color, conf)

                    # Publish to ROS
                    self.pub_status.publish(String(data=color))

                    # Switch to track mode
                    self.get_logger().info(f"[SEND] Detected {color}, switching to TRACK")
                    self.create_publisher(String, "/mode", 10).publish(String(data="track"))
                    self.detect_count = 0

        # ===================== TRACK =====================
        if self.mode == "track" and box is not None:
            x1, y1, x2, y2 = box
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            err_yaw = cx - w // 2
            err_pitch = cy - h // 2

            self.pub_error.publish(Vector3(
                x=float(err_yaw),
                y=float(err_pitch),
                z=0.0
            ))

        # Display (debug)
        cv2.imshow("A-Pi Camera", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


# ============================================================
# MAIN
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = CameraClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
