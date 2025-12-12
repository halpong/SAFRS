#!/usr/bin/env python3
# ===============================================================
# CAMERA PI - CAMERA NODE
# Standardized SAFRS Cluster Version
# - Config-based parameter loading
# - Clean FSM (nav / stby / track)
# - Color classification + pose detection
# - Error angle publishing (TRACK mode)
# - Ally/enemy 3-sec confirmation (STBY mode)
#
# ===============================================================

import time
import platform
import cv2
import numpy as np
import mediapipe as mp

from mediapipe.tasks import python
from mediapipe.tasks.python import vision

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory
import yaml
import os


# ===============================================================
# Interpreter selection
# ===============================================================
if platform.system() == "Windows":
    import tensorflow as tf
    Interpreter = tf.lite.Interpreter
else:
    import tflite_runtime.interpreter as tflite
    Interpreter = tflite.Interpreter


# ===============================================================
# Utility: Load YAML file
# ===============================================================
def load_yaml(relative_path):
    pkg_dir = get_package_share_directory("camera_pi")
    full = os.path.join(pkg_dir, "config", relative_path)
    with open(full, "r") as f:
        return yaml.safe_load(f)


# ===============================================================
# Helper: Softmax
# ===============================================================
def softmax(x):
    x = x.astype(np.float32)
    e = np.exp(x - np.max(x))
    return e / np.sum(e)


# ===============================================================
# Camera auto-detect
# ===============================================================
def auto_open_camera(device_try=10):
    for idx in range(device_try):
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if cap.isOpened():
            # MJPG pipeline
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            return cap
        cap.release()
    return None


# ===============================================================
# Crop upper body
# ===============================================================
def crop_upper_body(frame, keypoints, margin=30):
    try:
        Ls, Rs, Lh, Rh = keypoints[11], keypoints[12], keypoints[23], keypoints[24]

        xs = np.array([Ls[0], Rs[0], Lh[0], Rh[0]])
        ys = np.array([Ls[1], Rs[1], Lh[1], Rh[1]])

        x1 = int(max(xs.min() - margin, 0))
        y1 = int(max(ys.min() - margin, 0))
        x2 = int(min(xs.max() + margin, frame.shape[1]))
        y2 = int(min(ys.max() + margin, frame.shape[0]))

        if x2 <= x1 or y2 <= y1:
            return None, None
        return frame[y1:y2, x1:x2], (x1, y1, x2, y2)
    except:
        return None, None


# ===============================================================
# Color classifier wrapper
# (Simple version: mobileNet only)
# ===============================================================
class ColorClassifier:
    def __init__(self, model_path, input_size):
        self.interpreter = Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.in_idx = self.input_details[0]["index"]
        self.out_idx = self.output_details[0]["index"]

        _, _, _, self.w = self.input_details[0]["shape"]
        self.input_size = input_size

        _, self.in_zero = self.input_details[0]["quantization"]
        _, self.out_zero = self.output_details[0]["quantization"]

    def classify(self, roi):
        if roi is None or roi.size == 0:
            return "unknown", 0.0

        img = cv2.resize(roi, self.input_size).astype(np.float32) / 255.0
        img = np.expand_dims(img, 0)

        img_q = img.astype(np.float32)
        self.interpreter.set_tensor(self.in_idx, img_q)
        self.interpreter.invoke()

        logits = self.interpreter.get_tensor(self.out_idx)[0]
        prob = softmax(logits)

        cls = int(np.argmax(prob))
        label = "ally" if cls == 0 else "enemy"
        conf = float(prob[cls])

        if conf < 0.55:
            return "unknown", conf
        return label, conf


# ===============================================================
# PoseLandmarker loader
# ===============================================================
def load_pose_estimator(model_path):
    BaseOptions = python.BaseOptions
    PoseOptions = vision.PoseLandmarkerOptions
    RunningMode = vision.RunningMode

    opts = PoseOptions(
        base_options=BaseOptions(model_asset_path=model_path),
        running_mode=RunningMode.IMAGE,
        num_poses=1
    )
    return vision.PoseLandmarker.create_from_options(opts)


# ===============================================================
# CAMERA NODE
# ===============================================================
class CameraNode(Node):

    def __init__(self):
        super().__init__("camera_node")

        # -------------------------------------------------------
        # Load config files
        # -------------------------------------------------------
        cam_cfg = load_yaml("camera_params.yaml")["camera"]
        infer_cfg = load_yaml("inference_params.yaml")["inference"]

        # Camera parameters
        self.device_w = cam_cfg["resolution_width"]
        self.device_h = cam_cfg["resolution_height"]

        # Inference parameters
        self.conf_threshold = infer_cfg["confidence_threshold"]

        # -------------------------------------------------------
        # Load models
        # -------------------------------------------------------
        pkg_dir = get_package_share_directory("camera_pi")
        model_dir = os.path.join(pkg_dir, "model")

        pose_path = os.path.join(model_dir, "pose_landmarker_lite.task")
        cls_path = os.path.join(model_dir, "mobilenetv2_ally_enemy_int8.tflite")

        self.pose_estimator = load_pose_estimator(pose_path)
        self.classifier = ColorClassifier(cls_path, (224, 224))

        # -------------------------------------------------------
        # State machine variables
        # -------------------------------------------------------
        self.mode = "nav"
        self.cap = None

        self.last_label = None
        self.detect_count = 0
        self.last_time = 0

        self.sent_ally = False
        self.sent_enemy = False

        self.track_target = None
        self.error_send_count = 0
        self.MAX_ERROR_SEND = 5

        # -------------------------------------------------------
        # ROS Communication
        # -------------------------------------------------------
        self.sub_mode = self.create_subscription(String, "/mode", self.cb_mode, 10)
        self.sub_trig_done = self.create_subscription(String, "/c_trigger_done",
                                                      self.cb_trigger_done, 10)

        self.pub_status = self.create_publisher(String, "/a_target_status", 10)
        self.pub_error = self.create_publisher(Vector3, "/c_error_angle", 10)
        self.pub_mode_cmd = self.create_publisher(String, "/mode", 10)

        # Main loop timer
        self.timer = self.create_timer(0.066, self.loop)

        self.get_logger().info("[INIT] CameraNode started.")

    # -------------------------------------------------------
    # MODE callback
    # -------------------------------------------------------
    def cb_mode(self, msg):
        prev = self.mode
        new = msg.data.strip().lower()

        if prev == "track" and new == "stby":
            self.error_send_count = 0

        if new == "nav":
            self.reset_state()

        self.mode = new
        self.get_logger().info(f"[MODE] {prev} → {self.mode}")

    # -------------------------------------------------------
    # Trigger completion callback
    # -------------------------------------------------------
    def cb_trigger_done(self, msg):
        if self.mode == "track" and self.track_target == "enemy":
            self.track_target = None
            self.error_send_count = 0
            self.pub_mode_cmd.publish(String(data="stby"))

    # -------------------------------------------------------
    # Reset internal state
    # -------------------------------------------------------
    def reset_state(self):
        self.detect_count = 0
        self.last_label = None
        self.last_time = 0
        self.error_send_count = 0
        self.track_target = None

        self.sent_ally = False
        self.sent_enemy = False

        if self.cap:
            self.cap.release()
            self.cap = None
        cv2.destroyAllWindows()

    # -------------------------------------------------------
    # MAIN LOOP
    # -------------------------------------------------------
    def loop(self):

        # NAV : completely stop
        if self.mode == "nav":
            return

        # Open camera if needed
        if self.cap is None:
            self.cap = auto_open_camera()
            if self.cap is None:
                self.get_logger().warn("[CAM] Camera not found.")
                return
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.device_w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.device_h)

        ok, frame = self.cap.read()
        if not ok:
            self.cap.release()
            self.cap = None
            return

        H, W = frame.shape[:2]

        # Pose detection
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        pose_result = self.pose_estimator.detect(mp_image)

        if not pose_result.pose_landmarks:
            self.display_frame(frame)
            return

        # Extract keypoints
        keypoints = np.array([[lm.x * W, lm.y * H, lm.z]
                              for lm in pose_result.pose_landmarks[0]])

        roi, box = crop_upper_body(frame, keypoints)
        if roi is None:
            self.display_frame(frame)
            return

        # Classification
        color, conf = self.classifier.classify(roi)
        if conf < self.conf_threshold:
            color = "unknown"

        self.get_logger().info(f"[DETECT] {color} ({conf:.2f})")

        # -------------------------- TRACK MODE --------------------------
        if self.mode == "track":
            self.process_track_mode(color, box, W, H)
        # -------------------------- STBY MODE ---------------------------
        elif self.mode == "stby":
            self.process_stby_mode(color)

        self.draw_ui(frame, box, color, conf)
        self.display_frame(frame)

    # -------------------------------------------------------
    def process_track_mode(self, color, box, W, H):

        # Fix target class once
        if self.track_target is None:
            if color in ("ally", "enemy"):
                self.track_target = color
                self.error_send_count = 0
                self.get_logger().info(f"[TRACK] Target fixed → {color}")
            else:
                return

        if color != self.track_target:
            return

        # Calculate error
        x1, y1, x2, y2 = box
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        err_yaw = float(cx - W // 2)
        err_pitch = float(cy - H // 2)

        # Limit send count
        if self.error_send_count < self.MAX_ERROR_SEND:
            self.pub_error.publish(Vector3(x=err_yaw, y=err_pitch, z=0.0))
            self.error_send_count += 1

    # -------------------------------------------------------
    def process_stby_mode(self, color):

        if color not in ("ally", "enemy"):
            return

        # Enemy first policy
        if color == "ally" and not self.sent_enemy:
            return

        now = time.time()
        if now - self.last_time < 1.0:
            return

        self.last_time = now

        if self.last_label == color:
            self.detect_count += 1
        else:
            self.last_label = color
            self.detect_count = 1

        self.get_logger().info(f"[COUNT] {color}: {self.detect_count}/3")

        if self.detect_count >= 3:
            # Report result
            self.pub_status.publish(String(data=color))

            # Request TRACK mode
            self.pub_mode_cmd.publish(String(data="track"))
            self.track_target = color

            if color == "ally":
                self.sent_ally = True
            else:
                self.sent_enemy = True

            self.detect_count = 0
            self.error_send_count = 0

            if self.sent_ally and self.sent_enemy:
                self.pub_mode_cmd.publish(String(data="nav"))

    # -------------------------------------------------------
    def draw_ui(self, frame, box, color, conf):
        if box is None:
            return

        x1, y1, x2, y2 = box

        if color == "ally":
            draw = (255, 0, 0)
        elif color == "enemy":
            draw = (0, 0, 255)
        else:
            draw = (0, 255, 255)

        cv2.rectangle(frame, (x1, y1), (x2, y2), draw, 3)
        cv2.putText(frame, f"{color} {conf:.2f}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, draw, 2)

    # -------------------------------------------------------
    def display_frame(self, frame):
        if self.mode in ("stby", "track"):
            cv2.imshow("Camera View", frame)
            cv2.waitKey(1)

    # -------------------------------------------------------
    def destroy_node(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


# ===============================================================
# MAIN
# ===============================================================
def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
