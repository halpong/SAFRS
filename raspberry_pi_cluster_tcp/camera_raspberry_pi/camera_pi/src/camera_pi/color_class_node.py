#!/usr/bin/env python3
# ===============================================================
# CAMERA PI - COLOR CLASS NODE
# Standard SAFRS Cluster Version
# - Config-based parameter loading
# - Ally/enemy color classification (MobileNet + heuristic)
# - Pose-based upper-body ROI extraction
# - STBY 3-sec confirmation logic
# - TRACK mode error_angle output
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
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

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
# Utility: Load YAML
# ===============================================================
def load_yaml(relative_path):
    pkg_dir = get_package_share_directory("camera_pi")
    full = os.path.join(pkg_dir, "config", relative_path)
    with open(full, "r") as f:
        return yaml.safe_load(f)


# ===============================================================
# Softmax
# ===============================================================
def softmax(x):
    x = x.astype(np.float32)
    e = np.exp(x - np.max(x))
    return e / np.sum(e)


# ===============================================================
# Auto camera open
# ===============================================================
def auto_open_camera(max_try=10):
    for idx in range(max_try):
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            return cap
        cap.release()
    return None


# ===============================================================
# Crop upper body area
# ===============================================================
def crop_upper_body(frame, kpts, margin=30):
    try:
        Ls, Rs, Lh, Rh = kpts[11], kpts[12], kpts[23], kpts[24]
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
# ===============================================================
class ColorClassifier:

    def __init__(self, model_path, input_size):
        self.interpreter = Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.in_idx = self.input_details[0]["index"]
        self.out_idx = self.output_details[0]["index"]

        self.input_size = input_size

        _, self.in_zero = self.input_details[0]["quantization"]

    def classify(self, roi_bgr):

        if roi_bgr is None or roi_bgr.size == 0:
            return "unknown", 0.0

        img = cv2.resize(roi_bgr, self.input_size).astype(np.float32) / 255.0
        img = np.expand_dims(img, axis=0)

        self.interpreter.set_tensor(self.in_idx, img)
        self.interpreter.invoke()

        logits = self.interpreter.get_tensor(self.out_idx)[0]
        prob = softmax(logits)

        cls = int(np.argmax(prob))
        conf = float(prob[cls])
        label = "ally" if cls == 0 else "enemy"

        if conf < 0.55:
            return "unknown", conf

        return label, conf


# ===============================================================
# Load pose estimator
# ===============================================================
def load_pose_estimator(model_path):
    BaseOptions = python.BaseOptions
    PoseOptions = vision.PoseLandmarkerOptions
    RunningMode = vision.RunningMode

    opt = PoseOptions(
        base_options=BaseOptions(model_asset_path=model_path),
        running_mode=RunningMode.IMAGE,
        num_poses=1
    )
    return vision.PoseLandmarker.create_from_options(opt)


# ===============================================================
# CAMERA COLOR NODE
# ===============================================================
class ColorClassNode(Node):

    def __init__(self):
        super().__init__("color_class_node")

        # --------------------------------------------------------------
        # Load configs
        # --------------------------------------------------------------
        cam_cfg = load_yaml("camera_params.yaml")["camera"]
        infer_cfg = load_yaml("inference_params.yaml")["inference"]

        self.conf_threshold = infer_cfg["confidence_threshold"]

        pkg_dir = get_package_share_directory("camera_pi")
        model_dir = os.path.join(pkg_dir, "model")

        pose_path = os.path.join(model_dir, "pose_landmarker_lite.task")
        cls_path = os.path.join(model_dir, "mobilenetv2_ally_enemy_int8.tflite")

        self.pose_estimator = load_pose_estimator(pose_path)
        self.classifier = ColorClassifier(cls_path, (224, 224))

        self.device_w = cam_cfg["resolution_width"]
        self.device_h = cam_cfg["resolution_height"]

        # --------------------------------------------------------------
        # FSM Variables
        # --------------------------------------------------------------
        self.mode = "nav"
        self.cap = None

        self.detect_count = 0
        self.last_label = None
        self.last_time = 0

        self.sent_ally = False
        self.sent_enemy = False
        self.track_target = None

        # --------------------------------------------------------------
        # ROS topics
        # --------------------------------------------------------------
        self.sub_mode = self.create_subscription(String, "/mode", self.cb_mode, 10)
        self.pub_status = self.create_publisher(String, "/a_target_status", 10)
        self.pub_error = self.create_publisher(Vector3, "/c_error_angle", 10)

        self.timer = self.create_timer(0.066, self.loop)

        self.get_logger().info("[INIT] ColorClassNode started.")

    # --------------------------------------------------------------
    # MODE callback
    # --------------------------------------------------------------
    def cb_mode(self, msg):
        prev = self.mode
        new = msg.data.strip().lower()

        if new == "nav":
            self.reset_state()

        self.mode = new
        self.get_logger().info(f"[MODE] {prev} → {self.mode}")

    # --------------------------------------------------------------
    # Reset internal state
    # --------------------------------------------------------------
    def reset_state(self):
        self.detect_count = 0
        self.last_time = 0
        self.last_label = None
        self.track_target = None
        self.sent_ally = False
        self.sent_enemy = False

        if self.cap:
            self.cap.release()
            self.cap = None

        cv2.destroyAllWindows()

    # --------------------------------------------------------------
    # MAIN LOOP
    # --------------------------------------------------------------
    def loop(self):

        if self.mode == "nav":
            return

        # Reconnect camera if needed
        if self.cap is None:
            self.cap = auto_open_camera()
            if self.cap is None:
                self.get_logger().warn("[CAM] Camera not available.")
                return

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.device_w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.device_h)

        ok, frame = self.cap.read()
        if not ok:
            self.cap.release()
            self.cap = None
            return

        H, W = frame.shape[:2]

        # Run pose estimation
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        pose_result = self.pose_estimator.detect(mp_img)

        if not pose_result.pose_landmarks:
            self.display_frame(frame)
            return

        kpts = np.array([[lm.x * W, lm.y * H, lm.z]
                         for lm in pose_result.pose_landmarks[0]])

        roi, box = crop_upper_body(frame, kpts)
        if roi is None:
            self.display_frame(frame)
            return

        label, conf = self.classifier.classify(roi)
        if conf < self.conf_threshold:
            label = "unknown"

        self.get_logger().info(f"[DETECT] {label} ({conf:.2f})")

        # ------------------------------- TRACK Mode -------------------------------
        if self.mode == "track":
            self.process_track(frame, box, label, W, H)

        # ------------------------------- STBY Mode --------------------------------
        elif self.mode == "stby":
            self.process_stby(label)

        # Draw ROI
        self.draw_ui(frame, box, label, conf)
        self.display_frame(frame)

    # --------------------------------------------------------------
    # STBY MODE
    # --------------------------------------------------------------
    def process_stby(self, label):

        if label not in ("ally", "enemy"):
            return

        # 1초마다 count
        now = time.time()
        if now - self.last_time < 1.0:
            return

        self.last_time = now

        if self.last_label == label:
            self.detect_count += 1
        else:
            self.last_label = label
            self.detect_count = 1

        self.get_logger().info(f"[COUNT] {label}: {self.detect_count}/3")

        # 3회 검출 → 보고
        if self.detect_count >= 3:
            self.pub_status.publish(String(data=label))

            if label == "ally":
                self.sent_ally = True
            else:
                self.sent_enemy = True

            self.detect_count = 0

    # --------------------------------------------------------------
    # TRACK MODE
    # --------------------------------------------------------------
    def process_track(self, frame, box, label, W, H):

        if self.track_target is None:
            if label in ("ally", "enemy"):
                self.track_target = label
                self.get_logger().info(f"[TRACK] Target fixed → {label}")
            else:
                return

        if label != self.track_target:
            return

        x1, y1, x2, y2 = box
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        err_y = float(cx - W // 2)
        err_p = float(cy - H // 2)

        self.pub_error.publish(Vector3(x=err_y, y=err_p, z=0.0))

    # --------------------------------------------------------------
    def draw_ui(self, frame, box, label, conf):
        if box is None:
            return

        x1, y1, x2, y2 = box
        if label == "ally":
            c = (255, 0, 0)
        elif label == "enemy":
            c = (0, 0, 255)
        else:
            c = (0, 255, 255)

        cv2.rectangle(frame, (x1, y1), (x2, y2), c, 3)
        cv2.putText(frame, f"{label} {conf:.2f}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, c, 2)

    # --------------------------------------------------------------
    def display_frame(self, frame):
        if self.mode in ("stby", "track"):
            cv2.imshow("Color Class View", frame)
            cv2.waitKey(1)

    # --------------------------------------------------------------
    def destroy_node(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


# ===============================================================
# MAIN ENTRY
# ===============================================================
def main(args=None):
    rclpy.init(args=args)
    node = ColorClassNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
