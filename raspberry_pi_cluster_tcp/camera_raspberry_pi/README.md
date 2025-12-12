# SAFRS Camera Raspberry Pi Module

This module (Camera Pi) is responsible for **Ally/Enemy color classification**,  
**upper-body pose landmark extraction**, and **error-angle tracking** within the  
**SAFRS Cluster Architecture**.

The node receives FSM mode commands from the Main Pi (`nav → stby → track`)  
and optionally transmits error angle values to the Control Pi during TRACK mode.


---

## 📦 Directory Structure

```
camera_pi/
├── camera_pi/
│   ├── __init__.py
│   ├── camera_node.py           # Full camera + pose + color classification node
│   ├── color_class_node.py      # Lightweight color classifier node
│
├── config/
│   ├── camera_params.yaml       # Camera device parameters
│   ├── inference_params.yaml    # TFLite + Landmark inference settings
│   ├── zmq_params.yaml          # SAFRS ZMQ communication parameters
│
├── model/
│   ├── mobilenetv2_ally_enemy_int8.tflite
│   ├── mobilenetv2_int8_v2.tflite
│   ├── pose_landmarker_lite.task
│
├── resource/
│   └── camera_pi
│
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
│
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

---

## 🖥️ Module Description

| File | Purpose |
|------|---------|
| `camera_node.py` | Full camera pipeline: pose landmarking, color classification, FSM logic, tracking |
| `color_class_node.py` | Lightweight ally/enemy classifier (MobileNet) |
| `camera_params.yaml` | Camera hardware & performance config |
| `inference_params.yaml` | TFLite / Mediapipe inference configuration |
| `zmq_params.yaml` | ZMQ communication settings |
| `model/*.tflite` | Ally/enemy MobileNet models (INT8) |
| `pose_landmarker_lite.task` | Mediapipe pose landmark model |

---

## 🔧 Requirements

```
Ubuntu 22.04 (RPi 4B)
Python 3.10+
ROS2 Humble
OpenCV 4.x
tflite_runtime OR TensorFlow (auto-selected)
mediapipe 0.10+
pyzmq
numpy
```

---

## ⚙️ Configurations

All settings are located in the `config/` directory and can be loaded inside Python nodes.

---

### 1️⃣ **`camera_params.yaml`**

```
camera:
  device_id: 0
  resolution_width: 640
  resolution_height: 480
  fps: 15

  flip_horizontal: false
  flip_vertical: false

  # Performance options
  use_thread: true
  buffer_size: 2
```

---

### 2️⃣ **`inference_params.yaml`**

```
inference:
  color_class_model: "model/mobilenetv2_ally_enemy_int8.tflite"
  generic_model: "model/mobilenetv2_int8_v2.tflite"
  pose_model: "model/pose_landmarker_lite.task"

  confidence_threshold: 0.6

  input_width: 224
  input_height: 224

  mode: "color_class"   # color_class / pose / generic
```

---

### 3️⃣ **`zmq_params.yaml`**

```
zmq:
  pub_result_ip: "0.0.0.0"
  pub_result_port: 5100

  sub_mode_ip: "0.0.0.0"
  sub_mode_port: 5101

  pub_error_ip: "0.0.0.0"
  pub_error_port: 5102

  heartbeat_interval: 1.0
```

---

## 🚀 Processing Pipeline Overview

```
   Main Pi                        Camera Pi                        Control Pi
───────────────        ─────────────────────────────────        ──────────────────────
 publishes /mode   →    camera_node FSM controller
                            │
                            ├─ Capture frame
                            ├─ Pose landmark extraction (Mediapipe)
                            ├─ ROI crop (upper body)
                            ├─ Ally/Enemy classification (MobileNet)
                            │
                     publishes detection result
                            │
 TRACK mode: calculate error_angle
                            └─────────────→ send error_angle to Control Pi
```

---

## ▶️ Running the Camera Pi Module

#### 1️⃣ Run full camera pipeline

```
ros2 run camera_pi camera_node
```

#### 2️⃣ Run lightweight color classifier

```
ros2 run camera_pi color_class_node
```

---

## 🧪 Testing

```
colcon test --packages-select camera_pi
```

---

## 📌 Notes & Best Practices

- Camera is completely **disabled in NAV mode** to save CPU.
- STBY mode performs **3-second persistent detection** before confirming ally/enemy.
- TRACK mode transmits **error_angle (yaw/pitch)** only for the targeted class.
- Fully **crash-proof camera pipeline**:  
  auto-reconnect camera, safe ROI handling, landmark detection fallback.
- USB camera auto-detection handles `/dev/videoX` changes.

---

## 📜 License

SAFRS Robotics Platform

License: MIT (pending finalization)

---

## 🙋 Maintainer

**지윤목장**

SAFRS Robotics Team

