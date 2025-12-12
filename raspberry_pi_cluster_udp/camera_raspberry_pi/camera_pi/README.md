# 📷 SAFRS Camera Pi Module (UDP Version)

A lightweight, high-performance camera processing module for the SAFRS Robot System.  
This package runs on the **A-Pi (Camera Raspberry Pi)** and performs:

- USB Camera Capture  
- Pose-based ROI detection (optional)  
- Color Classification (ally / enemy / unknown)  
- UDP Streaming to D-Pi (Main Bridge)

This version **removes ROS2 dependencies** except for internal struct compatibility and focuses on **UDP-only communication** for faster and simpler deployment.

---

## 📁 Package Structure

```
camera_pi/
├── camera_pi/
│ ├── camera_udp_client.py # Main A-Pi executable
│ ├── init.py
│
├── config/
│ ├── camera_params.yaml # Camera settings (resolution, FPS, etc.)
│ ├── inference_params.yaml # Model paths, confidence thresholds
│ └── udp_params.yaml # UDP host/port settings
│
├── model/
│ ├── mobilenet_ally_enemy_int8.tflite
│ ├── mobilenet_generic_int8.tflite
│ └── pose_landmarker_lite.task
│
├── launch/
│ └── camera_udp_launch.py # Launch file (ROS2-friendly)
│
├── package.xml
└── setup.py
```

---

## 🚀 Installation

### ✔ 1. Clone into your ROS2 workspace
```bash
cd ~/ros2_ws/src
git clone https://github.com/KAIROS5-JIYUNFARM/SAFRS.git
```

---

### ✔ 2. Build

```
cd ~/ros2_ws
colcon build --packages-select camera_pi
source install/setup.bash
```

---

### ✔ 3. Run

```
ros2 launch camera_pi camera_udp_launch.py
```

---

## ⚙️ Configuration Files
```
camera_params.yaml
```

Controls camera capture behavior:

- resolution

- fps

- flip options

- threading

```
inference_params.yaml
```

Defines:

- model paths

- input resize

- confidence threshold

- processing mode (color / generic / pose)

```
udp_params.yaml
```

Defines:

- destination host (D-Pi)

- port for JSON streaming

- optional heartbeat/buffering settings

---

## 📡 UDP Output Format (JSON)

A-Pi sends a simple JSON packet:

```
{
  "type": "color",
  "label": "ally",
  "confidence": 0.87,
  "cx": 312,
  "cy": 140,
  "bbox": [120, 40, 400, 300],
  "timestamp": 1733993001.22
}
```

---

| Key          | Description              |
| ------------ | ------------------------ |
| `type`       | detection type ("color") |
| `label`      | ally / enemy / unknown   |
| `confidence` | classifier confidence    |
| `cx`, `cy`   | center of ROI            |
| `bbox`       | [x1, y1, x2, y2]         |
| `timestamp`  | epoch time               |

---

## 🧠 Model Requirements

- MobileNetV2 (int8) – optimized for Raspberry Pi CPU

- Pose Landmarker (optional) – used to extract upper-body region (ROI)

- All models are automatically loaded from:

```
share/camera_pi/model/
```

---

## 🔧 Launch File

`camera_udp_launch.py` launches the main executable:

```
executable="camera_udp_client"
```

---

## 📌 Notes

- This package is designed for A-Pi only.

- SLAM, Nav2, Teleop, Trigger Logic are not included in this node.

- If ROS2 FSM is needed later (NAV/STBY/TRACK), it can be added on top of the UDP layer.

---

## 🧪 Debugging

View Camera Output
```
export DISPLAY=:0
```

Check UDP packets
```
nc -ul 5005
```

---

## 📜 License

SAFRS Robotics Platform

License: MIT

---

## 🙋 Maintainer

**지윤목장**

SAFRS Robotics Team