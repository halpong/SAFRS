# 🚀 SAFRS AGV Robotics Platform — Full System Overview
**Unified Multi-Raspberry-Pi Cluster for an Autonomous Ground Vehicle**

The SAFRS AGV platform is a **distributed robotics system** using four Raspberry Pi units, communicating through **ZeroMQ PUB/SUB**, and operating a full **ROS2 Navigation2 stack**.

This document is the **master overview** of the entire SAFRS system.

---

## 🧠 System Architecture (4-RPi Cluster)

### 📌 Role Overview

| Raspberry Pi | Role | Responsibilities |
|--------------|------|-----------------|
| **Main Pi** | Central Brain | Nav2, AMCL, TF, ZMQ hub, RViz, `/cmd_vel` |
| **LiDAR Pi** | Mapping & LaserScan | LiDAR driver, Cartographer SLAM, `/scan` |
| **Camera Pi** | Vision System | Camera frames, TFLite inference, classification |
| **Motor Pi** | Hardware Control | Motor driver, encoder odom, `/odom` publishing |

---

### 🌐 Cluster Communication Overview

| Source Pi | Main Pi Receives     | Main Pi Publishes |
|-----------|----------------------|-------------------|
| Camera Pi | `/camera/image_raw`  | —                 |
| LiDAR Pi  | `/scan`              | —                 |
| Motor Pi  | `/odom`, `/tf`       | `/cmd_vel`        |

---

### 🔌 Default ZMQ Port Map

| Function       | Port |
|----------------|------|
| Camera → Main  | 7000 |
| LiDAR → Main   | 6000 |
| Main → Motor   | 5000 |
| Motor → Main   | 5001 |

---

### 📦 Software Requirements

| Tool               | Version      | Purpose               |
|--------------------|--------------|-----------------------|
| ROS2 Humble        | Required     | Nav2 / TF             |
| Python 3.10        | Required     | ZMQ nodes             |
| ZeroMQ (pyzmq)     | Required     | Cluster communication |
| Nav2 Stack         | Humble build | Planning & control    |

---

## 📁 Directory Structure (Top Level)

```bash
SAFRS/
├── Main_Raspberry_Pi/
│   └── main_pi/
│
├── Lidar_Raspberry_Pi/
│   ├── lidar_driver/
│   └── cartographer_mapping/
│
├── Camera_Raspberry_Pi/
│   └── camera_pi/
│
├── Motor_Raspberry_Pi/
│   └── motor_pi/
│
└── README.md
```
---

## 🔥 Module Descriptions

### 1️⃣ **Main Raspberry Pi — Central Navigation Controller**

Runs the core robotics stack:

- Navigation2 (planner, controller, BT)

- AMCL localization

- TF broadcasting

- ZMQ bridge for all sub-Pis

- RViz visualization

Launch:
```
ros2 launch project_hybrid_cluster main_pi.launch.py
```

---

### 2️⃣ **LiDAR Raspberry Pi — Mapping + LaserScan Provider**

Provides:

- `/scan` via Python LiDAR driver

Optional Cartographer SLAM

- Generates `map.pgm` + `map.yaml`

Driver:
```
ros2 run lidar_driver lidar_driver_node
```

SLAM:
```
ros2 launch cartographer_mapping mapping.launch.py
```

---

### 3️⃣ **Camera Raspberry Pi — Real-Time Vision Node**

Tasks:

- USB camera capture

- TFLite-based classification

- Optional pose estimation

- Publish JPEG frames → ZMQ → Main Pi

Run:
```
python3 -m camera_pi.camera_node
```

---

### 4️⃣ **Motor Raspberry Pi — Hardware Layer**

Responsibilities:

- Handle /cmd_vel → motor commands

- Forward commands to Arduino Mega (serial)

- Compute encoder odometry

- Publish /odom and TF

Run as systemd service:
```
sudo systemctl start motor_pi.service
```

---

## 📡 TF Tree Overview

```bash
map
 └── odom
      └── base_link
           └── laser_frame

```
---

## 🧩 Data Pipeline Summary

Perception

- From Camera Pi → `/camera/image_raw`

- From LiDAR Pi → `/scan`

Localization

- AMCL on Main Pi provides `/tf`, `/pose`

Planning

- Nav2 generates `/cmd_vel`

Control

- Motor Pi executes motor actions and provides `/odom`

---

## 🚀 How to Launch the Entire SAFRS System

### **1️⃣ Start Sub-PIs**

Camera Pi
```
python3 -m camera_pi.camera_node
```

LiDAR Pi
```
python3 -m lidar_driver.lidar_driver_node
```

(Optional) SLAM:
```
ros2 launch cartographer_mapping mapping.launch.py
```

Motor Pi
```
sudo systemctl start motor_pi.service
```

---

### **2️⃣ Start Main Pi**
```
ros2 launch project_hybrid_cluster main_pi.launch.py
```

Includes:

- ZMQ Bridge

- Nav2

- RViz

- Static TF

---

## 📌 Best Practices

- Use chrony to synchronize clocks between Pis

- Ensure stable Wi-Fi or Ethernet

- Use static IPs for each Raspberry Pi

- Always confirm ZMQ ports do not conflict

- LiDAR Pi performs SLAM, Main Pi performs AMCL only

---

## 📜 License

SAFRS Robotics Platform

License: MIT (pending finalization)

---

## 👤 Maintainers

**지윤목장**

SAFRS Robotics Team