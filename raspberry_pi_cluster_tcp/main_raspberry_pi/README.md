# SAFRS Main Raspberry Pi Module
The Main Raspberry Pi is the **central controller (brain)** of the SAFRS AGV system.

It performs:

- **ZMQ Bridge** between Camera Pi, LiDAR Pi, and Motor Pi  
- **Publishing `/cmd_vel`** to Motor Pi  
- **Receiving `/odom`, `/scan`, `/camera/image_raw`**  
- **Running Navigation2 (Nav2)** for autonomous driving  
- **Running AMCL localization** (No SLAM here — mapping is done on LiDAR Pi)  
- **RViz visualization & system monitoring**

This module is part of the **SAFRS AGV Robotics Platform** and follows the unified SAFRS software architecture.

---

## 📁 Project Structure

```
project_hybrid_cluster/
├── scripts/
│ ├── zmq/
│ │ ├── camera_zmq_node.py # Receives camera frames from Camera Pi
│ │ ├── lidar_zmq_node.py # Receives /scan data from LiDAR Pi
│ │ ├── motor_zmq_node.py # Sends /cmd_vel → Motor Pi
│ │ └── odom_zmq_node.py # Receives odometry from Motor Pi
│ ├── utils/
│ │ ├── zmq_utils.py # ZMQ wrapper utilities
│ │ ├── math_utils.py # Angle/quaternion math
│ │ ├── param_utils.py # YAML + parameter loader
│ │ └── image_utils.py # Image decoding helpers
│ └── init.py
│
├── launch/
│ ├── main_pi.launch.py # Main system launcher
│ ├── zmq_bridge.launch.py # Runs all ZMQ nodes
│ ├── navigation.launch.py # Nav2 + AMCL launcher
│ ├── rviz_view.launch.py # Visualization launcher
│ ├── camera_zmq.launch.py # Camera subscriber launcher
│ ├── lidar_zmq.launch.py # LiDAR subscriber launcher
│ ├── motor_zmq.launch.py # Cmd_vel → ZMQ bridge launcher
│ └── odom_zmq.launch.py # Odom subscriber launcher
│
├── config/
│ ├── nav2_params.yaml # Navigation2 configuration
│ ├── zmq_params.yaml # ZeroMQ IP/port configuration
│ └── map.yaml # (Optional) Saved map file
│
├── CMakeLists.txt
├── package.xml
└── README.md
```



---

## 🧩 System Overview

### ✔ 1. ZeroMQ Communication Hub  
Main Pi exchanges high-speed data with sub Pi's as follows:

| Source Pi | Main Pi Receives     | Main Pi Publishes |
|-----------|----------------------|-------------------|
| Camera Pi | `/camera/image_raw`  | —                 |
| LiDAR Pi  | `/scan`              | —                 |
| Motor Pi  | `/odom`, `/tf`       | `/cmd_vel`        |


#### Default ZMQ Ports  
(Editable in `config/zmq_params.yaml`)

| Function       | Port  |
|----------------|-------|
| Camera → Main  | 7000  |
| LiDAR → Main   | 6000  |
| Main → Motor   | 5000  |
| Motor → Main   | 5001  |


---

### ✔ 2. Navigation2 Execution (Nav2)

Main Pi runs the full Nav2 stack:

- Global Planner  
- Local Planner (Regulated Pure Pursuit)  
- Behavior Tree Manager  
- AMCL localization  
- Lifecycle Manager  

Nav2 publishes:

- `/cmd_vel`
- `/tf`
- `/path`
- `/global_costmap/*`
- `/local_costmap/*`

---

### ✔ 3. RViz Visualization

Main Pi visualizes:

- `/scan`  
- `/map`  
- `/odom`  
- `/camera/image_raw`  
- TF tree  
- Robot model (URDF)  

---

## 📂 Directory Details

### 1️⃣ **scripts/zmq/**

#### 🔹 `camera_zmq_node.py`
Receives JPEG/base64 camera frames from Camera Pi → publishes `/camera/image_raw`.

#### 🔹 `lidar_zmq_node.py`
Receives LiDAR distance arrays → publishes `/scan`.

#### 🔹 `motor_zmq_node.py`
Receives `/cmd_vel` → sends JSON commands to Motor Pi.

#### 🔹 `odom_zmq_node.py`
Receives `x y θ` from Motor Pi → publishes `/odom` and TF (`odom → base_link`).

---

### 2️⃣ **config/zmq_params.yaml**

Example:

#### 🔹camera:
  ip: "172.30.1.5"
  port: 7000

#### 🔹lidar:
  ip: "172.30.1.14"
  port: 6000

#### 🔹motor:
  ip: "172.30.1.133"
  pub_port: 5000
  odom_port: 5001

---

3️⃣ **launch/main_pi.launch.py**

Starts:

### 🔹ZMQ bridge (camera, lidar, motor, odom)

### 🔹Navigation2 stack + AMCL

### 🔹RViz2 visualizer

### 🔹TF (laser → base_link)

---

## 🚀 How to Run Main Pi

### 1️⃣ **Start ZMQ System**

```
ros2 launch project_hybrid_cluster zmq_bridge.launch.py
```

### 2️⃣ **Start Navigation2**

```
ros2 launch project_hybrid_cluster navigation.launch.py
```

### 3️⃣ **Start Visualization (RViz)**

```
ros2 launch project_hybrid_cluster rviz_view.launch.py
```

| Tool               | Version      | Purpose               |
|--------------------|--------------|-----------------------|
| ROS2 Humble        | Required     | Nav2 / TF             |
| Python 3.10        | Required     | ZMQ nodes             |
| ZeroMQ (pyzmq)     | Required     | Cluster communication |
| Nav2 Stack         | Humble build | Planning & control    |


---

## 📌 Notes & Best Practices

Main Pi does NOT run SLAM.
Mapping is handled by LiDAR Pi (Cartographer).

- Maintain consistent TF tree:
map → odom → base_link → laser

- Ensure all Pi devices use synchronized time (Chrony recommended).

- Keep Nav2 parameters consistent with robot dimensions.

- ZMQ communication must remain non-blocking for safety.

---

## 📜 License

SAFRS Robotics Platform

License: MIT

---

## 🙋 Maintainer

**지윤목장**

SAFRS Robotics Team