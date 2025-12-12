# SAFRS LiDAR Raspberry Pi Module
The LiDAR Raspberry Pi is responsible for:

- **Collecting raw laser scan data**
- **Publishing ROS2 LaserScan on `/scan`**
- **Optionally running Cartographer SLAM**
- **Providing map data to the Main Pi during mapping operations**

This module is part of the **SAFRS AGV Robotics Platform** and follows the unified SAFRS software architecture.

---

## 📁 Project Structure

```
LiDAR-Raspberry-Pi/
├── lidar_driver/ # Pure Python LiDAR driver
│ ├── lidar_driver/
│ │ └── lidar_node.py # Serial reader → LaserScan publisher
│ ├── config/
│ │ └── lidar_params.yaml # Driver parameters
│ ├── launch/
│ │ └── lidar.launch.py # Driver launch file
│ ├── resource/
│ ├── package.xml
│ ├── setup.py
│
├── cartographer_mapping/ # Cartographer SLAM package
│ ├── cartographer_mapping/
│ ├── config/
│ │ ├── cartographer.lua
│ │ └── cartographer_params.yaml
│ ├── launch/
│ │ └── mapping.launch.py
│ ├── resource/
│ ├── package.xml
│ ├── setup.py
│
└── README.md
```



---

## 🧩 System Overview

### ✔ 1. LiDAR Data Acquisition  
The Python LiDAR driver reads raw packets from **YDLIDAR X4 / X4 Pro**  
and publishes:

`/scan (sensor_msgs/LaserScan)`
---

### ✔ 2. Provide `/scan` to Main Pi  
Main Pi consumes:

- `/scan`  
- `/odom`  
- `/camera/image_raw`

Main Pi **does NOT run SLAM** → Only **Navigation2 + AMCL**.

---

### ✔ 3. (Optional) Cartographer SLAM  
LiDAR Pi can run Cartographer SLAM to generate:

- `map.pgm`  
- `map.yaml`  
- Real-time submaps & trajectory nodes  

Used for **map creation**.

---

## 📂 Directory Details

---

### 1️⃣ **lidar_driver/**
Pure Python LiDAR driver designed for SAFRS Robotics.

#### 🔹 `lidar_node.py`
- Reads raw serial packets  
- Decodes angle & distance data  
- Builds LaserScan messages  
- Publishes `/scan` at high frequency  

#### 🔹 `config/lidar_params.yaml`

```yaml
port: /dev/ttyUSB0
baudrate: 128000

frame_id: laser_frame

range_min: 0.08
range_max: 12.0

angle_min: -180
angle_max: 180

```

---

### 2️⃣ **cartographer_mapping/**

Includes:

| File                     | Purpose                              |
|--------------------------|--------------------------------------|
| `cartographer.lua`       | Base Cartographer SLAM configuration |
| `cartographer_params.yaml` | ROS parameters for SLAM            |
| `mapping.launch.py`      | Starts Cartographer 2D SLAM          |


Cartographer inputs:
`/scan`
`/tf`
`/tf_static`

Cartographer outputs:
`/map`
`/submap_list`
`/trajectory_node_list`

---

## 🚀 How to Run LiDAR + SLAM

### 1️⃣ **Start LiDAR Driver**
```
ros2 launch lidar_driver lidar.launch.py
```

Expected output:
[INFO] [lidar_x4pro_driver]: YDLiDAR X4 Pro connected @ 128000
Publishing LaserScan on /scan ...

---

### 2️⃣ **Start Cartographer SLAM**

```
ros2 launch cartographer_mapping mapping.launch.py
```

Open RViz:

```
rviz2
```

RViz Settings:
- Fixed Frame: map
- LaserScan: `/scan`

Enable Submaps

---

### 3️⃣ **Save Map**

```
ros2 run nav2_map_server map_saver_cli -f ~/map
```

---

## 🔧 Software Requirements
| Tool         | Version      | Purpose              |
|--------------|--------------|----------------------|
| ROS2 Humble  | Required     | SLAM / LaserScan     |
| Python 3.10  | Required     | LiDAR driver         |
| Cartographer | Humble build | Mapping              |
| pyserial     | Python       | Serial communication |


---

## 📌 Notes & Best Practices

- X4 / X4 Pro use fixed 128000 baudrate

- Frame name must be laser_frame

- SLAM only runs on LiDAR Pi

- Main Pi runs AMCL only, no SLAM

- Ensure TF tree:
map → odom → base_link → laser_frame

---

## 📜 License

SAFRS Robotics Platform

License: MIT (pending finalization)

---

## 🙋 Maintainer

**지윤목장**

SAFRS Robotics Team