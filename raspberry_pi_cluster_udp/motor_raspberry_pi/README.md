# SAFRS Motor Raspberry Pi (C-Pi)
## STM32 Motor & Odometry Control Node (UDP Cluster Version)

The **Motor Raspberry Pi (C-Pi)** is responsible for **low-level motor control and state feedback**
within the SAFRS AGV UDP-based distributed robotics system.

This node directly interfaces with **STM32 motor controller hardware** and bridges
ROS2 velocity commands into **PWM-based motor control**, while publishing **odometry, IMU,
and system status** back to the cluster.

---

## 🧠 Role in SAFRS System

C-Pi acts as the **final actuator control layer**.

### Responsibilities
- Receive `/cmd_vel` from D-Pi (Main Bridge)
- Convert velocity commands to **RPM → PWM**
- Transmit motor commands to STM32 via **UART**
- Receive feedback from STM32
- Publish:
  - `/odom`
  - `/imu/data`
  - `/status`
- Manage **motor enable / disable** based on system mode

---

## 📡 Communication Overview

### Subscribed Topics
- `/cmd_vel` (`geometry_msgs/Twist`)
- `/mode` (`std_msgs/String`)

### Published Topics
- `/odom` (`nav_msgs/Odometry`)
- `/imu/data` (`sensor_msgs/Imu`)
- `/status` (`std_msgs/String`)

---

## 📁 Directory Structure

```
motor_raspberry_pi/
└── motor_pi/
    ├── config/
    │   ├── serial_params.yaml      # STM32 serial configuration
    │   └── odom_params.yaml        # Odometry calculation parameters
    │
    ├── launch/
    │   └── motor_stm_controller.launch.py
    │
    ├── src/
    │   └── motor_pi/
    │       ├── __init__.py
    │       └── stm_controller_node.py
    │
    ├── package.xml
    └── setup.py
```

---

## ⚙️ Configuration Files

### 1️⃣ `serial_params.yaml`
Defines STM32 serial communication settings.

- UART port
- Baudrate
- Reconnect behavior

Used to ensure **robust STM32 connectivity**.

---

### 2️⃣ `odom_params.yaml`
Defines robot kinematics and odometry parameters.

- Wheel radius
- Wheel base
- Encoder CPR
- TF publish options
- Update rate

Odometry is computed on STM32 and published through this node.

---

## 🚀 How to Run

### Launch Motor Control Node

```bash
ros2 launch motor_pi motor_stm_controller.launch.py
```

If STM32 is connected correctly, you should see:

- UART connection logs
- Incoming `/cmd_vel` messages
- Periodic encoder or status output

---

## 🧩 Internal Control Flow

```
/cmd_vel (ROS2)
   ↓
C-Pi (Motor Pi)
   ↓
Velocity → RPM → PWM
   ↓
UART Command (STM32)
   ↓
Motor Driver
```

Feedback loop:

```
STM32
   ↓
UART Feedback
   ↓
C-Pi
   ↓
/odom, /imu/data, /status
```

---

## ⚠️ Notes & Best Practices

- Minimum PWM is enforced (deadzone protection)
- PID is currently disabled (straight-line motion stable)
- Do **not modify motor control logic** when adding gimbal or tracking features
- Ensure serial port permissions are correctly set:
  ```bash
  sudo usermod -a -G dialout $USER
  ```

---

## 🔧 Hardware Assumptions

- Motor: **JGB37-520 DC Motor**
- Gear Ratio: 1:30
- Encoder CPR: 44 × 30 = **1320**
- Controller: **STM32 (UART-based protocol)**

---

## 📜 License

SAFRS Robotics Platform

License: MIT

---

## 🙋 Maintainer

**지윤목장**

SAFRS Robotics Team

