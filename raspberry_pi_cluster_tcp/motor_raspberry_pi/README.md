# SAFRS Motor Raspberry Pi Module  
The Motor Raspberry Pi is responsible for:

- **Receiving navigation velocity commands (`/cmd_vel`)**
- **Translating high-level commands into motor driver commands (w/a/s/d/x)**
- **Managing the serial link to the Arduino Mega 2560**
- **Publishing raw encoder-based odometry**
- **Streaming `/odom` to the Main Raspberry Pi via ZMQ**

This module is part of the **SAFRS AGV Robotics Platform** and follows the unified SAFRS distributed architecture.

---

## 📁 Project Structure

```
motor_pi/
├── motor_pi/                   # Python module (ROS2 nodes)
│   ├── __init__.py
│   ├── motor_cmd_node.py       # MainPi → MotorPi: ZMQ cmd_vel translator
│   ├── motor_serial_node.py    # Serial bridge (Arduino <-> ZMQ)
│   └── motor_odom_node.py      # Odometry computation from encoder ticks
│
├── arduino/
│   ├── motor_controller.ino    # Arduino Mega firmware (4WD motor control)
│   ├── tb6612_motor.cpp
│   └── tb6612_motor.h
│
├── config/
│   ├── odom_params.yaml        # kinematics & wheel parameters
│   ├── serial_params.yaml      # Arduino serial config
│   └── zmq_params.yaml         # port & ZMQ settings
│
├── service/
│   ├── motor_pi.service        # systemd auto-start service
│   └── start_motor_pi.sh       # startup script for 3 background nodes
│
├── package.xml
├── setup.py
├── setup.cfg
└── README.md                   # this file
```

---

## 🧩 System Overview

### ✔ 1. Command Handling (Navigation2 → MotorPi)
Main Pi publishes velocity commands:

- `linear.x`
- `angular.z`

MotorPi receives them through ZMQ and converts them to:

| Action | Command |
|--------|---------|
| Forward | `w` |
| Backward | `s` |
| Turn Left | `a` |
| Turn Right | `d` |
| Stop | `x` |

These commands are forwarded to the Arduino via `motor_serial_node.py`.

---

### ✔ 2. Serial Bridge (MotorPi ↔ Arduino)
The Arduino Mega 2560 controls:

- 4 motors (LF, RF, LR, RR)
- Encoders (quadrature)

MotorPi:

- Sends ASCII commands (`w/s/a/d/x`)
- Receives encoder strings:
  ```
  LF:1024 RF:1019 LR:1030 RR:1022
  ```
- Publishes them over ZMQ to the odometry node.

---

### ✔ 3. Odometry Computation
From encoder ticks → wheel rotation → robot displacement:

MotorPi computes:

- `x`
- `y`
- `theta`

Then publishes to the Main Pi via ZMQ (`5001`).

These values feed Navigation2’s `/odom` topic.

---

## 📂 File Details

---

### 1️⃣ **motor_cmd_node.py**
Receives ZMQ `cmd_vel` data from Main Pi → converts → sends to serial bridge.

**Inputs:**
- `tcp://MAIN_PI:5000`

**Outputs:**
- `tcp://localhost:5003` → serial bridge

---

### 2️⃣ **motor_serial_node.py**  
Handles Arduino communication.

**Functions:**
- Scan `/dev/ttyACM*`
- Auto-reconnect on serial failure
- Forward Arduino encoder frames to odom node

**Outputs:**
- PUB → `tcp://*:5002`

---

### 3️⃣ **motor_odom_node.py**  
- Converts encoder ticks into `/odom`.

**Robot parameters (from YAML):**

```
wheel_radius: 0.033
wheel_base: 0.19
ticks_per_revolution: 1340
```

Publishes:

```
x y theta
```

---

## 🚀 How to Run MotorPi

### 1️⃣ Manual Execution (development mode)

```
ros2 run motor_pi motor_serial_node
ros2 run motor_pi motor_cmd_node
ros2 run motor_pi motor_odom_node
```

---

### 2️⃣ Run via Systemd (production mode)

```
sudo systemctl enable motor_pi.service
sudo systemctl start motor_pi.service
```

- Check logs:

```
journalctl -u motor_pi.service -f
```

---

## 🔧 Software Requirements

| Tool | Version | Purpose |
|------|---------|---------|
| ROS2 Humble | Required | MotorPi ROS2 nodes |
| Python 3.10 | Required | Node runtime |
| pyserial | Required | Serial communication |
| ZeroMQ | Required | Inter-Pi messaging |
| Arduino Mega 2560 | Firmware | Motor/encoder interface |

---

## 📌 Notes & Best Practices

- Ensure `/dev/ttyACM0` is correctly mapped.
- If Arduino disconnects, auto-reconnect is enabled.
- Odometry accuracy depends on:
  - wheel radius calibration  
  - encoder resolution  
  - wheelbase measurement  
- ZMQ ports for MotorPi:
  - **5000** → MainPi → cmd_vel  
  - **5003** → MotorPi → Arduino  
  - **5002** → Arduino → odom node  
  - **5001** → MotorPi → MainPi (/odom)

---

## 📜 License

SAFRS Robotics Platform

License: MIT (pending finalization)

---

## 🙋 Maintainer

**지윤목장**  

SAFRS Robotics Team
