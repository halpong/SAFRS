# SAFRS Main Raspberry Pi (D-Pi)
## Central Bridge & Mode Manager (UDP Cluster)

This directory contains the **Main Raspberry Pi (D-Pi)** package of the SAFRS project.  
The D-Pi acts as the **central coordinator and message bridge** in the UDP-based Raspberry Pi cluster.

It is responsible for:
- Broadcasting and synchronizing robot operating modes
- Relaying navigation velocity commands
- Enforcing safety timeouts
- Maintaining a single authoritative system state

---

## 📦 Package Overview

**Package name:** `main_pi`  
**Node name:** `main_bridge_node` (D-Bridge)

**Role in SAFRS Cluster:**
- Central control hub
- Mode authority (`NAV / STBY / TRACK`)
- Command relay between Teleop (B-Pi) and Motor Controller (C-Pi)

---

## 📁 Directory Structure

```
main_raspberry_pi/
├── README.md
└── main_pi/
├── config/
├── launch/
│ └── main_bridge.launch.py
├── package.xml
├── resource/
│ └── main_pi
├── setup.py
├── setup.cfg
├── src/
│ └── main_pi/
│ ├── init.py
│ └── main_bridge_node.py
└── test/
```

---

## 🧠 System Role (D-Pi)

The D-Pi ensures **consistent behavior across all Raspberry Pi nodes**.

### Core Responsibilities
- Receives `cmd_vel_nav` from **B-Pi (Teleop / LiDAR Pi)**
- Converts and forwards it as `cmd_vel` to **C-Pi (Motor Pi)** when in `NAV` mode
- Broadcasts `/mode` to **A / B / C Pi**
- Automatically switches to `STBY` mode if no navigation command is received for a defined timeout

---

## 🔁 Mode Management

| Mode  | Description |
|------|------------|
| NAV  | Normal navigation enabled |
| STBY | Standby (motors disabled, tracking idle) |
| TRACK | Target tracking mode (A/C Pi coordination) |

### Mode Rules
- Any node (A/B/C) may request a mode change
- **D-Pi rebroadcasts the mode** to all nodes
- D-Pi maintains the **final authoritative mode**

---

## 🔌 ROS2 Interfaces

### 📥 Subscribers
| Topic | Type | Description |
|------|------|------------|
| `/cmd_vel_nav` | `geometry_msgs/Twist` | Navigation command from B-Pi |
| `/mode` | `std_msgs/String` | Mode change requests from A/B/C |

### 📤 Publishers
| Topic | Type | Description |
|------|------|------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Forwarded motor command to C-Pi |
| `/mode` | `std_msgs/String` | Global mode broadcast |

---

## ⏱️ Safety Timeout Logic

- If no `/cmd_vel_nav` is received for **15 seconds**
- And current mode is `NAV`
- D-Pi automatically switches mode to `STBY`
- This prevents unintended motion due to lost teleop input

---

## 🚀 How to Run


Build workspace

```
colcon build --packages-select main_pi
source install/setup.bash
```

Run D-Pi bridge node

```
ros2 launch main_pi main_bridge.launch.py
```

---

## 🧩 Relationship with Other Nodes

| Pi   | Role                            |
| ---- | ------------------------------- |
| A-Pi | Camera & target detection       |
| B-Pi | Teleop / LiDAR navigation       |
| C-Pi | Motor & STM32 controller        |
| D-Pi | Central bridge & mode authority |

---

## 📌 Notes

- D-Pi contains no hardware-dependent code

- Can be tested on PC or any Linux machine with ROS2

- Designed to remain unchanged even if A/B/C Pi logic evolve

---

## 📜 License

SAFRS Robotics Platform

License: MIT

---

## 🙋 Maintainer

**지윤목장**

SAFRS Robotics Team