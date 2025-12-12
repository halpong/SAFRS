# SAFRS UDP → ROS2 Drive Bridge  
This package receives **UDP commands from a PC** and publishes a ROS2 topic  
to trigger autonomous driving on the AGV.

It is part of the **SAFRS Robotics Main Pi cluster**.

---

## 📌 Purpose

The **PC–PLC–STM32 system** sends `"START"` to the Raspberry Pi (Main Pi) via UDP.  
This package converts that UDP message into a ROS2 topic:

| Input (UDP) | Output (ROS2 Topic) |
|------------|----------------------|
| "START"    | `/start_drive = "start"` |

This event signal is used by the AGV to begin autonomous motion, mission execution, etc.

---

## 📁 Directory Structure

```
udp_to_drive/
├── udp_to_drive/
│   ├── __init__.py
│   ├── udp_bridge.py         # UDP → ROS2 bridge node
│   └── config.yaml           # Port & IP settings
│
├── launch/
│   └── udp_bridge.launch.py  # Launch file for ROS2 execution
│
├── resource/
│   └── udp_to_drive
│
├── package.xml
├── setup.py
└── README.md
```

---

## ⚙️ Configuration (config.yaml)

````yaml
udp:
  bind_ip: "0.0.0.0"        # Listen on all interfaces
  bind_port: 5006           # Port PC sends UDP 'START'
  buffer_size: 1024         # Maximum UDP packet size

topic:
  start_drive_topic: "/start_drive"
  message_value: "start"    # Published when START is received
````
---

## 🧠 Node Behavior

UDP → ROS2 Event Conversion

### ✔ 1. The node listens on UDP port 5006

### ✔ 2."START" is received:

Publish ROS2 message:

```
topic: /start_drive
msg:   "start"
```

### ✔ 3. Non-blocking UDP mode ensures:

- No delays

- Node stays responsive for ROS2 callbacks

---

## 🧩 Launching the Node

### ✔ 1. Normal ROS2 Launch

```
ros2 launch udp_to_drive udp_bridge.launch.py
```

### ✔ 2. Expected Console Output

```
[INFO] UDP listening on 0.0.0.0:5006
[UDP] Received: START
[ROS2] Published '/start_drive'
```

---

## ⚡ Systemd Auto-Start (Recommended)

```
sudo systemctl daemon-reload
sudo systemctl enable udp_to_drive.service
sudo systemctl start udp_to_drive.service
```

---

🛠 Dependencies

| Dependency          | Purpose                   |
| ------------------- | ------------------------- |
| **ROS2 Humble**     | Core framework            |
| **rclpy**           | ROS2 Python API           |
| **std_msgs/String** | Publishes drive event     |
| **socket**          | UDP communication library |

---

## 📜 License

SAFRS Robotics Platform
MIT License (SAFRS Robotics)

---

## 👤 Maintainer

**지윤목장** 

SAFRS Robotics Team