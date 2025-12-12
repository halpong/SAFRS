# SAFRS PC–PLC Gateway Module

The **PC–PLC Gateway** connects industrial PLC signals to the SAFRS Autonomous Robot System.  
This module allows a PLC (LSPLC/XGK/XGB series) to trigger robot behavior through:

- **RS232 → PC (PLC polling)**
- **PC → STM32 (serial trigger)**
- **PC → Raspberry Pi (UDP event trigger)**

This enables seamless integration between industrial automation equipment and the SAFRS AGV platform.

---

## 🚀 Features

- Polls PLC using ENQ-based ASCII frame protocol  
- Reads `%MW000` response and extracts the last WORD  
- Detects PLC trigger bit (e.g., **M0 = ON**)  
- Sends:
  - `"S"` to STM32 via serial  
  - `"START"` to Raspberry Pi via UDP  
- Fully configurable (serial, UDP, delay parameters)

---

## 📁 Project Structure

```
pc_plc_gateway/
├── gateway.py              # Main script running polling + triggers
├── config/
│   ├── plc_config.yaml     # PLC serial + frame settings
│   ├── stm_config.yaml     # STM32 serial settings
│   ├── rpi_udp.yaml        # UDP target for Raspberry Pi
│   └── gateway_config.yaml # General timing parameters
└── README.md
```

---

## ⚙️ System Overview

### Hardware Flow
```
PLC → PC (RS232)
PC → STM32 (Serial)
PC → Raspberry Pi (UDP)
```

### Trigger Sequence  
1. PLC returns `%MW000 = 0001`  
2. PC waits configured delay (default: 3 sec)  
3. PC sends:  
   - `"S"` → STM32  
   - `"START"` → Raspberry Pi  
4. Robot begins movement or predefined action  

---

## 🧩 Configuration Files

All settings are in `config/` folder.

Example:

### `plc_config.yaml`
```yaml
plc:
  port: "COM16"
  baudrate: 9600
  timeout_ms: 50
  enq_frame: "\x05""00RSS0106%MW000""\x04"
```

### `rpi_udp.yaml`
```yaml
rpi_udp:
  ip: "172.30.1.5"
  port: 5006
  message: "START"
```

---

## ▶ Running the Gateway

### ✔ 1. Install dependencies
```
pip install pyserial pyyaml
```

### ✔ 2. Connect hardware
| Device | Connection |
|-------|-----------|
| PLC | USB–RS232 to PC |
| STM32 | USB Serial |
| Raspberry Pi | LAN reachable (UDP) |

### ✔ 3. Run
```
python3 gateway.py
```

---

## 🔧 gateway.py (Summary)

`gateway.py` performs:

- Serial polling → PLC  
- ASCII frame parsing → extract last WORD  
- Trigger detection (`0001`)  
- Serial command to STM32 (`S`)  
- UDP trigger to RPi (`START`)  

📌 **Full code is inside `gateway.py` (not inside README).**

---

## 🧠 PLC Ladder Logic (Reference)

The PLC ladder program:

- Responds to ENQ frame (`00RSS0106%MW000`)
- Places `%MW000` as the data word  
- When internal bit M0 turns ON → `%MW000` becomes `0001`  
- Used by PC to determine robot start event  

*(Image reference from internal ladder file)*

---

## 📌 Notes & Best Practices

- Use shielded RS232 cables for noise immunity  
- Keep polling rate above 10–20 Hz for reliable detection  
- If used inside ROS2, wrap gateway as a ROS node  
- Consider adding logging for industrial deployment  

---

## 📜 License

SAFRS Robotics Platform

License: MIT (pending finalization)

---

## 🙋 Maintainer

**지윤목장**  

SAFRS Robotics Team
