# SAFRS AGV Mechanical Package
A complete mechanical design package for the SAFRS Autonomous Ground Vehicle (AGV).  
This repository contains the full CAD assembly, individual components, and all subassemblies required to build, modify, or extend the SAFRS AGV platform.

> ⚠ Images are not included yet — placeholders are provided and can be updated after upload.

---

## 📁 Project Structure

```
mechanical/
├── inventor/                     # CAD source files (primary modeling workspace)
│   ├── 01_parts/                 # Individual IPT part files (renamed & standardized)
│   ├── 02_subassemblies/         # IAM subassemblies (pitch/roll/yaw modules, motor mounts)
│   ├── safrs_agv_assembly.iam    # Main assembly file
│   └── Design Data/
│
├── SAFRS_export/                 # Distribution-ready mechanical package (Pack&Go equivalent)
│   ├── 01_parts/
│   ├── 02_subassemblies/
│   ├── Templates/
│   ├── Workspaces/
│   ├── OldVersions/
│   ├── Libraries/
│   ├── safrs_agv_assembly.iam
│   └── safrs_agv_assembly.ipj
│
└── README.md                     # This document
```



---

## 🧩 Overview of the Mechanical System

The SAFRS AGV platform consists of modular mechanical assemblies designed for:
- Autonomous navigation  
- Mixed sensor integration (LiDAR, camera, IMU)  
- High-torque motor driving  
- Payload and turret/gimbal expansion  
- ROS2-based motion control  
- Safe operation in smart-factory environments  

Core mechanical functional areas:
- **Chassis Base Structure**
- **Battery Compartment & Power Routing**
- **LiDAR Mount / Sensor Tower**
- **Motor Driver Mounts (L298N / TB6612 / custom)**
- **Pitch–Roll–Yaw Gimbal System**
- **Mecanum or 4WD wheel compatibility**
- **Electronics mounting plates (STM32, ESP32, Mega2560)**

---

## 📂 Directory Details

### **1️⃣ inventor/**
Primary CAD workspace.

Open the main assembly:
inventor/safrs_agv_assembly.iam

Includes:
- Standardized naming  
- Clean part hierarchy  
- Full subassembly structure  
- All IPT/IAM source files  

---

### **2️⃣ SAFRS_export/**
Distribution-ready CAD package (Pack&Go equivalent).

Open the project file:
SAFRS_export/safrs_agv_assembly.ipj

Then open the assembly:
safrs_agv_assembly.iam

---

## 🔧 Software Requirements

| Tool | Version | Purpose |
|------|---------|---------|
| Autodesk Inventor | 2023+ | CAD editing / assembly |
| Fusion 360 (optional) | — | Viewing, STEP export |
| Cura / PrusaSlicer | — | 3D printing |

---

## 🚀 How to Open the Project

### **Method A — Source Folder (inventor/)**
1. Open Inventor  
2. File → Projects → Open  
3. Select:
mechanical/inventor/safrs_agv_assembly.ipj

4. Then open:
inventor/safrs_agv_assembly.iam


---

### **Method B — Exported Package (SAFRS_export/)**
1. Go to:
mechanical/SAFRS_export/


2. Open:
safrs_agv_assembly.ipj


3. Then open:
safrs_agv_assembly.iam



---

## 🎯 Design Goals

- Clean & modular mechanical architecture  
- ROS2 navigation compatibility  
- Easy parametric modification  
- Ready for STEP/STL export  
- Supports LiDAR, camera, IMU, motor drivers  
- Expandable gimbal/sensor tower system  

---

## 📜 License

MIT License — SAFRS Robotics

---

## 🙋‍♂️ Maintainer

**지윤목장**  

SAFRS Robotics Team