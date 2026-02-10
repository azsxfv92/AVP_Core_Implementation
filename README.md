# AVP_Core_Implementation
> **Autonomous Valet Parking System on Jetson Orin Nano**

## 🎯 Project Vision
- A comprehensive 36-week AVP implementation roadmap for Jetson Orin Nano. It covers the full autonomous driving stack: Modern C++17, ROS 2, Yocto, and TensorRT/CUDA optimization. By integrating 10-sensor fusion and CARLA HILs, it targets <30ms E2E latency and a 94% success rate, demonstrating production-ready deployment skills.

## 🚀 Key Performance Indicators (Target)
- **End-to-End Latency**: < 30ms
- **Processing Speed**: > 30 FPS (with 10 Sensors)
- **Parking Success Rate**: 94% (CARLA Simulation)
- **Memory Footprint**: < 6.0GB / 8.0GB (Jetson Orin Nano) 

## 🛠 Tech Stack
- **Languages**: Modern C++17 (RAII, Move Semantics)
- **Middleware**: ROS 2 Humble (DDS Tuning) 
- **AI/Acceleration**: TensorRT 8.5 (INT8), CUDA 11.4 
- **Infrastructure**: Yocto Project, Docker

## 📈 Roadmap & Progress

### Roadmap : ./docs/Roadmap.md
### Current : Week 4 — Repo Hygiene Sprint + ROS2 Advanced (Zero-copy/DDS)

---

## Quick Start Guide
### 1) Clone
```bash
git clone https://github.com/azsxfv92/AVP_Core_Implementation
cd AVP_Core_Implementation
```
### 1) Install dependencies (ROS2 + common build tools)
```bash
chmod +x ./scripts/install_ROS2.sh
./scripts/install_ROS2.sh
```
### 2) Build & Run (One-command)
```bash
chmod +x ./scripts/run_local.sh
./scripts/run_local.sh
```
### Option) Run with logging (DDS baseline) 
```bash
DDS_BASELINE=1 TOPIC=/avp/vehicle_state ./scripts/run_local.sh
```

## 🚀 Setup & Execution Guide

### 1) Basic build tool: Update and install build tools (GCC 11+, CMake 3.22+)
```bash
sudo apt update
sudo apt install build-essential cmake gdb valgrind libbenchmark-dev -y
```

### 2) ROS humble: ROS2 humble Installation and verify ROS2
```bash
chmod +x install_ROS2.sh       
./scripts/install_ROS2.sh               #Run ROS2 installation script
# verify ROS2
source /opt/ros/humble/setup.bash
printenv | grep ROS             #Check ROS2 version
ros2 run demo_nodes_cpp talker  #Check if ROS2 is working - Send message_terminal_1
ros2 run demo_nodes_py listener #Check if ROS2 is working - Receive message_terminal_2
``` 

### 3) ROS path: Register ROS2 environment variable path
```bash
source ~/.bashrc   #Adopt changed setting
```

### 4) Install rqt_plot
```bash
source /opt/ros/humble/setup.bash
sudo apt update
# install rqt_plot and related plug-in  (it's installed according to the current ROS version)
sudo apt install -y ros-$ROS_DISTRO-rqt-plot ros-$ROS_DISTRO-rqt-common-plugins
```

### 5) Run
```bash
chmod +x ./scripts/run_local.sh
./scripts/run_local.sh
```


## DDS Baseline
This project uses **Fast DDS** via `rmw_fastrtps_cpp`.  
A baseline DDS profile is provided for reproducible runs (future tuning starts later).

- DDS profile: `configs/dds/fastdds.xml`
- Enable via: `source scripts/env_dds.sh` (automatically sourced by `scripts/run_local.sh`)

Verify:
```bash
source /opt/ros/humble/setup.bash
source scripts/env_dds.sh
echo $RMW_IMPLEMENTATION
echo $FASTRTPS_DEFAULT_PROFILES_FILE
```