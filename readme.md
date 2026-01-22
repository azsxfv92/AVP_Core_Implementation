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

### [Phase 1] Foundation (Week 1 - 8)
#### **Week 1: Modern C++ Core**
  - **Implementation** : Adopt `Object Pool` pattern and `Move Semantics`
  - **Archive** : remove memory allocation resource and implement zero-copy 
  - **Result**: Performance achieved **4.25ms for 100 cycles**.

#### **week 2: ROS2 Infrastructure**
  - **Implementation**: Set up ROS 2 Humble environment and initialized workspace.
  - **Archive**: Configured DDS for high-speed communication and integrated Week 1 core logic.

---

## 🚀 Setup & Execution Guide
### 1. Prerequisites
```bash
# week 1 .Update and install build tools (GCC 11+, CMake 3.22+)
sudo apt update
sudo apt install build-essential cmake gdb valgrind libbenchmark-dev -y

# week 1. build and execution
mkdir -p build && cd build && cmake .. && make && ./AVP_Core_Implementation

# week 2. ROS2 humble Installation
touch install_ros2.sh
chmod +x install_ros2.sh       
./install_ros2.sh               #Run ROS2 installation script
printenv | grep ROS             #Check ROS2 version
ros2 run demo_nodes_cpp talker  #Check if ROS2 is working - Send message_terminal_1
ros2 run demo_nodes_py listener #Check if ROS2 is working - Receive message_terminal_2

# week 2. Register ROS2 environment variable path
echo "source ~/AVP_Core_Implementation/install/setup.bash" >> ~/.bashrc
source ~/.bashrc   #Adopt changed setting

# week 2. build workspace and execute node(ROS2)
colcon build --symlink-install
source install/setup.bash
ros2 run AVP_Core_Implementation week1_perf_test
ros2 run AVP_Core_Implementation avp_main_node

# week 2. register alias for build and run
echo "alias cba='colcon build --symlink-install --packages-select avp_core_implementation && source install/setup.bash && echo \"Build Done\"'" >> ~/.bashrc
source ~/.bashrc
cba # use registered alias







