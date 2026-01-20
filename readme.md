# AVP_Core_Implementation
> **Autonomous Valet Parking System on Jetson Orin Nano**

## 🎯 Project Vision
- A comprehensive 36-week AVP implementation roadmap for Jetson Orin Nano. It covers the full autonomous driving stack: Modern C++17, ROS 2, Yocto, and TensorRT/CUDA optimization. By integrating 10-sensor fusion and CARLA HILs, it targets <30ms E2E latency and a 94% success rate, demonstrating production-ready deployment skills.

## 🚀 Key Performance Indicators (Target)
- [cite_start]**End-to-End Latency**: < 30ms
- [cite_start]**Processing Speed**: > 30 FPS (with 10 Sensors)
- [cite_start]**Parking Success Rate**: 94% (CARLA Simulation)
- [cite_start]**Memory Footprint**: < 6.0GB / 8.0GB (Jetson Orin Nano) 

## 🛠 Tech Stack
- [cite_start]**Languages**: Modern C++17 (RAII, Move Semantics)
- [cite_start]**Middleware**: ROS 2 Humble (DDS Tuning) 
- [cite_start]**AI/Acceleration**: TensorRT 8.5 (INT8), CUDA 11.4 
- [cite_start]**Infrastructure**: Yocto Project, Docker

## 📈 Roadmap & Progress

### [Phase 1] Foundation (Week 1 - 8)
- **Week 1: Modern C++ Core**
  - Implementation : Adopt `Object Pool` pattern and `Move Semantics`
  - Archive : remove memory allocation resource and implement zero-copy (100cycles 4.25ms)

- # install essential build toolcmake(GCC 11+, CMake 3.22+)
  - sudo apt update
  - sudo apt install build-essential cmake gdb valgrind -y

- # install benchmark tool
  - sudo apt install libbenchmark-dev -y

- # build
  - mkdir -p build && cd build
  - cmake ..
  - make
  - ./AVP_Core_Implementation

---

## 🔗 Documentation & Links


