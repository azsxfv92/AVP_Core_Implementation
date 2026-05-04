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
### Current : Week 11 — Compare cpu to cuda kernel to improve preprocessing time

---

## Quick Start Guide
### 1) Clone
```bash
git clone https://github.com/azsxfv92/AVP_Core_Implementation
cd AVP_Core_Implementation
```
### 2) Install dependencies (ROS2 + common build tools)
```bash
chmod +x ./scripts/install_ROS2.sh
./scripts/install_ROS2.sh
```
### 3) Build & Run (One-command)
```bash
chmod +x ./scripts/run_local.sh
./scripts/run_local.sh
```

### 4) Week 5 - Baseline Measurement
#### Official Hz = Count / Duration_sec from the bag info output. (Details: docs/sync_policy.md)
#### In a new terminal (while ./scripts/run_local.sh is running), run following commands:
```bash
rm -rf /tmp/w5_rate30 && rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps* 2>/dev/null || true
timeout --signal=SIGINT 30 ros2 bag record -o /tmp/w5_rate30 /avp/vehicle_state /avp/parking_slot
ros2 bag info /tmp/w5_rate30
```
#### Sync Harness (Approx / Exact)
```bash
source install/setup.bash
ros2 run avp_core_implementation sync_harness_node --ros-args -p policy:=approx -p queue_size:=10 -p slop_ms:=100 -p report_sec:=2.0
# (optional) exact
ros2 run avp_core_implementation sync_harness_node --ros-args -p policy:=exact -p queue_size:=10 -p report_sec:=2.0
```
#### One-command (Recommended): Run Approx/Exact and save CSV
```bash
chmod +x ./scripts/run_sync_test.sh
# example: slop 1ms
SLOP_MS=1 ./scripts/run_sync_test.sh
```

### Option) Run with logging (DDS baseline) 
```bash
DDS_BASELINE=1 TOPIC=/avp/vehicle_state ./scripts/run_local.sh
```

### 5) Week 6 - Yocto Baseline Reproduction
1. Prepare cache directories
2. Clone `poky` and checkout `kirkstone`
3. Initialize build environment
4. Configure `DL_DIR` and `SSTATE_DIR`
5. Build `core-image-minimal`
6. Extend minimal runtime package set
7. Rebuild and save logs

Detailed notes:
- `docs/yocto_build_notes.md`
- `results/run_info.txt`

### 6) Week 8 - Backpressure and Queue Policy
- worker-thread queue pipeline
- bounded queue + drop-oldest
- warning/critical watermark
- runtime observability metrics

### 7) week 9 - CARLA Camera Input + ROS2 Bridge Validation
#### Camera input spec
- Vehicle: `vehicle.tesla.model3`
- Role name: `hero`
- Sensor: `sensor.camera.rgb`
- Attach position: `x=1.5, z=2.4`
- Resolution: `1280x720`
- Sensor tick: `0.1`
- Raw bytes/frame: `3686400`
- Image encoding on ROS 2: `bgra8`

#### What was verified
- CARLA server startup and port readiness (`2000/2001`)
- Python client connection to CARLA (`carla==0.9.15`)
- Hero vehicle spawn and front RGB camera attach
- Camera callback frame reception in Python
- PNG export and metadata save
- ROS 2 bridge launch success
- ROS 2 image topic creation:
  - `/carla/hero/front/image`
  - `/carla/hero/front/camera_info`
- ROS 2 topic echo / info verification

### Week 9 execution flow

#### 1. Start CARLA server
```bash
cd ~/avp_core_implementation
./scripts/run_CARLA_remote.sh
```
#### 2. Launch ROS2 bridge
```bash
source /opt/ros/humble/setup.bash
source ~/carla_ros2_bridge_ws/install/setup.bash
export CARLA_ROOT=$HOME/sim
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
```
#### 3. Spawn vehicle and front camera
```bash
conda activate carla-python37
export PYTHONPATH=/home/<user>/sim/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg:$PYTHONPATH
cd ~/AVP_Core_Implementation
./scripts/run_camera_check.sh
```
#### 4. Verify ROS2 image topic
```bash 
source /opt/ros/humble/setup.bash
source ~/carla_ros2_bridge_ws/install/setup.bash
ros2 topic list | grep -E 'image|camera'
ros2 topic info -v /carla/hero/front/image
ros2 topic echo --once /carla/hero/front/image
```
  
### 8) week 10 - TensorRT Bring-up + Image inference on Jetson
### Week 10 execution flow

#### 1. Start CARLA server(On PC)
```bash
cd ~/avp_core_implementation
./scripts/run_CARLA_remote.sh
```

#### 2. Publish ROS2 image topic from CARLA callback(On PC)
```bash
source /opt/ros/humble/setup.bash
python3 ./tools/week10_carla_image_pub.py
```

#### 3. Source environment(On Jetson)
```bash
cd ~/avp_core_implementation
source /opt/ros/humble/setup.bash
source install/setup.bash
```

#### 4. Verify incoming image topic(On Jetson)
``` bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep avp
ros2 topic hz /avp/camera/front
```

#### 5. Run TensorRT image stream inference(On Jetson)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch avp_core_implementation trt_stream_infer.launch.py
```


### 9) week 11 - Compare cpu to cuda kernel to improve preprocessing time
### Week 11 Execution flow 

#### 1. Start CARLA server(On PC)
```bash
cd ~/avp_core_implementation
./scripts/run_CARLA_remote.sh
```

#### 2. Publish ROS2 image topic from CARLA callback(On PC)
```bash
source /opt/ros/humble/setup.bash
python3 ./tools/week10_carla_image_pub.py
```

#### 3. Source environment(On Jetson)
```bash
cd ~/avp_core_implementation
source /opt/ros/humble/setup.bash
source install/setup.bash
```

#### 4. Verify incoming image topic(On Jetson)
``` bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep avp
ros2 topic hz /avp/camera/front
```

#### 5. Run TensorRT image stream inference(On Jetson)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch avp_core_implementation trt_stream_infer.launch.py preprocess_backend:=cpu csv_path:=results/week11/stage_metrics/stage_times_cpu.csv
ros2 launch avp_core_implementation trt_stream_infer.launch.py preprocess_backend:=cuda csv_path:=results/week11/stage_metrics/stage_times_cuda.csv
```
### week11 result
| Backend | pre_ms   | h2d_ms   | kernel_ms | d2h_ms   | Notes                |
| CPU     | ~4.2-5.2 | 0        | 0         | 0        | OpenCV baseline      |
| CUDA    | ~3.0-3.4 | ~0.6-0.7 | ~0.9      | ~1.2-1.7 | reusable CUDA buffer |


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