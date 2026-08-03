# AVP_Core_Implementation
> **Autonomous Valet Parking System on Jetson Orin Nano**

![AEB demo](/results/week16_bridge/day6_aeb/media/day6_overlay.gif)

CARLA(Desktop) → Jetson Orin Nano(TensorRT FP16) → Autoware perception → AEB with fail-safe

*Emergency braking triggered by Autoware tracking output, then resuming after the hazard clears*

## 🎯 Project Vision
- Build an integrated autonomous vehicle software pipeline using CARLA, ROS 2, Jetson, TensorRT/CUDA, and Autoware.
- Connect the existing CARLA multi-camera perception pipeline to Autoware-compatible interfaces.
- Establish a closed-loop simulation architecture in which Autoware generates vehicle commands and CARLA returns ego-vehicle state feedback.
- Measure end-to-end latency across perception, planning, and control.
-  Add heartbeat monitoring, watchdog timeout detection, fault injection, and fail-safe transition handling.
- Validate the integrated system through repeatable driving scenarios, quantitative response-time measurements, and documented test results.

## 🚀 Key Performance Indicators (Target)
- **Autoware Integration** : Complete CARLA → perception → Autoware planning/control → CARLA closed-loop connection
- **Topic Interface Validation**: Verify command and feedback paths with reproducible ROS 2 publisher/subscriber analysis
- **End-to-End Latency**: Measure perception → planning → control latency and identify major bottlenecks
- **Runtime Monitoring**: Detect node timeout and heartbeat loss within a defined threshold
- **Fail-Safe Response**: Trigger an emergency-stop or safe-state transition after injected faults
- **Fault-Injection Evaluation**: Measure response-time distribution using repeated tests, including p50 and p99
- **Scenario Reproducibility**: Reproduce basic route driving, stopping, and obstacle-related scenarios in CARLA
- **Documentation**: Maintain architecture diagrams, experiment logs, result tables, and a technical report

## 🛠 Tech Stack
- **Languages**: Modern C++17, Python
- **Middleware**: ROS 2 Humble, DDS, ROS 2 QoS
- **Autonomous Driving Stack**: Autoware
- **Simulation**: CARLA 0.9.15
- **AI / Acceleration**: TensorRT, CUDA, OpenCV
- **Edge Platform**: Jetson Orin Nano
- **Profiling / Analysis**: Nsight Systems, CSV-based latency analysis
- **Video / Transport**: GStreamer, ROS 2 Image and CompressedImage transport
- **Safety / Validation**: Heartbeat monitoring, watchdog timeout detection, fault injection, fail-safe handling
- **Build / Development**: CMake, colcon, Git, Docker

## 📈 Roadmap & Progress

### Roadmap : Stage 1 — Autoware Integration, Safety Monitoring, and Fault-Injection Validation
### Current : Week 16 Day 6 — AEB (Autonomous Emergency Braking) on Autoware tracking output

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


### 10) Week 12 - Add camera transport encode/decode and GStreamer video save

### Week 12 Goal
Week 12 focuses on extending the CARLA camera inference pipeline with video encode/save and compressed image transport.
Main goals:
- Add GStreamer-based overlay video recording.
- Add raw/compressed camera input selection.
- Compare raw `sensor_msgs/msg/Image` and JPEG compressed `sensor_msgs/msg/CompressedImage`.
- Measure `decode_ms`, `pre_ms`, `infer_ms`, `post_ms`, `encode_ms`, and `total_ms`.
- Analyze the trade-off between network bandwidth reduction and JPEG decode latency.

### Week 12 Execution flow

#### 1. Start CARLA server(On PC)
```bash
cd ~/avp_core_implementation
./scripts/run_CARLA_remote.sh
```

#### 2. Publish raw ROS2 image topic from CARLA callback(On PC)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 ./tools/week10_carla_image_pub.py
```

#### 3. Publish compressed ROS2 image topic from CARLA callback(On PC)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 ./tools/week12_carla_compressed_image_pub.py
```

#### 4. Source environment(On Jetson)
```bash
cd ~/avp_core_implementation
source /opt/ros/humble/setup.bash
source install/setup.bash
```

#### 5. Verify incoming camera topic(On Jetson)
```bash
ros2 topic list | grep camera
ros2 topic info -v /avp/camera/front
ros2 topic info -v /avp/camera/front/compressed
ros2 topic hz /avp/camera/front/compressed --qos-reliability best_effort
ros2 topic bw /avp/camera/front/compressed --qos-reliability best_effort
```

#### 6.  Run TensorRT inference with compressed image input(On Jetson)
ros2 launch avp_core_implementation trt_stream_infer.launch.py \
  input_transport:=compressed \
  compressed_image_topic:=/avp/camera/front/compressed \
  preprocess_backend:=cuda \
  enable_encode:=true \
  output_video_path:=results/week12/output/week12_overlay_compressed.mp4 \
  encode_width:=640 \
  encode_height:=360 \
  encode_fps:=10 \
  csv_path:=results/week12/GStreamer/compressed_on.csv

### Week 12 Result
| Transport       | decode_ms | pre_ms | infer_ms | post_ms | encode_ms | total_ms | Notes                                              |
| Raw Image       | 0         | ~3.07  | ~13.71   | ~2.06   | ~0.55     | ~19.46   | Simple pipeline, but high network payload          |
| CompressedImage | ~4.63     | ~2.63  | ~13.71   | ~0.87   | ~0.76     | ~22.72   | ~13.8x smaller payload, but JPEG decode cost added |

### Week 12 Transport Result
| Transport       | Frame payload | Compression ratio | Drop | Notes                                  |
| Raw Image       | ~691 KB/frame | 1.0x              | 0    | No decode cost, larger network traffic |
| CompressedImage | ~50 KB/frame  | ~13.8x            | 0    | Lower bandwidth, added decode latency  |

Main observation:

```text
CompressedImage transport reduced the camera frame payload by about 13.8x.
However, JPEG decode added about 4.63 ms on the Jetson.
The raw path had lower p50 total latency, but the compressed path provided much lower network bandwidth usage.
```

### 11) Week 13 - Multi-stream CARLA camera input + async video save queue

### Week 13 Goal

Week 13 focuses on extending the single-camera CARLA TensorRT pipeline into a multi-stream pipeline and separating video encode/save from the main inference callback.

Main goals:
- Add front/rear CARLA camera sensors.
- Publish two compressed camera topics:
    - /avp/camera/front/compressed
    - /avp/camera/rear/compressed
- Subscribe to both streams on Jetson.
- Add stream_id and per-stream frame_id to the stage time CSV.
- Separate video encode/save from the inference callback using a save queue and a dedicated save worker thread.
- Save front/rear overlay videos independently.
- Measure save queue behavior using save_wait_ms, encode_ms, queue_depth, and drop_reason.

### Week 13 Execution flow

#### 1. Start CARLA server(On PC)
```bash
cd ~/avp_core_implementation
./scripts/run_CARLA_remote.sh
```

#### 2. Publish raw ROS2 image topic from CARLA callback(On PC)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 ./tools/week13_carla_multi_camera_pub.py
```

#### 3. Source environment(On Jetson)
```bash
cd ~/avp_core_implementation
source /opt/ros/humble/setup.bash
source install/setup.bash
```

#### 4. Verify incoming front/rear compressed topics(On Jetson)
```bash
ros2 topic list | grep camera

ros2 topic hz /avp/camera/front/compressed
ros2 topic hz /avp/camera/rear/compressed

ros2 topic bw /avp/camera/front/compressed
ros2 topic bw /avp/camera/rear/compressed
```

#### 5. Run TensorRT multi-stream inference without video save(On Jetson)
```bash
ros2 launch avp_core_implementation trt_stream_infer.launch.py \
  engine_path:=models/trt/yolov5n_fp16.engine \
  front_topic:=/avp/camera/front/compressed \
  rear_topic:=/avp/camera/rear/compressed \
  overlay_topic:=/avp/infer/overlay \
  preprocess_backend:=gpu \
  enable_encode:=true \
  encode_width:=640 \
  encode_height:=360 \
  encode_fps:=10
```


### 12) Week 15 - Autoware setup and Planning/Control topic analysis

### Week 15 Goal

The goal of Week 15 was to shift the project direction from a CARLA-only topic analysis toward an Autoware-compatible architecture.

In this week, I installed Autoware on the PC environment, launched the planning simulator in headless mode, and analyzed the main Planning → Control → Vehicle command topic flow.

### What was done

- Installed and launched Autoware planning simulator in headless mode.
- Collected Autoware node, topic, and service information.
- Identified the main planning trajectory topic.
- Identified the trajectory follower control command topic.
- Analyzed `vehicle_cmd_gate` input/output topics.
- Identified the final Autoware control command for a future CARLA adapter.
- Analyzed the localization and vehicle status feedback topics required by Planning and Control.
- Identified /simulation/simple_planning_simulator as the current ego vehicle state provider.


### Key topic flow
#### 1) Command path:
```text
/planning/scenario_planning/trajectory
        ↓
/control/trajectory_follower/control_cmd
        ↓
/control/vehicle_cmd_gate
        ↓
/control/command/control_cmd
        ↓
Future CARLA adapter
```

#### 2) Feedback path:
```text
CARLA ego vehicle state
        ↓
Future CARLA adapter
        ↓
/localization/kinematic_state
/localization/acceleration
/vehicle/status/steering_status
/vehicle/status/velocity_status
/vehicle/status/gear_status
/vehicle/status/control_mode
        ↓
Autoware Planning and Control
```

### Key findings
- /planning/scenario_planning/trajectory was identified as the main Planning output consumed by Control.
- /control/command/control_cmd was identified as the final command candidate for the future CARLA adapter.
- /localization/kinematic_state was identified as the highest-priority vehicle feedback topic.
- /localization/acceleration and /vehicle/status/steering_status are also consumed by the trajectory follower and vehicle_cmd_gate.
- The future CARLA adapter must work as both a control-command subscriber and a vehicle-state feedback publisher.




### 13) Week 16 Day 6 - AEB (Autonomous Emergency Braking) on Autoware tracking output

### Week 16 Day 6 Goal

Build a longitudinal AEB function that consumes Autoware `multi_object_tracker` output and brakes the CARLA ego vehicle, without the function node depending on the simulator.

### What was done

- Implemented `aeb_node` (C++): transforms tracked objects from `map` to `base_link` via TF, judges hazard by longitudinal distance/lateral offset/existence probability, and publishes `target_accel` + `aeb/status`.
- Kept `aeb_node` simulator-agnostic (`import carla` free); all CARLA control lives in `tools/week16_carla_scenario.py`.
- CARLA scenario node cruises under Traffic Manager autopilot and only takes manual control to brake when AEB engages, to avoid steering conflicts with the Traffic Manager.
- Added a fail-safe input-timeout: brake if either the tracking topic or the raw per-frame detection topic goes stale, since `multi_object_tracker` can keep publishing extrapolated tracks for a while after the real perception source dies.
- Recorded evidence with `tools/week16_record_aeb_evidence.py` (CSV/log of speed, target_accel, aeb_status, autopilot_active).

### Week 16 Day 6 execution flow

#### 1. Start CARLA server (On PC)
```bash
cd ~/avp_core_implementation
./scripts/run_CARLA_remote.sh
```

#### 2. Launch Autoware planning simulator
```bash
./scripts/run_autoware_remote.sh
./scripts/set_initial_pose.sh

# planning_simulator.launch.xml spawns dummy_perception_publisher, which remaps onto
# /perception/object_recognition/detection/objects and masks a dead perception source.
pkill -9 -f autoware_dummy_perception_publisher_node
```

#### 3. Run CARLA scenario (spawns hero + front obstacle, publishes speed, applies target_accel)
```bash
source install/setup.bash
python3 tools/week16_carla_scenario.py
```

#### 4. Run inference and Autoware bridge
```bash
ros2 run avp_core_implementation trt_infer_node        # On Jetson to perform inference and record video
python3 tools/week16_detection_to_autoware_node.py      # On PC
```

#### 5. Run AEB node
```bash
ros2 run avp_core_implementation aeb_node
```

### Key findings

- Reproduced obstacle-triggered braking multiple times: the ego vehicle stops cleanly behind the tracked obstacle with no chattering.
- Reproduced the perception-timeout fail-safe: killing `trt_infer_node` latches `aeb/status = true` and holds the vehicle stopped.
- Found that Autoware's own `dummy_perception_publisher` (from `planning_simulator.launch.xml`) remaps its output onto the same `/perception/object_recognition/detection/objects` topic used by this pipeline, which can silently mask a dead perception source — worth checking for before trusting a fail-safe test.

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