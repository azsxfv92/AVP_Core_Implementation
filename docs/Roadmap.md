# AVP Roadmap (Week 1 ~ Week 36) — Weekly Summary (Goal / Main Features / Deliverables)
---
## Week 1 — Modern C++ Core
**Goal**
- Establish a low-latency C++ core with predictable memory behavior (no runtime allocations in the hot path)

**Main Features**
- Object Pool for reusable allocations
- Move semantics to reduce copies
- Zero-copy data flow for the critical path

**Deliverables**
- `week1_perf_test` (or equivalent) runnable target
- Benchmark result: **4.25ms / 100 cycles**
- Short note: architecture + what was removed (allocations/copies)

---

## Week 2 — ROS 2 Infrastructure
**Goal**
- Bootstrap a ROS 2 Humble workspace and integrate the Week 1 core into a ROS 2 package

**Main Features**
- ROS 2 Humble environment setup
- Workspace/package initialization
- DDS configuration baseline and message flow integration with Week 1 core

**Deliverables**
- ROS 2 package builds via `colcon build`
- Minimal ROS 2 verification (talker/listener)
- Integrated demo node (Week 1 core callable from ROS 2)

---

## Week 3 — Virtual Sensors & Signal Processing
**Goal**
- Create deterministic virtual sensor inputs and validate filtering logic under noise/outliers

**Main Features**
- Virtual sensor generator (noise + outliers)
- Moving Average filter
- Median filter
- Basic evaluation on how filters handle noisy/outlier data

**Deliverables**
- Virtual sensor generator module + sample runs
- Filter implementations + example outputs (before/after)
- Simple metrics/log output (e.g., noise reduction, outlier suppression evidence)


## Week 4 — Repo Hygiene Sprint + ROS2 Advanced (Zero-copy/DDS)
**Goal**
- Establish a “fresh-machine runnable repo” baseline and enter ROS2 performance tuning

**Main Features**
- One-command run skeleton / README install-build-run flow / start DDS tuning

**Deliverables**
- README (minimum runnable)  
- scripts/run_local.sh (draft)  
- strengthened .gitignore  
- results/run_info.txt template

---

## Week 5 — Sensor Synchronization + Reproducible Test Harness
**Goal**
- Prove sensor sync via metrics + build a reproducible input harness

**Main Features**
- Auto metrics: sync rate / latency / drop  
- rosbag2 record/play for deterministic replay

**Deliverables**
- results/sync_metrics.csv  
- scripts/run_sync_test.sh  
- short analysis note in README

---

## Week 6 — Yocto Risk Control + Environment Freeze (Operability)
**Goal**
- Lock down Yocto build/cache/log rules to prevent schedule collapse

**Main Features**
- Separate DL_DIR / SSTATE_DIR  
- Failure-log retention rules + build checklist

**Deliverables**
- yocto_build_notes.md  
- build failure log storage rule  
- updated run_info.txt template (versions/power/clocks)

---

## Week 7 — Jetson Flash/Boot + Baseline Runtime Fixation
**Goal**
- Make ROS2+CUDA+TensorRT baseline stable and repeatable on Jetson

**Main Features**
- Flash → boot → environment validation → minimal node execution

**Deliverables**
- jetson_setup_checklist.md  
- version capture (commands + outputs)  
- minimum runnable demo

---

## Week 8 — Operability #1: Backpressure Standardization (Queue/Drop/Watermark)
**Goal**
- First step toward “long-running without crashing” by fixing queue policy

**Main Features**
- bounded queues + drop-oldest + watermark thresholds + drop-reason counters

**Deliverables**
- queue_policy.md (policy + rationale)  
- one results set (drop rate / latency p95)

---

## Week 9 — Start CARLA Input Pipeline + Prepare TensorRT Engines
**Goal**
- From Week 9, fix input source to CARLA (1–2 cameras): CARLA → ROS2 topics → Jetson reception  
- Generate TensorRT fp16/int8 engines and freeze measurement conditions

**Main Features**
- CARLA on PC + ROS2 bridge + Jetson subscriber  
- TRT engine (fp16 first; int8 starts)

**Deliverables**
- scripts/run_carla_basic.sh (PC: CARLA+bridge)  
- Jetson topic reception logs (ros2 topic hz/echo)  
- results/run_info.txt (versions/power/clocks/resolution/topics)  
- (if possible) fp16 engine + baseline latency table

---

## Week 10 — CARLA → Jetson TRT Inference E2E Demo + Stage Timing
**Goal**
- Complete a working E2E demo: CARLA camera → Jetson TRT inference  
- Start stage timing (minimal segments)

**Main Features**
- PC CARLA → Jetson preprocess → TRT infer → publish detections/overlay  
- stage timing logging (recv/pre/infer/post)

**Deliverables**
- scripts/run_carla_infer.sh  
- results/e2e_stage_times.csv  
- 1 demo capture (video/screenshot) + README section

---

## Week 11 — CUDA Preprocess Kernel (Performance as Main Story) + CARLA-based A/B
**Goal**
- Keep CARLA input fixed and replace preprocess with CUDA kernels to show clear wins

**Main Features**
- fused preprocess kernel + (optional) pinned memory  
- before/after measurement on identical CARLA input

**Deliverables**
- preprocess.cu + A/B table (OpenCV vs CUDA)  
- results/nsys/*.nsys-rep (if possible)  
- results/metrics.csv (p50/p95, fps, util)

---

## Week 12 — (Optional) Make Encode/Decode a Main Story on CARLA + Reporting Standardization
**Goal**
- Prepare to make encode/decode the “main bottleneck story” even with CARLA input

**Main Features (choose A or B or both)**
- A) Recommended: unify Jetson result recording via GStreamer encode (easy to expose bottlenecks)
- B) Advanced: stream CARLA camera as H264 (RTSP) from PC and decode on Jetson

**Deliverables**
- scripts/profile_nsys.sh + fixed results folder conventions  
- (A) overlay → encode save demo + stage timing including encode

---

## Week 13 — Multi-Stream + Video Save Bottleneck as Main Story
**Goal**
- Reproduce and fix “GPU util high but FPS low” in a real pipeline

**Main Features**
- multi-stream + dedicated save/encode thread + integrated backpressure

**Deliverables**
- multi-stream option + results table (GPU util/FPS/power)  
- 1 bottleneck explanation diagram

---

## Week 14 — One-Command Benchmark v1 (Auto-Saving Results)
**Goal**
- Have a benchmark that auto-saves all results with one command

**Main Features**
- scripts/run_bench.sh + collect_metrics.py + auto run_info recording

**Deliverables**
- results/e2e_metrics.json (draft schema)  
- results/metrics.csv  
- benchmark usage documentation

---

## Week 15 — Tracking (ByteTrack) + Latency/Drop Impact Analysis
**Goal**
- Explain how bottlenecks shift when features are added (with numbers)

**Main Features**
- tracking integration + quantified latency/perf impact

**Deliverables**
- tracking on/off comparison table  
- quality vs latency trade-off note (e.g., MOTA/IDF1 vs p95)

---

## Week 16 — Real-time Tracking + Stability Reinforcement (Queues/Drops)
**Goal**
- Ensure the “real-time first” policy holds even with tracking enabled

**Main Features**
- audit threads/locks/queues + remove sources of latency spikes

**Deliverables**
- bottleneck cause list  
- improvement changes (commit/log) + before/after table (p95 focus)

---

## Week 17 — Localization (NDT+EKF) + Degrade Mode Design
**Goal**
- Keep E2E budget with localization added + define degrade policies

**Main Features**
- NDT/EKF + degrade modes (e.g., disable saving/visualization)

**Deliverables**
- degrade policy table  
- localization metrics + budget comparison table

---

## Week 18 — Fault Injection + Watchdog (Detect/Recover)
**Goal**
- Run the full loop: reproduce → detect → mitigate (with evidence)

**Main Features**
- 4 fault-injection scenarios + healthcheck/heartbeat + standardized recovery actions

**Deliverables**
- fault_injection_scenarios.md  
- results per fault (metrics) + response policy table

---

## Week 19 — Planning (OMPL) + E2E Latency Budget
**Goal**
- Control the system via explicit per-stage latency budgets

**Main Features**
- OMPL + latency budget + alert thresholds

**Deliverables**
- e2e_budget.md  
- mitigation policy for budget violations

---

## Week 20 — Safety Node + System State Machine (IDLE/RUN/DEGRADED/FAILSAFE)
**Goal**
- Make operations explainable via explicit state transitions

**Main Features**
- safety conditions + state transitions + emergency stop

**Deliverables**
- state_machine.md (diagram)  
- demo showing FAILSAFE transition

---

## Week 21 — CARLA Full Scale (10 Sensors) + Network/Bandwidth Bottleneck as Main Story
**Goal**
- Spawn 10 sensors and treat transport/QoS/bandwidth as a first-class system bottleneck

**Main Features**
- image_transport compression + QoS retuning + recovery/degrade under disruptions

**Deliverables**
- results/net_metrics.csv (bw/latency p95/drop)  
- 1-page network bottleneck report (cause → fix → results)  
- scripts/run_carla_10sensors.sh

---

## Week 22 — One-Command Repro Benchmark (Full Pipeline including CARLA)
**Goal**
- One command runs the pipeline and auto-saves metrics (including CARLA + PC↔Jetson)

**Main Features**
- scripts/run_full_pipeline.sh (includes CARLA lifecycle or at least validation steps)
- Auto-generated e2e_metrics.json (p50/p95, fps, drop, bw)

**Deliverables**
- run_full_pipeline.sh (final)  
- e2e_metrics.json + run_info.txt (auto)  
- reproducibility results from 3 repeated runs (variance recorded)

---

## Week 23 — AVP Scenario x50 + Failure Case Categorization (Initial)
**Goal**
- Obtain KPI baseline (success rate / error) + initial failure taxonomy

**Main Features**
- parking scenarios + classify ~10 failure cases

**Deliverables**
- scenario_results.csv  
- initial failure taxonomy table

---

## Week 24 — Documentation & Portfolio Packaging (Project-only)
**Goal**
- Produce a one-page, evidence-backed project summary

**Main Features**
- README restructure + 1 demo video + benchmark result tables + stability section

**Deliverables**
- one-page portfolio (Problem → Approach → Metrics → Evidence links)  
- README top section with key metrics + links  
- sections highlighting: encode/decode, one-command bench, long-run stability

---

## Week 25 — Safety Spec (Rust-based) for Operability
**Goal**
- Turn operations/safety/recovery into explicit specifications (practical engineering signal)

**Main Features**
- heartbeat/timeout/criticality table + degrade/stop policies

**Deliverables**
- safety_spec.md  
- state transition diagram  
- metric definitions

---

## Week 26 — Implement Rust Safety Node + ROS2 Bridge
**Goal**
- Build an ops node that observes system health and publishes status

**Main Features**
- health/metrics publishing + basic alarms + minimal degrade trigger

**Deliverables**
- runnable safety_node + metrics topic  
- logs/results saved + README section

---

## Week 27 — Failure Injection + Recovery Verification (Evidence-driven)
**Goal**
- Prove recovery works under injected faults

**Main Features**
- fault scripts + automated recovery flow + evidence packaging

**Deliverables**
- before/after reports per fault  
- reproducible scripts

---

## Week 28 — systemd Deployment + Watchdog + 2h Soak Test Report
**Goal**
- Prove production-style operability (auto start/restart + long-run stability)

**Main Features**
- systemd service + watchdog kick + soak tests

**Deliverables**
- systemd unit files  
- soak_test_report.md + results (json/csv)

---

## Week 29 — NPU Readiness #1: Backend Abstraction (IBackend) + ORTBackend
**Goal**
- Show “accelerator swappability” in code structure

**Main Features**
- IBackend interface + minimal TRTBackend/ORTBackend

**Deliverables**
- backend_interface.h  
- two runnable paths (TRT vs ORT) + comparison run scripts

---

## Week 30 — NPU Readiness #2: ORT Execution Provider Comparison Report
**Goal**
- Build evidence on “what breaks when moving to NPU” (ops support, perf, fallbacks)

**Main Features**
- ORT EP comparisons (CPU/CUDA/(optional) TRT EP) + supported ops / perf / accuracy tables

**Deliverables**
- ort_ep_comparison.md  
- results/ep_metrics.csv

---

## Week 31 — Failure Taxonomy + Incident Template (Repro/Fix/Verify)
**Goal**
- Systematize failures into a “real-world incident response” format

**Main Features**
- 10-category taxonomy + detection signals + response actions

**Deliverables**
- failure_taxonomy.md  
- incident_template.md  
- 10 filled examples

---

## Week 32 — Demos & Visualization: 5 High-Quality Evidence-Linked Clips
**Goal**
- Not 100 videos—only the 5 that an engineer will actually watch

**Main Features**
- (1) decode→infer→encode (2) before/after bottleneck (3) drop policy (4) fault→recovery (5) soak summary

**Deliverables**
- 5 demo clips  
- an “Evidence Map” linking videos → results → code  
- 1-page explanation

---

## Week 33 — Decision Gate: Autoware PR vs Public Benchmark Repo
**Goal**
- Establish external credibility via contribution or a clean public benchmark repo

**Main Features**
- Plan A: Autoware PR / Plan B: publish a Jetson benchmark repo

**Deliverables**
- decision rule document + executed choice + link

---

## Week 34 — External Evidence Execution (Link-based Proof of Progress)
**Goal**
- Provide verifiable external evidence via links (PR or public repo)

**Main Features**
- respond to review feedback / improve docs / strengthen reproducibility

**Deliverables**
- PR link or public repo link  
- usage guide + example results
