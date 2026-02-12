# Week 5 Sensor Sync Policy (v0)
 
## 1) Why
- We need reproducible sync behavior and metrics under identical inputs (rosbag2).
- Sync behavior must be defined as a policy (not just code), so results can be interpreted consistently.

## 2) Sync Targets (Topics)
- topic_a: /avp/vehicle_state (type: avp_core_implementation/msg/VehicleState, nominal hz: 2hz)
- topic_b: /avp/parking_slot (type: avp_core_implementation/msg/ParkingSlot, nominal hz: 2hz)

## 3) Time Base & Timestamps
- Timestamp source: message.header.stamp
- Note: header.stamp may represent sensor HW time or driver/system time depending on the stack.
- Current priority: consistency and reproducibility (exact meaning can be refined later).

## 4) Sync Policy
- Default: ApproximateTime
  - slop_ms: 100ms
  - queue_size: 10
- Comparison baseline: ExactTime (run on the same bag)

## 5) Metrics (auto-generated in Week 5)
- sync_rate_hz: 
  - Definition: synced_callback_count / duration_sec
  - Expected upper bound: ~2.0 Hz (limited by /avp/parking_slot at 2Hz)
- cb_latency_ms (p50/p95):
  - Definition: callback processing time measured by steady_clock
- skew_ms (p50/p95): 
  - Definition: max(header.stamp) - min(header.stamp) within each synced set
  - Note: larger slop_ms can increase skew p95
- drop_count: must be explicitly defined
  - Option B (initial): expected_hz*duration - synced_callbacks
    - expected_hz = min(input_hz) = 2.0
  - Option A (later, recommended): upgrade to sync_id continuity

## 6) Repro Plan (rosbag2)
- record: 
  - /avp/parking_slot
  - /avp/vehicle_state
- play: use --clock; decide whether nodes use use_sim_time:=true

## 7) Measurement Rule (Rate/Hz)
- Official Hz (ground truth): use rosbag count over 30s (not ros2 topic hz).
  - Procedure:
    1. rm -rf /tmp/w5_rate30 
    2. timeout --signal=SIGINT 30 ros2 bag record -o /tmp/w5_rate30 /avp/vehicle_state /avp/parking_slot
    3. ros2 bag info /tmp/w5_rate30
    4. Hz = Count / Duration_sec
- Notes:
  - Official Hz is derived from 30s rosbag recording: Hz = Count/30.
  - ros2 topic hz is sanity-check only (startup/transport can bias).
  - Pre-check: ensure no duplicated runs (pgrep -af "ros2 launch" must be empty).