#!/usr/bin/env bash
set -euo pipefail

#simple_planning_simulator subscribes to /initialpose3d, not /initialpose.
#The initial pose must be set again whenever Autoware is restarted.

export ROS_DOMAIN_ID=42

ros2 topic pub /initialpose3d geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 3643.99, y: 73610.75, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06853891945200942]}}" \
  -1

echo "initialpose published. Verify with: ros2 run tf2_ros tf2_echo map base_link"
