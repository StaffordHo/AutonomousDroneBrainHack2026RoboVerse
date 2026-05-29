# Successful ROS2 Depth-Velocity + Barrel Dataset Capture Workflow

This document describes the RoboVerse setup that finally behaved reliably on
the Legion laptop: a low-load ROS2 launch that keeps MAVSDK velocity control in
one process, uses Gazebo depth for obstacle avoidance, and optionally captures
candidate red/yellow fuel-barrel images for YOLO training.

## What This Version Does

- Runs PX4 SITL with `x500_vision` in the `roboverse` world.
- Uses MAVSDK `VelocityNedYaw` control instead of absolute local-position
  setpoints, because the VIO/local position estimate can drift or jump.
- Uses only the depth bridge plus one follower node for exploration.
- Adds the RGB bridge plus `dataset_capture_node` only when collecting training
  data.
- Avoids the heavier ROS2 graph during data collection:
  no A*, no mission manager, no frontier node, no YOLO detector.

This is not the final scoring stack yet. It is the stable foundation for
collecting fuel-barrel images and training a better detector.

## Why This Worked

Earlier full ROS2 runs launched many Python processes at once:

- RGB bridge
- depth bridge
- depth mapper
- detector
- frontier goal node
- mission manager
- A* planner
- MAVSDK follower

On the test machine that caused SIGKILLs, Offboard timeouts, or OS hangs. The
successful version narrows the runtime graph to the smallest useful set:

1. Gazebo depth bridge.
2. MAVSDK waypoint follower in velocity mode.
3. Optional Gazebo RGB bridge.
4. Optional dataset capture node.

The follower performs simple depth-histogram steering:

- split the depth image into left, center, and right regions;
- move forward when center is clear;
- slow down as center distance falls;
- strafe/turn toward the clearer side when blocked;
- reverse gently in critical near-obstacle cases.

## Prerequisites

- ROS2 Humble sourced.
- PX4 SITL and Gazebo installed.
- The RoboVerse world available at:

```bash
/home/stafford99/PX4-Autopilot/Tools/simulation/gz/worlds/roboverse.sdf
```

- This repository checked out at:

```bash
~/roboverse_qualifier
```

## Start PX4/Gazebo

In terminal 1:

```bash
~/start_px4.sh
```

Choose:

```text
1) x500_vision
1) roboverse
2) No
```

When PX4 reaches `pxh>`, it is useful to set the EKF origin:

```text
commander set_ekf_origin 47.397742 8.545594 488.0
```

If local position later jumps to impossible values, restart PX4/Gazebo and set
the origin again before debugging ROS2.

## Build The ROS2 Stack

In terminal 2:

```bash
cd ~/roboverse_qualifier/ros2_astar_mission
colcon build --symlink-install
source install/setup.bash
```

Always use `ROS_LOG_DIR=/tmp/ros_logs` for launch commands. The default ROS log
directory can fail under restricted or full home-directory conditions.

## Smoke Test: Depth-Velocity Exploration Only

This is the proven low-load movement test. It starts only the depth bridge and
MAVSDK follower.

```bash
cd ~/roboverse_qualifier/ros2_astar_mission
source install/setup.bash

ROS_LOG_DIR=/tmp/ros_logs ros2 launch roboverse_astar astar_mission.launch.py \
  use_mavsdk_control:=true \
  use_px4_ros2_control:=false \
  use_sensor_bridge:=true \
  use_image_bridge:=false \
  use_depth_bridge:=true \
  use_depth_mapper:=false \
  use_detector:=false \
  use_dataset_capture:=false \
  use_frontier:=false \
  use_mission_manager:=false \
  use_astar:=false \
  offboard_control_mode:=velocity \
  velocity_source:=depth \
  follower_velocity_speed_m_s:=0.28 \
  depth_process_hz:=3.0 \
  depth_stale_timeout_s:=2.0 \
  depth_safe_distance_m:=2.2 \
  depth_critical_distance_m:=1.05 \
  depth_strafe_speed_m_s:=0.18 \
  depth_turn_hysteresis_m:=0.35 \
  command_hz:=5.0 \
  local_pose_publish_hz:=2.0 \
  set_mavsdk_stream_rates:=false \
  system_address:=udpin://0.0.0.0:14540 \
  disable_gcs_failsafe:=true
```

Healthy logs look like:

```text
MAVSDK offboard started.
Depth velocity clear L/C/R/min=...
Depth velocity blocked L/C/R/min=...
Depth velocity critical L/C/R/min=...
```

Short `Depth velocity source has no recent depth frame; holding.` warnings are
acceptable if the stream resumes. Repeated warnings mean the depth bridge or
Gazebo sensor is not keeping up.

## Data Collection: Explore And Capture Barrel Candidates

Once the smoke test works, add RGB bridge and dataset capture:

```bash
cd ~/roboverse_qualifier/ros2_astar_mission
source install/setup.bash

ROS_LOG_DIR=/tmp/ros_logs ros2 launch roboverse_astar astar_mission.launch.py \
  use_mavsdk_control:=true \
  use_px4_ros2_control:=false \
  use_sensor_bridge:=true \
  use_image_bridge:=true \
  use_depth_bridge:=true \
  use_depth_mapper:=false \
  use_detector:=false \
  use_dataset_capture:=true \
  use_frontier:=false \
  use_mission_manager:=false \
  use_astar:=false \
  offboard_control_mode:=velocity \
  velocity_source:=depth \
  follower_velocity_speed_m_s:=0.28 \
  depth_process_hz:=3.0 \
  depth_stale_timeout_s:=2.0 \
  depth_safe_distance_m:=2.2 \
  depth_critical_distance_m:=1.05 \
  depth_strafe_speed_m_s:=0.18 \
  depth_turn_hysteresis_m:=0.35 \
  dataset_process_hz:=2.0 \
  dataset_candidate_capture_period_s:=1.0 \
  dataset_max_image_width:=640 \
  dataset_save_raw_periodic:=false \
  command_hz:=5.0 \
  local_pose_publish_hz:=2.0 \
  set_mavsdk_stream_rates:=false \
  system_address:=udpin://0.0.0.0:14540 \
  disable_gcs_failsafe:=true
```

Output is written under:

```bash
~/roboverse_qualifier/ros2_astar_mission/datasets/fuel_barrels_v1/
```

Important subfolders:

- `images/train/`: full candidate frames for YOLO.
- `labels/train/`: YOLO-format weak labels.
- `annotated/`: review images with boxes drawn.
- `crops/`: per-candidate crops split by class.
- `data.yaml`: Ultralytics dataset config.

The labels are weak auto-labels from permissive HSV masks. Review `annotated/`
and correct bad labels before treating the dataset as final training data.

## Training YOLO After Review

After reviewing/correcting labels, train with:

```bash
cd ~/roboverse_qualifier/ros2_astar_mission
python3 roboverse_astar/scripts/train_yolo.py \
  --data ~/roboverse_qualifier/ros2_astar_mission/datasets/fuel_barrels_v1/data.yaml \
  --model yolov8n.pt \
  --epochs 100 \
  --imgsz 640 \
  --batch 8
```

For weak-label bootstrapping, start with `yolov8n.pt`. Move to larger models
only after inference speed and memory are acceptable.

## Common Mistakes

- Do not paste `disable_gcs_failsafe:=true0.0.0:14540`. The correct final
  lines are:

```bash
  system_address:=udpin://0.0.0.0:14540 \
  disable_gcs_failsafe:=true
```

- Do not enable `use_detector`, `use_mission_manager`, `use_astar`, or
  `use_frontier` during capture until the low-load data collection path is
  stable.
- Do not launch QGroundControl if it is missing or causing datalink failsafe
  confusion; use `disable_gcs_failsafe:=true` for these autonomous tests.
- Do not rely on `cv_bridge`; this environment has a NumPy 2.x compatibility
  issue with the ROS Humble `cv_bridge` binary.
- Do not commit generated images, crops, labels, logs, or PX4 `.ulg` files.

## Next Development Steps

1. Collect candidate images from several complete exploration runs.
2. Review and correct labels.
3. Train a YOLO nano model.
4. Re-enable `fuel_detector_node` at low inference frequency.
5. Use detections to trigger target confirmation and scoring behavior.
6. Only then reintroduce higher-level goal management or A* if needed.
