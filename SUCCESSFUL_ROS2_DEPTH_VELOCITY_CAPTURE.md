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
- Optionally runs `roboverse_visualizer_node`, which only subscribes to mission
  state topics and publishes RViz marker/path displays.
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
5. Optional ROS2 visualization sidecar for `/roboverse/markers` and
   `/roboverse/flight_path`.

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
~/PX4-Autopilot/Tools/simulation/gz/worlds/roboverse.sdf
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
  depth_safe_distance_m:=2.3 \
  depth_critical_distance_m:=1.10 \
  depth_side_safe_distance_m:=1.75 \
  depth_clear_side_slow_distance_m:=2.20 \
  depth_side_critical_distance_m:=1.10 \
  depth_min_safe_distance_m:=1.10 \
  depth_clear_min_slow_distance_m:=1.80 \
  depth_min_critical_distance_m:=0.90 \
  depth_strafe_speed_m_s:=0.16 \
  depth_blocked_strafe_speed_m_s:=0.08 \
  depth_turn_hysteresis_m:=0.35 \
  depth_escape_retarget_s:=4.0 \
  depth_critical_trap_hard_stop_enabled:=true \
  depth_critical_trap_hard_stop_s:=8.0 \
  danger_zone_enabled:=true \
  danger_zone_radius_m:=3.0 \
  danger_zone_trigger_count:=3 \
  danger_zone_trigger_window_s:=18.0 \
  danger_zone_hold_s:=240.0 \
  danger_zone_push_speed_m_s:=0.18 \
  danger_zone_cluster_hard_stop_enabled:=true \
  danger_zone_cluster_hard_stop_radius_m:=5.0 \
  danger_zone_cluster_hard_stop_window_s:=45.0 \
  depth_path_log_enabled:=true \
  depth_path_log_period_s:=0.5 \
  enable_arena_bounds:=true \
  arena_max_n_m:=38.0 \
  arena_max_e_m:=38.0 \
  max_local_position_jump_m:=5.0 \
  local_position_jump_hold_s:=8.0 \
  local_position_max_out_of_bounds_m:=1.5 \
  local_position_hard_stop:=true \
  local_position_hard_stop_action:=kill \
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
Depth velocity blocked-side L/C/R/min=...
Depth velocity critical L/C/R/min=...
Depth velocity critical-side L/C/R/min=...
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
  depth_safe_distance_m:=2.3 \
  depth_critical_distance_m:=1.10 \
  depth_side_safe_distance_m:=1.75 \
  depth_clear_side_slow_distance_m:=2.20 \
  depth_side_critical_distance_m:=1.10 \
  depth_min_safe_distance_m:=1.10 \
  depth_clear_min_slow_distance_m:=1.80 \
  depth_min_critical_distance_m:=0.90 \
  depth_strafe_speed_m_s:=0.16 \
  depth_blocked_strafe_speed_m_s:=0.08 \
  depth_turn_hysteresis_m:=0.35 \
  depth_escape_retarget_s:=4.0 \
  depth_critical_trap_hard_stop_enabled:=true \
  depth_critical_trap_hard_stop_s:=8.0 \
  danger_zone_enabled:=true \
  danger_zone_radius_m:=3.0 \
  danger_zone_trigger_count:=3 \
  danger_zone_trigger_window_s:=18.0 \
  danger_zone_hold_s:=240.0 \
  danger_zone_push_speed_m_s:=0.18 \
  danger_zone_cluster_hard_stop_enabled:=true \
  danger_zone_cluster_hard_stop_radius_m:=5.0 \
  danger_zone_cluster_hard_stop_window_s:=45.0 \
  depth_path_log_enabled:=true \
  depth_path_log_period_s:=0.5 \
  enable_arena_bounds:=true \
  arena_max_n_m:=38.0 \
  arena_max_e_m:=38.0 \
  max_local_position_jump_m:=5.0 \
  local_position_jump_hold_s:=8.0 \
  local_position_max_out_of_bounds_m:=1.5 \
  local_position_hard_stop:=true \
  local_position_hard_stop_action:=kill \
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
ros2_astar_mission/datasets/fuel_barrels_v1/
```

Important subfolders:

- `images/train/`: full candidate frames for YOLO.
- `labels/train/`: YOLO-format weak labels.
- `annotated/`: review images with boxes drawn.
- `crops/`: per-candidate crops split by class.
- `data.yaml`: Ultralytics dataset config.

The labels are weak auto-labels from permissive HSV masks. Review `annotated/`
and correct bad labels before treating the dataset as final training data.

Do not merge launch arguments accidentally. This is wrong and will create a
bad boolean value instead of setting the MAVSDK address:

```bash
disable_gcs_failsafe:=true0.0.0:14540
```

Keep these as two separate arguments:

```bash
system_address:=udpin://0.0.0.0:14540 \
disable_gcs_failsafe:=true
```

## May 30 Crash Analysis

The May 30 dataset-capture runs did not show a ROS process crash. They show the
flight becoming unhealthy while the controller was still running.

Two failure modes were visible in the logs:

- The first long run reached the far north arena boundary around `N=38.8`,
  `E=16.4`. Depth collapsed to roughly `0.3-0.7m`, and the old controller kept
  adding a new yaw offset on every blocked/critical command. That made the
  vehicle spin and alternate side escapes while trapped near the wall.
- The later `0.35m/s` run exposed a side-clearance bug: readings such as
  `L/C/R/min=6.2/7.5/1.1/0.9` and `6.3/6.7/1.0/0.8` were still treated as
  `clear` because the center depth was open. The drone then bumped an obstacle,
  after which local NED estimates jumped to impossible coordinates.
- A later side-gated run proved depth was engaged, but the vehicle could keep
  cycling through the same narrow hazard pocket. Repeated `blocked-side` or
  `critical-side` readings at the same local pose need to become a temporary
  danger zone, not just another local strafe/yaw attempt.

Fixes now in the follower:

- Blocked/critical yaw targets are latched and only re-target every few seconds,
  so the yaw command no longer accumulates every control tick.
- A soft arena-boundary guard pushes the velocity back inward near the `0..40m`
  world edges.
- Side/min depth gates now trigger `blocked-side` or `critical-side` even when
  the center lane is open, preventing the side-collision case above.
- Clear-state speed now also scales down when side or minimum clearance is only
  barely above the block threshold.
- Repeated blocked/critical depth events create a temporary local danger zone
  that pushes the vehicle away from the spot before it re-enters the same pocket.
- Blocked-state sideways recovery is capped separately from normal strafe speed
  because the forward depth camera cannot fully see blind lateral motion.
- Every depth-velocity run now writes a CSV path log that can be plotted over
  the qualifier map with `tools/plot_depth_run.py`.
- A local-position hard stop stops Offboard and commands simulator kill if
  MAVSDK reports a multi-meter pose jump or a pose more than `1.5m` outside the
  arena after contact.
- Sustained `critical*` depth states and clustered temporary danger zones now
  trigger the same simulator kill, so wall traps abort before waiting for the
  estimator to go bad.
- The recommended cruise speed is back to `0.28m/s`, with blind blocked strafe
  capped at `0.08m/s` and danger-zone push reduced to `0.18m/s`.

## Live ROS2 Visualization

The follower publishes `/roboverse/follower_status` as JSON while still using
MAVSDK for control. The visualizer converts pose, depth state, command velocity,
temporary danger zones, goals, A* paths, and fuel detections into:

- `/roboverse/flight_path` (`nav_msgs/Path`)
- `/roboverse/markers` (`visualization_msgs/MarkerArray`)

For RViz, set the fixed frame to `px4_ned`, then add those two topics. The
maintained `./run_depth_capture.sh` enables this sidecar by default so obstacle
pockets and hard-stop triggers are visible during dataset runs.

## Training YOLO After Review

After reviewing/correcting labels, train with:

```bash
cd ~/roboverse_qualifier/ros2_astar_mission
python3 roboverse_astar/scripts/train_yolo.py \
  --data datasets/fuel_barrels_v1/data.yaml \
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
