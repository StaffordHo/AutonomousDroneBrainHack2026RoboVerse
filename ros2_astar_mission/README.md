# RoboVerse ROS2 A* Baseline

This folder is a clean “back to fundamentals” stack for the RoboVerse Qualifier.

It avoids the previous monolithic heading-sweep behavior and splits the mission into ROS2 nodes:

- `depth_mapper_node`: builds a 40m x 40m occupancy grid from `/depth_camera`.
- `frontier_goal_node`: optional standalone end-to-end 4m-lane coverage goal generator.
- `mission_manager_node`: prioritizes fuel-barrel investigation goals, then falls back to its own lightweight coverage route.
- `astar_planner_node`: plans an A* path through the live occupancy grid.
- `fuel_detector_node`: runs YOLO if available, with HSV fallback for testing.
- `mavsdk_waypoint_follower_node`: follows A* waypoints using the working MAVSDK command path.
- `px4_offboard_node`: pure ROS2 PX4 offboard follower for when `px4_msgs` is installed.
- `dataset_capture_node`: saves images for YOLO labeling and training.
- `roboverse_visualizer_node`: publishes RViz-friendly path and marker topics from the ROS2 mission state.

## Local World File Finding

The local RoboVerse world is at:

```bash
~/PX4-Autopilot/Tools/simulation/gz/worlds/roboverse.sdf
```

That SDF contains the physics/plugins and a single `base6.glb` mesh. It does **not** expose individual obstacle or fuel-barrel coordinates. So A* must build its map online from depth instead of loading static obstacles from the world file.

The challenge envelope from the qualifier spec is:

- 40m width x 40m breadth x 8m high
- about 4m x 4m grid cells
- yellow barrels are ground level
- red barrels are above ground, e.g. on boxes or inside open boxes
- likely target count for your current assumption: 4 red, 5 yellow

## Build

From a ROS2 Humble terminal:

```bash
cd ~/roboverse_qualifier/ros2_astar_mission
colcon build --symlink-install
source install/setup.bash
```

## Start PX4/Gazebo

Start the RoboVerse world as usual with the `x500_vision` vehicle. If the image topic differs, override it in the launch command.

The workshop notes also recommend setting the EKF origin after PX4 is ready.
In the PX4 `pxh>` console:

```text
commander set_ekf_origin 47.397742 8.545594 488.0
```

If `position_velocity_ned()` later jumps to impossible values, restart PX4 and
repeat this before debugging the ROS2 planner.

## ROS2 Visualization

MAVSDK remains the control path. ROS2 is used as middleware for telemetry,
sensor streams, and visualization, matching the workshop notes on transforming
camera/depth data into NED/world-frame map products.

The MAVSDK follower now publishes a compact JSON status topic:

```bash
/roboverse/follower_status
```

Enable the visualization sidecar with:

```bash
ros2 launch roboverse_astar astar_mission.launch.py use_visualizer:=true
```

It publishes:

- `/roboverse/flight_path` (`nav_msgs/Path`) for the actual flown trail.
- `/roboverse/markers` (`visualization_msgs/MarkerArray`) for arena bounds,
  drone heading, commanded velocity, depth rays, temporary danger zones,
  goals/waypoints, A* path, and detected barrel estimates.

Open RViz in another sourced terminal:

```bash
rviz2
```

Set the fixed frame to `px4_ned`, then add displays for `MarkerArray`
`/roboverse/markers`, `Path` `/roboverse/flight_path`, and optionally
`OccupancyGrid` `/roboverse/occupancy` or `Path` `/roboverse/astar_path`.
The maintained `./run_depth_capture.sh` enables the visualizer by default
without enabling the heavier mapper, detector, mission manager, or A* nodes.

## Run With MAVSDK Control, ROS2 Mapping/Planning

`px4_msgs` is not importable on this machine right now, so this is the practical first run:

```bash
cd ~/roboverse_qualifier/ros2_astar_mission
source install/setup.bash
ros2 launch roboverse_astar astar_mission.launch.py \
  use_mavsdk_control:=true \
  use_px4_ros2_control:=false \
  use_frontier:=false \
  system_address:=udpin://0.0.0.0:14540 \
  disable_gcs_failsafe:=true
```

If the desktop/terminal hangs shortly after launch, start in stages. First test
only PX4 connection and Offboard control:

```bash
ROS_LOG_DIR=/tmp/ros_logs ros2 launch roboverse_astar astar_mission.launch.py \
  use_mavsdk_control:=true \
  use_px4_ros2_control:=false \
  use_sensor_bridge:=false \
  use_depth_mapper:=false \
  use_detector:=false \
  use_frontier:=false \
  use_mission_manager:=false \
  use_astar:=false \
  system_address:=udpin://0.0.0.0:14540 \
  disable_gcs_failsafe:=true
```

Stop each launch with `Ctrl-C` and wait for it to exit before starting the next
one. If the drone is already armed or airborne from a previous attempt, restart
PX4/Gazebo for the cleanest test.

Then add mapping and A* without YOLO:

```bash
ROS_LOG_DIR=/tmp/ros_logs ros2 launch roboverse_astar astar_mission.launch.py \
  use_mavsdk_control:=true \
  use_px4_ros2_control:=false \
  use_detector:=false \
  use_image_bridge:=false \
  use_depth_bridge:=true \
  use_frontier:=false \
  depth_process_hz:=2.0 \
  depth_publish_hz:=1.5 \
  depth_num_rays:=32 \
  command_hz:=3.0 \
  local_pose_publish_hz:=3.0 \
  mavsdk_position_rate_hz:=3.0 \
  mavsdk_attitude_rate_hz:=3.0 \
  system_address:=udpin://0.0.0.0:14540 \
  disable_gcs_failsafe:=true
```

If the laptop still kills one of the small Python nodes, remove both A* and the
mission manager for the next smoke test. The MAVSDK follower can generate a
small local coverage route by itself, which isolates PX4/MAVSDK motion from the
ROS2 planning graph:

```bash
ROS_LOG_DIR=/tmp/ros_logs ros2 launch roboverse_astar astar_mission.launch.py \
  use_mavsdk_control:=true \
  use_px4_ros2_control:=false \
  use_sensor_bridge:=false \
  use_depth_mapper:=false \
  use_detector:=false \
  use_frontier:=false \
  use_mission_manager:=false \
  use_astar:=false \
  enable_follower_coverage:=true \
  follower_coverage_half_extent_m:=2.0 \
  follower_coverage_lane_spacing_m:=1.0 \
  follower_coverage_reached_radius_m:=0.7 \
  command_hz:=3.0 \
  local_pose_publish_hz:=3.0 \
  set_mavsdk_stream_rates:=false \
  direct_goal_max_step_m:=0.25 \
  system_address:=udpin://0.0.0.0:14540 \
  disable_gcs_failsafe:=true
```

If LOCAL_POSITION_NED starts jumping to impossible values, use the docs-backed
velocity version of the same test. This does not trust N/E position for
movement; it sends a tiny timed square pattern and uses a small vertical
velocity correction to hold the takeoff altitude:

```bash
ROS_LOG_DIR=/tmp/ros_logs ros2 launch roboverse_astar astar_mission.launch.py \
  use_mavsdk_control:=true \
  use_px4_ros2_control:=false \
  use_sensor_bridge:=false \
  use_depth_mapper:=false \
  use_detector:=false \
  use_frontier:=false \
  use_mission_manager:=false \
  use_astar:=false \
  enable_follower_coverage:=true \
  offboard_control_mode:=velocity \
  follower_velocity_speed_m_s:=0.25 \
  follower_velocity_leg_s:=3.0 \
  follower_velocity_pause_s:=1.0 \
  follower_velocity_yaw_deg:=0.0 \
  command_hz:=5.0 \
  local_pose_publish_hz:=2.0 \
  set_mavsdk_stream_rates:=false \
  system_address:=udpin://0.0.0.0:14540 \
  disable_gcs_failsafe:=true
```

After the velocity-only pattern is healthy, add just the depth bridge and let
the MAVSDK follower steer directly from depth histograms. This is the current
lowest-load autonomy path: no mission manager, no A*, no frontier node, no
detector, and no `cv_bridge`.

```bash
ROS_LOG_DIR=/tmp/ros_logs ros2 launch roboverse_astar astar_mission.launch.py \
  use_mavsdk_control:=true \
  use_px4_ros2_control:=false \
  use_sensor_bridge:=true \
  use_image_bridge:=false \
  use_depth_bridge:=true \
  use_depth_mapper:=false \
  use_detector:=false \
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

To collect fuel-barrel training data while the depth-velocity explorer flies,
turn on only the RGB bridge and the lightweight dataset capture node. This saves
candidate full frames, annotated review images, crops, YOLO label text files,
and `data.yaml` under `datasets/fuel_barrels_v1/`.

```bash
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

The dataset capture labels are deliberately permissive weak labels. Review the
`annotated/` images and correct labels before running a final YOLO training
job, especially for red barrels whose rendered appearance may be orange/white.

The May 30 capture runs showed two important failure modes: yaw accumulation
near the north boundary, and a later side bump where the center depth stayed
open while side/min depth had fallen below 1.1m. The follower now latches
blocked yaw targets, has a soft `0..38m` arena guard, blocks on side/min depth
collapse, slows clear-state flight when side clearance is marginal, remembers
repeated blocked/critical spots as temporary danger zones, and stops Offboard
before commanding simulator kill after multi-meter local-position jumps or hard
arena breaches. Sustained `critical*` depth states and clustered danger zones
also trigger the same simulator kill before waiting for the estimator to fail.
Keep `system_address:=udpin://0.0.0.0:14540` and
`disable_gcs_failsafe:=true` as separate launch arguments.

For fewer launch-argument typos, run the maintained script:

```bash
cd ~/roboverse_qualifier/ros2_astar_mission
./run_depth_capture.sh
```

It also writes terminal output to `/tmp/ros_logs/depth_capture_*.log` and a
path CSV to `/tmp/ros_logs/depth_velocity_path_*.csv`. Plot the latest CSV over
the qualifier map with:

```bash
cd ~/roboverse_qualifier/ros2_astar_mission
./tools/plot_depth_run.py --output /tmp/ros_logs/depth_velocity_path_plot.png
```

Known choke points from a plotted run can be pinned with
`static_danger_zones:="N,E,radius;N,E,radius"`, for example
`static_danger_zones:="14.1,27.4,3.0"` after verifying the coordinates on the
map overlay.

Only after that is stable, enable detection with the nano model first:

```bash
ROS_LOG_DIR=/tmp/ros_logs ros2 launch roboverse_astar astar_mission.launch.py \
  use_mavsdk_control:=true \
  use_px4_ros2_control:=false \
  use_image_bridge:=true \
  use_depth_bridge:=true \
  use_frontier:=false \
  yolo_device:=cpu \
  detector_inference_period_s:=1.5 \
  detector_max_image_width:=320 \
  depth_process_hz:=2.0 \
  depth_publish_hz:=1.5 \
  depth_num_rays:=32 \
  command_hz:=3.0 \
  local_pose_publish_hz:=3.0 \
  mavsdk_position_rate_hz:=3.0 \
  mavsdk_attitude_rate_hz:=3.0 \
  system_address:=udpin://0.0.0.0:14540 \
  disable_gcs_failsafe:=true \
  model_path:=/path/to/yolov8n_roboverse.pt
```

This starts the Gazebo-to-ROS2 image/depth bridge, mapper, detector, goal manager, A* planner, and MAVSDK waypoint follower.

The launch now defaults `use_frontier:=false` because the mission manager has
the same lawnmower coverage route built in. That removes one Python process
from the critical path and avoids the repeated `frontier_goal_node` SIGKILLs
seen on the Legion laptop while swap was full. Re-enable the standalone node
only when debugging coverage-goal output directly.

The image/depth nodes intentionally do not use `cv_bridge`. This machine has a
ROS Humble `cv_bridge` binary compiled against NumPy 1.x, while the active
Python environment uses NumPy 2.x. The package decodes `sensor_msgs/Image`
directly to avoid that crash.

`disable_gcs_failsafe:=true` is useful in your current setup because the start
script tries to launch QGroundControl but the AppImage is missing. PX4 otherwise
can trigger a datalink-loss RTL/land even though the autonomous companion is
still connected.

If MAVSDK cannot connect on your PX4 launch, override:

```bash
ros2 launch roboverse_astar astar_mission.launch.py \
  system_address:=udpin://0.0.0.0:14540
```

## Run Pure ROS2 PX4 Offboard Later

After installing/sourcing `px4_msgs` and verifying `/fmu/in/*` and `/fmu/out/*` topics:

```bash
ros2 launch roboverse_astar astar_mission.launch.py \
  use_mavsdk_control:=false \
  use_px4_ros2_control:=true
```

Verify first:

```bash
ros2 topic list | grep /fmu
python3 -c "import px4_msgs; print('px4_msgs ok')"
```

## Capture YOLO Dataset Frames

Run after the bridge and local pose publisher are active:

```bash
ros2 run roboverse_astar dataset_capture_node \
  --ros-args \
  -p output_dir:=datasets/fuel_barrels_v1/captured \
  -p capture_period_s:=0.8
```

Label images into:

```text
datasets/fuel_barrels_v1/images/train
datasets/fuel_barrels_v1/images/val
datasets/fuel_barrels_v1/labels/train
datasets/fuel_barrels_v1/labels/val
```

YOLO classes:

```text
0 red_fuel_barrel
1 yellow_fuel_barrel
```

Capture from low and high altitudes, from oblique angles, with partial occlusion, bright/dark regions, red barrels on boxes, red barrels in open boxes, and yellow barrels at ground level.

## Train YOLO

```bash
cd ~/roboverse_qualifier/ros2_astar_mission
python3 roboverse_astar/scripts/train_yolo.py \
  --data datasets/fuel_barrels_v1/data.yaml \
  --model yolov8n.pt \
  --epochs 100 \
  --imgsz 640
```

Use the resulting `best.pt` as:

```bash
ros2 launch roboverse_astar astar_mission.launch.py \
  model_path:=training_runs/fuel_barrels_yolo/weights/best.pt
```

## Why This Should Explore More Completely

The old mission often made local heading decisions and got trapped revisiting known corridors. This stack separates global intent from local safety:

1. Coverage goals force end-to-end traversal of the 40m x 40m arena.
2. Depth creates a live occupancy grid.
3. A* routes around known obstacles to the next coverage or detection goal.
4. YOLO detections create immediate high-priority goals, without abandoning the full-map route.

That gives us a more testable system: if exploration fails, we can inspect the map, path, current goal, and waypoint topics independently.
