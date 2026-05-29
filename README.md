# Autonomous Drone BrainHack 2026 RoboVerse

This repository contains the working code and experiments for the RoboVerse Qualifier challenge using PX4 SITL, Gazebo, MAVSDK, ROS2, depth sensing, and fuel-barrel detection.

The qualifier mission is GNSS-free: the drone must explore a 40m x 40m x 8m space-port world, avoid obstacles, and detect red and yellow fuel barrels within a 10 minute run. Yellow barrels are on ground level. Red barrels are above ground, such as on boxes or inside open boxes.

## Current Best Starting Points

Use `x500_vision` in the `roboverse` Gazebo world.

### RRT-assisted monolithic mission

This is the strongest single-file competition stack so far. It keeps the original MAVSDK control loop, adds frontier coverage, candidate memory, narrow-passage handling, next-best-view exploration, and local RRT recovery.

```bash
cd ~/roboverse_qualifier/rrt_assisted_mission
RRT_ASSIST_MAX_ITERATIONS=200 \
RRT_ASSIST_MAX_RANGE_M=8.5 \
UNCHARTED_NARROW_BONUS=28 \
OPEN_CRUISE_STEP_M=0.62 \
NBV_FRONTIER_GOALS_PER_PASS=8 \
python3 competition_mission.py
```

### Full-world survey then scoring run

Use this when the map is fixed and you want a first run to export likely barrel waypoints, then a second run to prioritize those locations.

```bash
cd ~/roboverse_qualifier/rrt_assisted_mission
SURVEY_TIME_LIMIT_S=720 \
NBV_FRONTIER_GOALS_PER_PASS=10 \
OPEN_CRUISE_STEP_M=0.62 \
python3 survey_world_mission.py

python3 score_from_survey_waypoints.py
```

### ROS2 A* baseline

This is the cleaner "back to fundamentals" architecture. It separates mapping, detection, goal generation, A* planning, and PX4/MAVSDK control into ROS2 nodes.

The current successful ROS2 path is intentionally lighter than the full graph:
MAVSDK velocity control plus the Gazebo depth bridge for exploration, with an
optional RGB dataset-capture node for collecting fuel-barrel training images.
See [SUCCESSFUL_ROS2_DEPTH_VELOCITY_CAPTURE.md](SUCCESSFUL_ROS2_DEPTH_VELOCITY_CAPTURE.md)
for the exact reproducible workflow.

```bash
cd ~/roboverse_qualifier/ros2_astar_mission
colcon build --symlink-install
source install/setup.bash

ROS_LOG_DIR=/tmp/ros_logs ros2 launch roboverse_astar astar_mission.launch.py \
  use_mavsdk_control:=true \
  use_px4_ros2_control:=false \
  system_address:=udpin://0.0.0.0:14540 \
  disable_gcs_failsafe:=true \
  yolo_device:=cpu \
  model_path:=/home/stafford99/roboverse_qualifier/rrt_assisted_mission/Codes/yolov8s_roboverse.pt
```

If memory is tight, use the nano model:

```bash
model_path:=/home/stafford99/roboverse_qualifier/rrt_assisted_mission/Codes/yolov8n_roboverse.pt
```

## Repository Map

- `competition_mission.py`: original integrated mission script.
- `rrt_assisted_mission/`: current high-performing MAVSDK mission with RRT assist, NBV exploration, survey mode, and scoring-from-waypoints mode.
- `mx_hybrid_mission/`: cleaned integration of the useful faster/narrow-passage ideas studied from the MX code.
- `ros2_astar_mission/`: ROS2 package for occupancy mapping, frontier goals, A* planning, YOLO/HSV detection, and MAVSDK/PX4 control.
- `Codes/`: helper scripts, detectors, planners, notebooks, and model files.
- `hybrid_mission/`: earlier hybrid competition experiment.
- `Stafford's V2 code/`: archived V2 reference implementation.
- `documents/`, `tasks/`, `roboverse_setup_documentation.md`: notes, task tracking, and setup references.

Generated logs, raw camera captures, evidence screenshots, ROS2 build outputs, and survey output CSV/JSON files are ignored by Git so the repository stays usable.

## Models

The project uses YOLO models for red/yellow fuel-barrel detection. Trained `.pt` weights are intentionally ignored by Git because they are large and made the normal GitHub push unreliable. Keep them locally, or publish them later through GitHub Releases, cloud storage, or Git LFS.

The main model paths used during testing were:

- `Codes/yolov8s_roboverse.pt`
- `Codes/yolov8n_roboverse.pt`
- `rrt_assisted_mission/Codes/yolov8s_roboverse.pt`
- `rrt_assisted_mission/Codes/yolov8n_roboverse.pt`

After cloning, copy or symlink the desired weights into one of those paths, or pass the model path explicitly with `model_path:=...` for ROS2 launch commands.

The ROS2 detector also includes HSV fallback logic for fast testing and robustness when YOLO is unavailable.

## PX4/Gazebo Notes

The tested simulator launch is:

```bash
~/start_px4.sh
```

Choose:

```text
1) x500_vision
1) roboverse
```

The `x500_vision` model exposes the OakD-Lite RGB image topic and stereo/depth-related topics. The current stacks use the simulator camera/depth streams available from that model; they do not require switching to `x500_depth`.

## Important Safety Rule

Do not manually control the drone during qualifier runs. The qualifier rules disqualify teams that manually control the quadcopter with keyboard, mouse, controller, joystick, or gamepad.

Teleop scripts in this repository are for debugging outside the judged run only.

## More Detail

See [APPROACHES.md](APPROACHES.md) for a comparison of the different mission approaches and why each one exists.
