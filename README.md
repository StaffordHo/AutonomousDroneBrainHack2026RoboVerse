# RoboVerse ROS2 MAVSDK Drone Stack

This branch contains only the runnable ROS2/MAVSDK stack for the RoboVerse
qualifier workflow.

It keeps MAVSDK Python as the drone control path and uses ROS2 Humble as
middleware for camera/depth topics, pose telemetry, dataset capture, path
logging, and RViz visualization.

Large local artifacts are intentionally not included:

- trained `.pt` model weights
- generated datasets and labels
- ROS2 `build/`, `install/`, and `log/` folders
- simulator logs, screenshots, evidence images, notebooks, and workshop PDFs

## What To Run First

Use this first for the lowest-load repeatable run:

```bash
cd ros2_astar_mission
./run_depth_capture.sh
```

That script builds the ROS2 package, starts the Gazebo-to-ROS2 image/depth
bridges, runs MAVSDK depth-velocity exploration, captures weak fuel-barrel
training images, writes path logs, and enables RViz marker/path visualization.

## Requirements

Tested environment:

- Ubuntu 22.04
- ROS2 Humble
- PX4 SITL plus Gazebo Harmonic/ros_gz_bridge as provided by the RoboVerse setup
- Python 3.10
- MAVSDK Python

Install the Python dependencies:

```bash
python3 -m pip install --user -r requirements.txt
```

For YOLO training or YOLO detector inference, also install:

```bash
python3 -m pip install --user -r requirements-yolo.txt
```

Optional Ubuntu helper:

```bash
./scripts/install_ubuntu22_ros2_humble_deps.sh
```

## Build

```bash
cd ros2_astar_mission
colcon build --symlink-install
source install/setup.bash
```

Always use `/tmp/ros_logs` for ROS launch logs on machines where `~/.ros/log`
may be restricted or full:

```bash
mkdir -p /tmp/ros_logs
```

## Start PX4/Gazebo

Start the RoboVerse simulator the usual way:

```bash
~/start_px4.sh
```

Choose:

```text
1) x500_vision
1) roboverse
2) No
```

When PX4 reaches `pxh>`, set the EKF origin:

```text
commander set_ekf_origin 47.397742 8.545594 488.0
```

If local NED position jumps to impossible values after a collision, restart
PX4/Gazebo and set the EKF origin again before debugging code.

## Run Dataset Capture And Depth-Velocity Exploration

In a second terminal:

```bash
cd ros2_astar_mission
./run_depth_capture.sh
```

Useful outputs:

- terminal log: `/tmp/ros_logs/depth_capture_*.log`
- path CSV: `/tmp/ros_logs/depth_velocity_path_*.csv`
- weak dataset images/labels: `ros2_astar_mission/datasets/fuel_barrels_v1/`
- RViz markers: `/roboverse/markers`
- flight path: `/roboverse/flight_path`
- follower status JSON: `/roboverse/follower_status`

## RViz Visualization

In another sourced terminal:

```bash
cd ros2_astar_mission
source install/setup.bash
rviz2
```

Set the fixed frame to:

```text
px4_ned
```

Add displays:

- `MarkerArray`: `/roboverse/markers`
- `Path`: `/roboverse/flight_path`
- optional `OccupancyGrid`: `/roboverse/occupancy`
- optional `Path`: `/roboverse/astar_path`

The markers show arena bounds, drone heading, commanded velocity, depth rays,
temporary danger zones, hard-stop status, goals, waypoints, A* path, and fuel
detection estimates.

## Plot A Run Afterward

```bash
cd ros2_astar_mission
./tools/plot_depth_run.py --output /tmp/ros_logs/depth_velocity_path_plot.png
```

The plotter uses the latest `/tmp/ros_logs/depth_velocity_path_*.csv` by
default. If you have a top-down map image or PDF, pass `--map-image` or
`--map-pdf`; otherwise it plots on a blank 40m arena.

## Optional Full ROS2 Graph

Once the low-load run is stable, you can enable mapper, detector, mission
manager, and A*:

```bash
cd ros2_astar_mission
source install/setup.bash
ROS_LOG_DIR=/tmp/ros_logs ros2 launch roboverse_astar astar_mission.launch.py \
  use_mavsdk_control:=true \
  use_px4_ros2_control:=false \
  use_visualizer:=true \
  system_address:=udpin://0.0.0.0:14540 \
  disable_gcs_failsafe:=true
```

The detector falls back to HSV if YOLO or model weights are unavailable. For
YOLO, download or train weights separately and pass:

```bash
model_path:=/path/to/best.pt
```

## Train YOLO From Captured Images

Review and correct the weak labels first, then run:

```bash
cd ros2_astar_mission
python3 roboverse_astar/scripts/train_yolo.py \
  --data datasets/fuel_barrels_v1/data.yaml \
  --model yolov8n.pt \
  --epochs 100 \
  --imgsz 640 \
  --batch 8
```

The resulting weights are written under `training_runs/`, which is ignored by
Git. Publish weights with GitHub Releases, cloud storage, or Git LFS rather
than normal Git history.

## Troubleshooting

- MAVSDK connection: keep `system_address:=udpin://0.0.0.0:14540`.
- Missing Gazebo image/depth frames: verify topics with `gz topic -l`.
- ROS log write errors: launch with `ROS_LOG_DIR=/tmp/ros_logs`.
- Existing follower lock: stop the previous launch before starting another run.
- Map exit after collision: restart PX4/Gazebo; local position is likely corrupt.

## Main Scripts

- `ros2_astar_mission/run_depth_capture.sh`: recommended first run.
- `ros2_astar_mission/start_sensor_bridge.sh`: manual sensor bridge helper.
- `ros2_astar_mission/tools/plot_depth_run.py`: post-run path plotter.
- `ros2_astar_mission/roboverse_astar/scripts/train_yolo.py`: optional YOLO training.
