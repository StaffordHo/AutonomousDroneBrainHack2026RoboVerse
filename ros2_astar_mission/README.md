# RoboVerse ROS2 A* Baseline

This folder is a clean “back to fundamentals” stack for the RoboVerse Qualifier.

It avoids the previous monolithic heading-sweep behavior and splits the mission into ROS2 nodes:

- `depth_mapper_node`: builds a 40m x 40m occupancy grid from `/depth_camera`.
- `frontier_goal_node`: generates end-to-end 4m-lane coverage goals for the full map.
- `mission_manager_node`: prioritizes fuel-barrel investigation goals, then falls back to coverage.
- `astar_planner_node`: plans an A* path through the live occupancy grid.
- `fuel_detector_node`: runs YOLO if available, with HSV fallback for testing.
- `mavsdk_waypoint_follower_node`: follows A* waypoints using the working MAVSDK command path.
- `px4_offboard_node`: pure ROS2 PX4 offboard follower for when `px4_msgs` is installed.
- `dataset_capture_node`: saves images for YOLO labeling and training.

## Local World File Finding

The local RoboVerse world is at:

```bash
/home/stafford99/PX4-Autopilot/Tools/simulation/gz/worlds/roboverse.sdf
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

## Run With MAVSDK Control, ROS2 Mapping/Planning

`px4_msgs` is not importable on this machine right now, so this is the practical first run:

```bash
cd ~/roboverse_qualifier/ros2_astar_mission
source install/setup.bash
ros2 launch roboverse_astar astar_mission.launch.py \
  use_mavsdk_control:=true \
  use_px4_ros2_control:=false \
  system_address:=udpin://0.0.0.0:14540 \
  disable_gcs_failsafe:=true \
  model_path:=/home/stafford99/roboverse_qualifier/rrt_assisted_mission/Codes/yolov8s_roboverse.pt
```

This starts the Gazebo-to-ROS2 image/depth bridge, mapper, detector, goal manager, A* planner, and MAVSDK waypoint follower.

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
  -p output_dir:=/home/stafford99/roboverse_qualifier/ros2_astar_mission/datasets/fuel_barrels_v1/captured \
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
cd ~/roboverse_qualifier/ros2_astar_mission/roboverse_astar
python3 scripts/train_yolo.py \
  --data /home/stafford99/roboverse_qualifier/ros2_astar_mission/datasets/fuel_barrels_v1/data.yaml \
  --model yolov8n.pt \
  --epochs 100 \
  --imgsz 640
```

Use the resulting `best.pt` as:

```bash
ros2 launch roboverse_astar astar_mission.launch.py \
  model_path:=/home/stafford99/roboverse_qualifier/ros2_astar_mission/training_runs/fuel_barrels_yolo/weights/best.pt
```

## Why This Should Explore More Completely

The old mission often made local heading decisions and got trapped revisiting known corridors. This stack separates global intent from local safety:

1. Coverage goals force end-to-end traversal of the 40m x 40m arena.
2. Depth creates a live occupancy grid.
3. A* routes around known obstacles to the next coverage or detection goal.
4. YOLO detections create immediate high-priority goals, without abandoning the full-map route.

That gives us a more testable system: if exploration fails, we can inspect the map, path, current goal, and waypoint topics independently.
