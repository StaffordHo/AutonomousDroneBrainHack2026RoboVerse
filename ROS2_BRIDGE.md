# ROS2 Sensor Bridge Mode

The mission still uses MAVSDK for PX4 arm/takeoff/offboard position commands.
That command path is already working in the logs.

ROS2 can be enabled for the RGB/depth sensor path when `ros_gz_bridge` is running.
This lets the mission consume `sensor_msgs/msg/Image` from ROS2 nodes while keeping
the MAVSDK control fallback intact.

## Why Not Full ROS2 PX4 Offboard Yet?

ROS2 Humble and `rclpy` are installed, but `px4_msgs` is not importable in this
Python environment. PX4 ROS2 offboard control uses `px4_msgs` topics such as
`/fmu/in/trajectory_setpoint`, `/fmu/in/offboard_control_mode`, and
`/fmu/in/vehicle_command`.

Until `px4_msgs` is installed and the uXRCE-DDS agent topics are verified, a pure
ROS2 command path would be more fragile than the current MAVSDK command path.

## Start The Gazebo-To-ROS2 Sensor Bridge

Run this in a separate terminal after the `x500_vision` RoboVerse world is up:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /depth_camera@sensor_msgs/msg/Image[gz.msgs.Image
```

Then start the mission with ROS2 sensor mode enabled:

```bash
USE_ROS2_SENSOR_BRIDGE=1 python3 competition_mission.py
```

Optional overrides:

```bash
USE_ROS2_SENSOR_BRIDGE=1 \
ROS2_IMAGE_TOPIC=/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image \
ROS2_DEPTH_TOPIC=/depth_camera \
python3 competition_mission.py
```

If ROS2 sensor mode is disabled or unavailable, the mission falls back to direct
Gazebo transport subscriptions.

## Verify Topics

In a normal terminal outside this sandbox, use:

```bash
ros2 topic list
ros2 topic echo /fmu/out/vehicle_odometry --once
```

If `/fmu/out/*` and `/fmu/in/*` topics exist and `px4_msgs` is installed, the next
step would be a separate ROS2 PX4 offboard controller. Until then, MAVSDK remains
the safer command path.
