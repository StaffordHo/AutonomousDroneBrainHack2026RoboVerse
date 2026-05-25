from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    model_path = LaunchConfiguration("model_path")
    yolo_device = LaunchConfiguration("yolo_device")
    system_address = LaunchConfiguration("system_address")
    disable_gcs_failsafe = LaunchConfiguration("disable_gcs_failsafe")
    use_sensor_bridge = LaunchConfiguration("use_sensor_bridge")
    use_mavsdk_control = LaunchConfiguration("use_mavsdk_control")
    use_px4_ros2_control = LaunchConfiguration("use_px4_ros2_control")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sensor_bridge", default_value="true"),
            DeclareLaunchArgument("use_mavsdk_control", default_value="true"),
            DeclareLaunchArgument("use_px4_ros2_control", default_value="false"),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image",
            ),
            DeclareLaunchArgument("depth_topic", default_value="/depth_camera"),
            DeclareLaunchArgument("model_path", default_value="Codes/yolov8s_roboverse.pt"),
            DeclareLaunchArgument("yolo_device", default_value="cpu"),
            DeclareLaunchArgument("system_address", default_value="udpin://0.0.0.0:14540"),
            DeclareLaunchArgument("disable_gcs_failsafe", default_value="true"),
            ExecuteProcess(
                condition=IfCondition(use_sensor_bridge),
                cmd=[
                    "ros2",
                    "run",
                    "ros_gz_bridge",
                    "parameter_bridge",
                    [image_topic, "@sensor_msgs/msg/Image[gz.msgs.Image"],
                    [depth_topic, "@sensor_msgs/msg/Image[gz.msgs.Image"],
                ],
                output="screen",
            ),
            Node(
                package="roboverse_astar",
                executable="depth_mapper_node",
                name="depth_mapper_node",
                output="screen",
                parameters=[{"depth_topic": depth_topic}],
            ),
            Node(
                package="roboverse_astar",
                executable="fuel_detector_node",
                name="fuel_detector_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "depth_topic": depth_topic,
                        "model_path": model_path,
                        "device": yolo_device,
                    }
                ],
            ),
            Node(
                package="roboverse_astar",
                executable="frontier_goal_node",
                name="frontier_goal_node",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="roboverse_astar",
                executable="mission_manager_node",
                name="mission_manager_node",
                output="screen",
            ),
            Node(
                package="roboverse_astar",
                executable="astar_planner_node",
                name="astar_planner_node",
                output="screen",
            ),
            Node(
                condition=IfCondition(use_mavsdk_control),
                package="roboverse_astar",
                executable="mavsdk_waypoint_follower_node",
                name="mavsdk_waypoint_follower_node",
                output="screen",
                parameters=[
                    {
                        "system_address": system_address,
                        "disable_gcs_failsafe": disable_gcs_failsafe,
                    }
                ],
            ),
            Node(
                condition=IfCondition(use_px4_ros2_control),
                package="roboverse_astar",
                executable="px4_offboard_node",
                name="px4_offboard_node",
                output="screen",
            ),
        ]
    )
