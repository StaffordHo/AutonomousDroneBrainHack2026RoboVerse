from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    model_path = LaunchConfiguration("model_path")
    yolo_device = LaunchConfiguration("yolo_device")
    system_address = LaunchConfiguration("system_address")
    disable_gcs_failsafe = LaunchConfiguration("disable_gcs_failsafe")
    use_sensor_bridge = LaunchConfiguration("use_sensor_bridge")
    use_image_bridge = LaunchConfiguration("use_image_bridge")
    use_depth_bridge = LaunchConfiguration("use_depth_bridge")
    use_depth_mapper = LaunchConfiguration("use_depth_mapper")
    use_detector = LaunchConfiguration("use_detector")
    use_dataset_capture = LaunchConfiguration("use_dataset_capture")
    use_frontier = LaunchConfiguration("use_frontier")
    enable_internal_coverage = LaunchConfiguration("enable_internal_coverage")
    use_mission_manager = LaunchConfiguration("use_mission_manager")
    use_astar = LaunchConfiguration("use_astar")
    use_mavsdk_control = LaunchConfiguration("use_mavsdk_control")
    use_px4_ros2_control = LaunchConfiguration("use_px4_ros2_control")
    detector_inference_period_s = LaunchConfiguration("detector_inference_period_s")
    detector_max_image_width = LaunchConfiguration("detector_max_image_width")
    dataset_output_dir = LaunchConfiguration("dataset_output_dir")
    dataset_capture_period_s = LaunchConfiguration("dataset_capture_period_s")
    dataset_candidate_capture_period_s = LaunchConfiguration("dataset_candidate_capture_period_s")
    dataset_process_hz = LaunchConfiguration("dataset_process_hz")
    dataset_max_image_width = LaunchConfiguration("dataset_max_image_width")
    dataset_save_raw_periodic = LaunchConfiguration("dataset_save_raw_periodic")
    dataset_max_saved_frames = LaunchConfiguration("dataset_max_saved_frames")
    depth_process_hz = LaunchConfiguration("depth_process_hz")
    depth_publish_hz = LaunchConfiguration("depth_publish_hz")
    depth_num_rays = LaunchConfiguration("depth_num_rays")
    command_hz = LaunchConfiguration("command_hz")
    local_pose_publish_hz = LaunchConfiguration("local_pose_publish_hz")
    mavsdk_position_rate_hz = LaunchConfiguration("mavsdk_position_rate_hz")
    mavsdk_attitude_rate_hz = LaunchConfiguration("mavsdk_attitude_rate_hz")
    set_mavsdk_stream_rates = LaunchConfiguration("set_mavsdk_stream_rates")
    offboard_control_mode = LaunchConfiguration("offboard_control_mode")
    velocity_source = LaunchConfiguration("velocity_source")
    direct_goal_max_step_m = LaunchConfiguration("direct_goal_max_step_m")
    enable_follower_coverage = LaunchConfiguration("enable_follower_coverage")
    follower_coverage_half_extent_m = LaunchConfiguration("follower_coverage_half_extent_m")
    follower_coverage_lane_spacing_m = LaunchConfiguration("follower_coverage_lane_spacing_m")
    follower_coverage_reached_radius_m = LaunchConfiguration("follower_coverage_reached_radius_m")
    follower_velocity_speed_m_s = LaunchConfiguration("follower_velocity_speed_m_s")
    follower_velocity_leg_s = LaunchConfiguration("follower_velocity_leg_s")
    follower_velocity_pause_s = LaunchConfiguration("follower_velocity_pause_s")
    follower_velocity_yaw_deg = LaunchConfiguration("follower_velocity_yaw_deg")
    depth_stale_timeout_s = LaunchConfiguration("depth_stale_timeout_s")
    depth_safe_distance_m = LaunchConfiguration("depth_safe_distance_m")
    depth_slow_distance_m = LaunchConfiguration("depth_slow_distance_m")
    depth_critical_distance_m = LaunchConfiguration("depth_critical_distance_m")
    depth_strafe_speed_m_s = LaunchConfiguration("depth_strafe_speed_m_s")
    depth_reverse_speed_m_s = LaunchConfiguration("depth_reverse_speed_m_s")
    depth_side_gain = LaunchConfiguration("depth_side_gain")
    depth_yaw_bias_deg = LaunchConfiguration("depth_yaw_bias_deg")
    depth_turn_hysteresis_m = LaunchConfiguration("depth_turn_hysteresis_m")
    velocity_altitude_hold = LaunchConfiguration("velocity_altitude_hold")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sensor_bridge", default_value="true"),
            DeclareLaunchArgument("use_image_bridge", default_value="true"),
            DeclareLaunchArgument("use_depth_bridge", default_value="true"),
            DeclareLaunchArgument("use_depth_mapper", default_value="true"),
            DeclareLaunchArgument("use_detector", default_value="true"),
            DeclareLaunchArgument("use_dataset_capture", default_value="false"),
            DeclareLaunchArgument("use_frontier", default_value="false"),
            DeclareLaunchArgument("enable_internal_coverage", default_value="true"),
            DeclareLaunchArgument("use_mission_manager", default_value="true"),
            DeclareLaunchArgument("use_astar", default_value="true"),
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
            DeclareLaunchArgument("detector_inference_period_s", default_value="1.0"),
            DeclareLaunchArgument("detector_max_image_width", default_value="416"),
            DeclareLaunchArgument("dataset_output_dir", default_value="/home/stafford99/roboverse_qualifier/ros2_astar_mission/datasets/fuel_barrels_v1"),
            DeclareLaunchArgument("dataset_capture_period_s", default_value="2.0"),
            DeclareLaunchArgument("dataset_candidate_capture_period_s", default_value="1.0"),
            DeclareLaunchArgument("dataset_process_hz", default_value="3.0"),
            DeclareLaunchArgument("dataset_max_image_width", default_value="640"),
            DeclareLaunchArgument("dataset_save_raw_periodic", default_value="false"),
            DeclareLaunchArgument("dataset_max_saved_frames", default_value="900"),
            DeclareLaunchArgument("depth_process_hz", default_value="3.0"),
            DeclareLaunchArgument("depth_publish_hz", default_value="2.0"),
            DeclareLaunchArgument("depth_num_rays", default_value="48"),
            DeclareLaunchArgument("command_hz", default_value="3.0"),
            DeclareLaunchArgument("local_pose_publish_hz", default_value="3.0"),
            DeclareLaunchArgument("mavsdk_position_rate_hz", default_value="3.0"),
            DeclareLaunchArgument("mavsdk_attitude_rate_hz", default_value="3.0"),
            DeclareLaunchArgument("set_mavsdk_stream_rates", default_value="false"),
            DeclareLaunchArgument("offboard_control_mode", default_value="position"),
            DeclareLaunchArgument("velocity_source", default_value="pattern"),
            DeclareLaunchArgument("direct_goal_max_step_m", default_value="0.45"),
            DeclareLaunchArgument("enable_follower_coverage", default_value="false"),
            DeclareLaunchArgument("follower_coverage_half_extent_m", default_value="6.0"),
            DeclareLaunchArgument("follower_coverage_lane_spacing_m", default_value="3.0"),
            DeclareLaunchArgument("follower_coverage_reached_radius_m", default_value="0.8"),
            DeclareLaunchArgument("follower_velocity_speed_m_s", default_value="0.35"),
            DeclareLaunchArgument("follower_velocity_leg_s", default_value="4.0"),
            DeclareLaunchArgument("follower_velocity_pause_s", default_value="1.0"),
            DeclareLaunchArgument("follower_velocity_yaw_deg", default_value="0.0"),
            DeclareLaunchArgument("depth_stale_timeout_s", default_value="2.0"),
            DeclareLaunchArgument("depth_safe_distance_m", default_value="2.2"),
            DeclareLaunchArgument("depth_slow_distance_m", default_value="4.0"),
            DeclareLaunchArgument("depth_critical_distance_m", default_value="1.05"),
            DeclareLaunchArgument("depth_strafe_speed_m_s", default_value="0.18"),
            DeclareLaunchArgument("depth_reverse_speed_m_s", default_value="0.10"),
            DeclareLaunchArgument("depth_side_gain", default_value="0.45"),
            DeclareLaunchArgument("depth_yaw_bias_deg", default_value="14.0"),
            DeclareLaunchArgument("depth_turn_hysteresis_m", default_value="0.35"),
            DeclareLaunchArgument("velocity_altitude_hold", default_value="true"),
            ExecuteProcess(
                condition=IfCondition(
                    PythonExpression(
                        ["'", use_sensor_bridge, "' == 'true' and '", use_image_bridge, "' == 'true'"]
                    )
                ),
                cmd=[
                    "ros2",
                    "run",
                    "ros_gz_bridge",
                    "parameter_bridge",
                    [image_topic, "@sensor_msgs/msg/Image[gz.msgs.Image"],
                ],
                output="screen",
            ),
            ExecuteProcess(
                condition=IfCondition(
                    PythonExpression(
                        ["'", use_sensor_bridge, "' == 'true' and '", use_depth_bridge, "' == 'true'"]
                    )
                ),
                cmd=[
                    "ros2",
                    "run",
                    "ros_gz_bridge",
                    "parameter_bridge",
                    [depth_topic, "@sensor_msgs/msg/Image[gz.msgs.Image"],
                ],
                output="screen",
            ),
            Node(
                condition=IfCondition(use_depth_mapper),
                package="roboverse_astar",
                executable="depth_mapper_node",
                name="depth_mapper_node",
                output="screen",
                parameters=[
                    {
                        "depth_topic": depth_topic,
                        "process_hz": depth_process_hz,
                        "publish_hz": depth_publish_hz,
                        "num_rays": depth_num_rays,
                    }
                ],
            ),
            Node(
                condition=IfCondition(use_detector),
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
                        "inference_period_s": detector_inference_period_s,
                        "max_image_width": detector_max_image_width,
                    }
                ],
            ),
            Node(
                condition=IfCondition(use_dataset_capture),
                package="roboverse_astar",
                executable="dataset_capture_node",
                name="dataset_capture_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "output_dir": dataset_output_dir,
                        "capture_period_s": dataset_capture_period_s,
                        "candidate_capture_period_s": dataset_candidate_capture_period_s,
                        "process_hz": dataset_process_hz,
                        "max_image_width": dataset_max_image_width,
                        "save_raw_periodic": dataset_save_raw_periodic,
                        "max_saved_frames": dataset_max_saved_frames,
                    }
                ],
            ),
            Node(
                condition=IfCondition(use_frontier),
                package="roboverse_astar",
                executable="frontier_goal_node",
                name="frontier_goal_node",
                output="screen",
            ),
            Node(
                condition=IfCondition(use_mission_manager),
                package="roboverse_astar",
                executable="mission_manager_node",
                name="mission_manager_node",
                output="screen",
                parameters=[
                    {
                        "enable_internal_coverage": enable_internal_coverage,
                    }
                ],
            ),
            Node(
                condition=IfCondition(use_astar),
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
                        "command_hz": command_hz,
                        "local_pose_publish_hz": local_pose_publish_hz,
                        "mavsdk_position_rate_hz": mavsdk_position_rate_hz,
                        "mavsdk_attitude_rate_hz": mavsdk_attitude_rate_hz,
                        "set_mavsdk_stream_rates": set_mavsdk_stream_rates,
                        "offboard_control_mode": offboard_control_mode,
                        "velocity_source": velocity_source,
                        "direct_goal_max_step_m": direct_goal_max_step_m,
                        "enable_follower_coverage": enable_follower_coverage,
                        "follower_coverage_half_extent_m": follower_coverage_half_extent_m,
                        "follower_coverage_lane_spacing_m": follower_coverage_lane_spacing_m,
                        "follower_coverage_reached_radius_m": follower_coverage_reached_radius_m,
                        "follower_velocity_speed_m_s": follower_velocity_speed_m_s,
                        "follower_velocity_leg_s": follower_velocity_leg_s,
                        "follower_velocity_pause_s": follower_velocity_pause_s,
                        "follower_velocity_yaw_deg": follower_velocity_yaw_deg,
                        "depth_topic": depth_topic,
                        "depth_process_hz": depth_process_hz,
                        "depth_stale_timeout_s": depth_stale_timeout_s,
                        "depth_safe_distance_m": depth_safe_distance_m,
                        "depth_slow_distance_m": depth_slow_distance_m,
                        "depth_critical_distance_m": depth_critical_distance_m,
                        "depth_strafe_speed_m_s": depth_strafe_speed_m_s,
                        "depth_reverse_speed_m_s": depth_reverse_speed_m_s,
                        "depth_side_gain": depth_side_gain,
                        "depth_yaw_bias_deg": depth_yaw_bias_deg,
                        "depth_turn_hysteresis_m": depth_turn_hysteresis_m,
                        "velocity_altitude_hold": velocity_altitude_hold,
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
