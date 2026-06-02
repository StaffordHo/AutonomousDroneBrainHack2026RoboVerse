#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

colcon build --symlink-install

# ROS setup files reference optional COLCON_* variables directly, so source
# them with nounset disabled and then restore the stricter shell mode.
set +u
source install/setup.bash
set -u

mkdir -p /tmp/ros_logs
run_log="/tmp/ros_logs/depth_capture_$(date +%Y%m%d_%H%M%S).log"
dataset_dir="$(pwd)/datasets/fuel_barrels_v1"

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
  use_visualizer:=true \
  offboard_control_mode:=velocity \
  velocity_source:=depth \
  follower_velocity_speed_m_s:=0.28 \
  depth_process_hz:=3.0 \
  depth_stale_timeout_s:=2.0 \
  depth_safe_distance_m:=2.3 \
  depth_slow_distance_m:=4.0 \
  depth_critical_distance_m:=1.10 \
  depth_side_safe_distance_m:=1.75 \
  depth_clear_side_slow_distance_m:=2.20 \
  depth_side_critical_distance_m:=1.10 \
  depth_min_safe_distance_m:=1.10 \
  depth_clear_min_slow_distance_m:=1.80 \
  depth_min_critical_distance_m:=0.90 \
  depth_strafe_speed_m_s:=0.16 \
  depth_blocked_strafe_speed_m_s:=0.08 \
  depth_reverse_speed_m_s:=0.10 \
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
  arena_min_n_m:=0.6 \
  arena_max_n_m:=38.0 \
  arena_min_e_m:=0.6 \
  arena_max_e_m:=38.0 \
  arena_boundary_margin_m:=2.0 \
  arena_boundary_push_speed_m_s:=0.22 \
  max_local_position_jump_m:=5.0 \
  local_position_jump_hold_s:=8.0 \
  local_position_max_out_of_bounds_m:=1.5 \
  local_position_hard_stop:=true \
  local_position_hard_stop_action:=kill \
  dataset_process_hz:=2.0 \
  dataset_output_dir:="$dataset_dir" \
  dataset_candidate_capture_period_s:=1.0 \
  dataset_max_image_width:=640 \
  dataset_save_raw_periodic:=false \
  command_hz:=5.0 \
  local_pose_publish_hz:=2.0 \
  follower_status_publish_hz:=2.0 \
  visualization_publish_hz:=3.0 \
  set_mavsdk_stream_rates:=false \
  system_address:=udpin://0.0.0.0:14540 \
  disable_gcs_failsafe:=true 2>&1 | tee "$run_log"
