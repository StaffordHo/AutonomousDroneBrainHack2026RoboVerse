#!/usr/bin/env bash
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "Tip: source ROS2 Humble before building, for example:"
  echo "  source /opt/ros/humble/setup.bash"
fi

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-pip \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-rclpy \
  ros-humble-ros-gz-bridge \
  ros-humble-rviz2 \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs \
  ros-humble-visualization-msgs

python3 -m pip install --user -r requirements.txt

echo "Base dependencies installed."
echo "For YOLO training/inference, run:"
echo "  python3 -m pip install --user -r requirements-yolo.txt"
