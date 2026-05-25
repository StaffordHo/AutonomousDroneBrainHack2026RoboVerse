#!/usr/bin/env bash
set -euo pipefail

IMAGE_TOPIC="${IMAGE_TOPIC:-/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image}"
DEPTH_TOPIC="${DEPTH_TOPIC:-/depth_camera}"

ros2 run ros_gz_bridge parameter_bridge \
  "${IMAGE_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image" \
  "${DEPTH_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image"
