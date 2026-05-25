#!/usr/bin/env python3
"""Report whether this mission folder is configured to use ROS2 sensor topics."""

import os
import shutil
import subprocess

from ros2_sensor_bridge import ROS2_AVAILABLE


def main():
    use_bridge = os.getenv("USE_ROS2_SENSOR_BRIDGE", "0") == "1"
    ros2_cli = shutil.which("ros2")

    print("ROS2 usage check")
    print("================")
    print(f"USE_ROS2_SENSOR_BRIDGE={os.getenv('USE_ROS2_SENSOR_BRIDGE', '0')}")
    print(f"Mission will use ROS2 bridge: {use_bridge and ROS2_AVAILABLE}")
    print(f"rclpy/sensor_msgs importable: {ROS2_AVAILABLE}")
    print(f"ros2 CLI found: {ros2_cli or 'no'}")
    print(f"ROS2_IMAGE_TOPIC={os.getenv('ROS2_IMAGE_TOPIC', '<default Gazebo image topic>')}")
    print(f"ROS2_DEPTH_TOPIC={os.getenv('ROS2_DEPTH_TOPIC', '/depth_camera')}")

    if not use_bridge:
        print("\nCurrent mission default: Gazebo transport, not ROS2.")
        return

    if not ROS2_AVAILABLE:
        print("\nROS2 bridge was requested, but Python ROS2 packages are unavailable.")
        return

    if ros2_cli is None:
        print("\nROS2 bridge can import Python packages, but the ros2 CLI is not on PATH.")
        return

    try:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            check=False,
            capture_output=True,
            text=True,
            timeout=3.0,
        )
    except Exception as error:
        print(f"\nCould not query ros2 topic list: {error}")
        return

    print("\nros2 topic list:")
    print(result.stdout.strip() or "<no topics reported>")
    if result.stderr.strip():
        print("\nros2 stderr:")
        print(result.stderr.strip())


if __name__ == "__main__":
    main()
