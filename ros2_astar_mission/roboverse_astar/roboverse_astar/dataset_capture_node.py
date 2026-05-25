import os
from datetime import datetime
from typing import Optional

import cv2
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image

from .common import image_msg_to_bgr, pose_to_ne_down_yaw


class DatasetCaptureNode(Node):
    """Saves ROS2 camera frames for YOLO labeling/training."""

    def __init__(self):
        super().__init__("dataset_capture_node")
        self.declare_parameter("image_topic", "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image")
        self.declare_parameter("pose_topic", "/roboverse/local_pose")
        self.declare_parameter("output_dir", "/home/stafford99/roboverse_qualifier/ros2_astar_mission/datasets/fuel_barrels_v1/captured")
        self.declare_parameter("capture_period_s", 1.0)

        self.latest_frame = None
        self.pose: Optional[PoseStamped] = None
        self.counter = 0
        os.makedirs(str(self.get_parameter("output_dir").value), exist_ok=True)

        self.create_subscription(Image, str(self.get_parameter("image_topic").value), self.image_callback, 5)
        self.create_subscription(PoseStamped, str(self.get_parameter("pose_topic").value), self.pose_callback, 10)
        self.create_timer(float(self.get_parameter("capture_period_s").value), self.save_frame)

    def image_callback(self, msg: Image):
        try:
            self.latest_frame = image_msg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f"Image conversion failed: {exc}")

    def pose_callback(self, msg: PoseStamped):
        self.pose = msg

    def save_frame(self):
        if self.latest_frame is None:
            return
        n, e, d, yaw = pose_to_ne_down_yaw(self.pose)
        stamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S_%f")
        filename = (
            f"frame_{stamp}_n{n:+05.1f}_e{e:+05.1f}_d{d:+04.1f}_yaw{yaw:+.2f}_{self.counter:05d}.jpg"
        )
        path = os.path.join(str(self.get_parameter("output_dir").value), filename)
        cv2.imwrite(path, self.latest_frame)
        self.counter += 1
        self.get_logger().info(f"Saved {path}")


def main(args=None):
    rclpy.init(args=args)
    node = DatasetCaptureNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
