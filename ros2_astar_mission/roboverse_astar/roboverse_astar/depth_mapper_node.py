import math
import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import Image

from .common import GridSpec, image_msg_to_depth, line_cells, pose_to_ne_down_yaw


class DepthMapperNode(Node):
    """
    Builds a local 2D occupancy grid from the x500 depth camera.

    The map uses PX4 local NED-like coordinates:
    - OccupancyGrid x axis = local north metres.
    - OccupancyGrid y axis = local east metres.
    - OccupancyGrid origin defaults to (-20, -20), matching the 40m x 40m map.
    """

    def __init__(self):
        super().__init__("depth_mapper_node")

        self.declare_parameter("depth_topic", "/depth_camera")
        self.declare_parameter("pose_topic", "/roboverse/local_pose")
        self.declare_parameter("map_topic", "/roboverse/occupancy")
        self.declare_parameter("map_frame", "px4_ned")
        self.declare_parameter("map_size_m", 40.0)
        self.declare_parameter("resolution_m", 0.5)
        self.declare_parameter("max_depth_m", 11.5)
        self.declare_parameter("min_depth_m", 0.25)
        self.declare_parameter("obstacle_inflation_m", 0.75)
        self.declare_parameter("num_rays", 48)
        self.declare_parameter("camera_fx", 433.0)
        self.declare_parameter("camera_cx", 320.0)
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("process_hz", 3.0)
        self.declare_parameter("robot_clear_radius_m", 0.9)

        size_m = float(self.get_parameter("map_size_m").value)
        resolution_m = float(self.get_parameter("resolution_m").value)
        self.grid_spec = GridSpec(
            size_m=size_m,
            resolution_m=resolution_m,
            origin_north_m=-size_m / 2.0,
            origin_east_m=-size_m / 2.0,
        )
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.obstacle_inflation_cells = int(
            math.ceil(float(self.get_parameter("obstacle_inflation_m").value) / resolution_m)
        )
        self.num_rays = int(self.get_parameter("num_rays").value)
        self.camera_fx = float(self.get_parameter("camera_fx").value)
        self.camera_cx = float(self.get_parameter("camera_cx").value)

        self.pose: Optional[PoseStamped] = None
        self.last_process_time = 0.0
        self.grid = np.full(
            (self.grid_spec.height, self.grid_spec.width),
            -1,
            dtype=np.int8,
        )

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self.pose_callback,
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self.depth_callback,
            1,
        )
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            5,
        )

        self.create_timer(
            1.0 / max(0.5, float(self.get_parameter("publish_hz").value)),
            self.publish_map,
        )

    def pose_callback(self, msg: PoseStamped):
        self.pose = msg

    def clean_depth(self, depth_msg: Image):
        depth = image_msg_to_depth(depth_msg)
        depth[(~np.isfinite(depth)) | (depth > self.max_depth_m)] = self.max_depth_m
        depth[depth < self.min_depth_m] = self.max_depth_m
        return depth

    def depth_callback(self, msg: Image):
        if self.pose is None:
            return

        now = time.monotonic()
        period = 1.0 / max(0.2, float(self.get_parameter("process_hz").value))
        if now - self.last_process_time < period:
            return
        self.last_process_time = now

        try:
            depth = self.clean_depth(msg)
        except Exception as exc:
            self.get_logger().warn(f"Depth conversion failed: {exc}")
            return

        north, east, _down, yaw_rad = pose_to_ne_down_yaw(self.pose)
        start_cell = self.grid_spec.world_to_grid(north, east)
        if not self.grid_spec.in_bounds(*start_cell):
            return

        h, w = depth.shape[:2]
        y1 = int(h * 0.22)
        y2 = int(h * 0.58)
        band = depth[y1:y2, :]

        for idx in range(self.num_rays):
            x1 = int(idx * w / self.num_rays)
            x2 = int((idx + 1) * w / self.num_rays)
            if x2 <= x1:
                continue

            region = band[:, x1:x2]
            valid = region[np.isfinite(region)]
            valid = valid[(valid >= self.min_depth_m) & (valid <= self.max_depth_m)]
            if valid.size < max(5, region.size * 0.03):
                continue

            distance_m = float(np.percentile(valid, 20))
            u = (x1 + x2) * 0.5
            ray_angle = yaw_rad + math.atan((u - self.camera_cx) / max(self.camera_fx, 1e-6))

            hit_is_obstacle = distance_m < self.max_depth_m - 0.2
            free_distance_m = max(0.0, distance_m - self.grid_spec.resolution_m)

            free_n = north + free_distance_m * math.cos(ray_angle)
            free_e = east + free_distance_m * math.sin(ray_angle)
            free_cell = self.grid_spec.world_to_grid(free_n, free_e)

            for cell in line_cells(start_cell[0], start_cell[1], free_cell[0], free_cell[1]):
                if self.grid_spec.in_bounds(*cell):
                    gy, gx = cell[1], cell[0]
                    if self.grid[gy, gx] < 60:
                        self.grid[gy, gx] = 0

            if hit_is_obstacle:
                obs_n = north + distance_m * math.cos(ray_angle)
                obs_e = east + distance_m * math.sin(ray_angle)
                self.mark_obstacle(obs_n, obs_e)

        self.clear_robot_footprint(north, east)

    def mark_obstacle(self, north: float, east: float):
        gx, gy = self.grid_spec.world_to_grid(north, east)
        for dx in range(-self.obstacle_inflation_cells, self.obstacle_inflation_cells + 1):
            for dy in range(-self.obstacle_inflation_cells, self.obstacle_inflation_cells + 1):
                if dx * dx + dy * dy > self.obstacle_inflation_cells * self.obstacle_inflation_cells:
                    continue
                cx, cy = gx + dx, gy + dy
                if self.grid_spec.in_bounds(cx, cy):
                    self.grid[cy, cx] = 100

    def clear_robot_footprint(self, north: float, east: float):
        radius_cells = int(
            math.ceil(
                float(self.get_parameter("robot_clear_radius_m").value)
                / self.grid_spec.resolution_m
            )
        )
        gx, gy = self.grid_spec.world_to_grid(north, east)
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                cx, cy = gx + dx, gy + dy
                if self.grid_spec.in_bounds(cx, cy):
                    self.grid[cy, cx] = 0

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.grid_spec.resolution_m
        msg.info.width = self.grid_spec.width
        msg.info.height = self.grid_spec.height
        msg.info.origin.position.x = self.grid_spec.origin_north_m
        msg.info.origin.position.y = self.grid_spec.origin_east_m
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.reshape(-1).astype(np.int8).tolist()
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthMapperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
