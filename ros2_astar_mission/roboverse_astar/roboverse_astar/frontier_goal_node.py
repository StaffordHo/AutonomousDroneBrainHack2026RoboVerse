import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

from .common import GridSpec, make_pose, pose_to_ne_down_yaw


class FrontierGoalNode(Node):
    """
    Publishes coverage goals for the 40m x 40m RoboVerse arena.

    This is deliberately more fundamental than the old local heading sweeps:
    it treats the map as a set of 4m-ish lanes, sends end-to-end goals, and lets
    A* route around obstacles discovered by depth mapping.
    """

    def __init__(self):
        super().__init__("frontier_goal_node")

        self.declare_parameter("pose_topic", "/roboverse/local_pose")
        self.declare_parameter("map_topic", "/roboverse/occupancy")
        self.declare_parameter("frontier_goal_topic", "/roboverse/frontier_goal")
        self.declare_parameter("map_frame", "px4_ned")
        self.declare_parameter("arena_half_extent_m", 19.0)
        self.declare_parameter("lane_spacing_m", 4.0)
        self.declare_parameter("low_scan_down_m", -1.5)
        self.declare_parameter("high_scan_down_m", -2.6)
        self.declare_parameter("two_altitude_pass", True)
        self.declare_parameter("goal_reached_radius_m", 1.4)
        self.declare_parameter("publish_hz", 1.0)

        self.pose_msg: Optional[PoseStamped] = None
        self.map_msg: Optional[OccupancyGrid] = None
        self.route: List[Tuple[float, float, float]] = []
        self.route_index = 0
        self.route_initialized = False

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self.pose_callback,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self.map_callback,
            5,
        )
        self.goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("frontier_goal_topic").value),
            10,
        )
        self.create_timer(
            1.0 / max(0.2, float(self.get_parameter("publish_hz").value)),
            self.publish_goal,
        )

    def pose_callback(self, msg: PoseStamped):
        self.pose_msg = msg

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def build_route(self, current_n: float, current_e: float):
        half = float(self.get_parameter("arena_half_extent_m").value)
        spacing = float(self.get_parameter("lane_spacing_m").value)
        low_down = float(self.get_parameter("low_scan_down_m").value)
        high_down = float(self.get_parameter("high_scan_down_m").value)
        two_altitude = bool(self.get_parameter("two_altitude_pass").value)

        coords = []
        value = -half
        while value <= half + 1e-6:
            coords.append(round(value, 2))
            value += spacing
        if coords[-1] < half:
            coords.append(half)

        low_route = []
        for i, north in enumerate(coords):
            east_order = coords if i % 2 == 0 else list(reversed(coords))
            for east in east_order:
                low_route.append((north, east, low_down))

        # Start near the current position, then continue cyclically.
        start_index = min(
            range(len(low_route)),
            key=lambda idx: math.hypot(low_route[idx][0] - current_n, low_route[idx][1] - current_e),
        )
        low_route = low_route[start_index:] + low_route[:start_index]

        self.route = low_route
        if two_altitude:
            self.route.extend((n, e, high_down) for n, e, _ in low_route)

        self.route_initialized = True
        self.route_index = 0
        self.get_logger().info(f"Coverage route initialized with {len(self.route)} goals.")

    def goal_cell_blocked(self, north: float, east: float) -> bool:
        if self.map_msg is None:
            return False
        info = self.map_msg.info
        grid = GridSpec(
            size_m=float(info.width) * float(info.resolution),
            resolution_m=float(info.resolution),
            origin_north_m=float(info.origin.position.x),
            origin_east_m=float(info.origin.position.y),
        )
        gx, gy = grid.world_to_grid(north, east)
        if not grid.in_bounds(gx, gy):
            return True
        return int(self.map_msg.data[gy * int(info.width) + gx]) >= 60

    def publish_goal(self):
        if self.pose_msg is None:
            return

        current_n, current_e, _down, _yaw = pose_to_ne_down_yaw(self.pose_msg)
        if not self.route_initialized:
            self.build_route(current_n, current_e)

        reached_radius = float(self.get_parameter("goal_reached_radius_m").value)

        for _ in range(len(self.route)):
            goal_n, goal_e, goal_down = self.route[self.route_index]
            if self.goal_cell_blocked(goal_n, goal_e):
                self.route_index = (self.route_index + 1) % len(self.route)
                continue

            dist = math.hypot(goal_n - current_n, goal_e - current_e)
            if dist < reached_radius:
                self.route_index = (self.route_index + 1) % len(self.route)
                continue

            yaw = math.atan2(goal_e - current_e, goal_n - current_n)
            self.goal_pub.publish(
                make_pose(
                    str(self.get_parameter("map_frame").value),
                    self.get_clock().now().to_msg(),
                    goal_n,
                    goal_e,
                    goal_down,
                    yaw,
                )
            )
            return


def main(args=None):
    rclpy.init(args=args)
    node = FrontierGoalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
