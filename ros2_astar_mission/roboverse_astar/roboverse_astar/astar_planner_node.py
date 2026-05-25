import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node

from .astar import astar_grid, simplify_path
from .common import GridSpec, make_pose, pose_to_ne_down_yaw, quaternion_to_yaw


class AStarPlannerNode(Node):
    """Plans local NED waypoints on the online occupancy grid."""

    def __init__(self):
        super().__init__("astar_planner_node")

        self.declare_parameter("map_topic", "/roboverse/occupancy")
        self.declare_parameter("pose_topic", "/roboverse/local_pose")
        self.declare_parameter("goal_topic", "/roboverse/goal")
        self.declare_parameter("path_topic", "/roboverse/astar_path")
        self.declare_parameter("next_waypoint_topic", "/roboverse/next_waypoint")
        self.declare_parameter("map_frame", "px4_ned")
        self.declare_parameter("replan_hz", 2.0)
        self.declare_parameter("occupied_threshold", 60)
        self.declare_parameter("unknown_cost", 4.0)
        self.declare_parameter("waypoint_lookahead_cells", 5)
        self.declare_parameter("nearest_goal_search_cells", 8)

        self.map_msg: Optional[OccupancyGrid] = None
        self.pose_msg: Optional[PoseStamped] = None
        self.goal_msg: Optional[PoseStamped] = None
        self.last_failure_log_time = 0.0

        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self.map_callback,
            5,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self.pose_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            self.goal_callback,
            10,
        )

        self.path_pub = self.create_publisher(Path, str(self.get_parameter("path_topic").value), 5)
        self.next_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("next_waypoint_topic").value),
            10,
        )
        self.create_timer(
            1.0 / max(0.2, float(self.get_parameter("replan_hz").value)),
            self.plan_once,
        )

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def pose_callback(self, msg: PoseStamped):
        self.pose_msg = msg

    def goal_callback(self, msg: PoseStamped):
        self.goal_msg = msg

    def grid_spec_from_map(self) -> Optional[GridSpec]:
        if self.map_msg is None:
            return None
        info = self.map_msg.info
        return GridSpec(
            size_m=float(info.width) * float(info.resolution),
            resolution_m=float(info.resolution),
            origin_north_m=float(info.origin.position.x),
            origin_east_m=float(info.origin.position.y),
        )

    def plan_once(self):
        if self.map_msg is None or self.pose_msg is None or self.goal_msg is None:
            return

        grid = self.grid_spec_from_map()
        if grid is None:
            return

        current_n, current_e, _current_down, current_yaw = pose_to_ne_down_yaw(self.pose_msg)
        goal_n = float(self.goal_msg.pose.position.x)
        goal_e = float(self.goal_msg.pose.position.y)
        goal_down = float(self.goal_msg.pose.position.z)

        start = grid.world_to_grid(current_n, current_e)
        goal = grid.world_to_grid(goal_n, goal_e)
        original_goal = goal
        goal = self.nearest_passable_goal(goal)
        if goal is None:
            self.log_failure(start, original_goal, "goal occupied and no nearby free cell")
            return

        path_cells = astar_grid(
            self.map_msg.data,
            int(self.map_msg.info.width),
            int(self.map_msg.info.height),
            start,
            goal,
            occupied_threshold=int(self.get_parameter("occupied_threshold").value),
            unknown_cost=float(self.get_parameter("unknown_cost").value),
        )
        if not path_cells:
            self.log_failure(start, goal, "no route through occupancy grid")
            return

        reduced = simplify_path(path_cells, stride=max(1, int(self.get_parameter("waypoint_lookahead_cells").value)))

        path_msg = Path()
        path_msg.header.frame_id = str(self.get_parameter("map_frame").value)
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for gx, gy in reduced:
            north, east = grid.grid_to_world(gx, gy)
            yaw = math.atan2(east - current_e, north - current_n)
            path_msg.poses.append(make_pose(path_msg.header.frame_id, path_msg.header.stamp, north, east, goal_down, yaw))

        self.path_pub.publish(path_msg)

        waypoint_index = 1 if len(reduced) > 1 else 0
        next_n, next_e = grid.grid_to_world(*reduced[waypoint_index])
        yaw = math.atan2(next_e - current_e, next_n - current_n) if len(reduced) > 1 else current_yaw
        self.next_pub.publish(
            make_pose(path_msg.header.frame_id, path_msg.header.stamp, next_n, next_e, goal_down, yaw)
        )

    def nearest_passable_goal(self, goal):
        if self.map_msg is None:
            return goal

        width = int(self.map_msg.info.width)
        height = int(self.map_msg.info.height)
        occupied_threshold = int(self.get_parameter("occupied_threshold").value)

        def is_passable(cell):
            gx, gy = cell
            if not (0 <= gx < width and 0 <= gy < height):
                return False
            return int(self.map_msg.data[gy * width + gx]) < occupied_threshold

        if is_passable(goal):
            return goal

        max_radius = int(self.get_parameter("nearest_goal_search_cells").value)
        best = None
        best_dist = 1e9
        for radius in range(1, max_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    candidate = (goal[0] + dx, goal[1] + dy)
                    if not is_passable(candidate):
                        continue
                    dist = math.hypot(dx, dy)
                    if dist < best_dist:
                        best = candidate
                        best_dist = dist
            if best is not None:
                return best

        return None

    def log_failure(self, start, goal, reason):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_failure_log_time < 3.0:
            return
        self.last_failure_log_time = now
        self.get_logger().warn(
            f"A* failed from {start} to {goal}: {reason}; holding current waypoint."
        )


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
