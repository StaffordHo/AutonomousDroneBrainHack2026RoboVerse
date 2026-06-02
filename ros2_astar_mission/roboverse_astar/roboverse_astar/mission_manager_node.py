import json
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String

from .common import GridSpec, make_pose, pose_to_ne_down_yaw


class MissionManagerNode(Node):
    """
    Chooses between exploration goals and barrel investigation goals.

    The detector does not directly control the drone. It contributes candidate
    target points; this node prioritizes those candidates, then falls back to
    coverage goals. Coverage can come from the standalone FrontierGoalNode or
    from this node's lightweight internal route when the machine is under load.
    """

    def __init__(self):
        super().__init__("mission_manager_node")

        self.declare_parameter("pose_topic", "/roboverse/local_pose")
        self.declare_parameter("frontier_goal_topic", "/roboverse/frontier_goal")
        self.declare_parameter("frontier_goal_timeout_s", 3.0)
        self.declare_parameter("detections_topic", "/roboverse/fuel_detections")
        self.declare_parameter("map_topic", "/roboverse/occupancy")
        self.declare_parameter("goal_topic", "/roboverse/goal")
        self.declare_parameter("map_frame", "px4_ned")
        self.declare_parameter("candidate_duplicate_radius_m", 1.6)
        self.declare_parameter("candidate_reached_radius_m", 1.2)
        self.declare_parameter("yellow_visit_down_m", -1.45)
        self.declare_parameter("red_visit_down_m", -2.35)
        self.declare_parameter("red_priority_bonus", 3.0)
        self.declare_parameter("enable_internal_coverage", True)
        self.declare_parameter("arena_half_extent_m", 19.0)
        self.declare_parameter("lane_spacing_m", 4.0)
        self.declare_parameter("low_scan_down_m", -1.5)
        self.declare_parameter("high_scan_down_m", -2.6)
        self.declare_parameter("two_altitude_pass", True)
        self.declare_parameter("coverage_reached_radius_m", 1.4)
        self.declare_parameter("publish_hz", 2.0)

        self.pose_msg: Optional[PoseStamped] = None
        self.frontier_goal: Optional[PoseStamped] = None
        self.frontier_goal_time_s = 0.0
        self.map_msg: Optional[OccupancyGrid] = None
        self.candidates: List[Dict] = []
        self.completed: List[Dict] = []
        self.coverage_route: List[Tuple[float, float, float]] = []
        self.coverage_route_index = 0
        self.coverage_route_initialized = False

        self.create_subscription(PoseStamped, str(self.get_parameter("pose_topic").value), self.pose_callback, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("frontier_goal_topic").value), self.frontier_callback, 10)
        self.create_subscription(OccupancyGrid, str(self.get_parameter("map_topic").value), self.map_callback, 5)
        self.create_subscription(String, str(self.get_parameter("detections_topic").value), self.detection_callback, 20)
        self.goal_pub = self.create_publisher(PoseStamped, str(self.get_parameter("goal_topic").value), 10)
        self.create_timer(1.0 / max(0.2, float(self.get_parameter("publish_hz").value)), self.publish_goal)

    def pose_callback(self, msg: PoseStamped):
        self.pose_msg = msg

    def frontier_callback(self, msg: PoseStamped):
        self.frontier_goal = msg
        self.frontier_goal_time_s = self.get_clock().now().nanoseconds * 1e-9

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def detection_callback(self, msg: String):
        try:
            det = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        visit_n = det.get("visit_north_m")
        visit_e = det.get("visit_east_m")
        if visit_n is None or visit_e is None:
            return

        if self.is_duplicate(float(visit_n), float(visit_e)):
            return

        det["priority"] = self.score_candidate(det)
        self.candidates.append(det)
        self.candidates.sort(key=lambda item: item.get("priority", 0.0), reverse=True)
        self.get_logger().info(
            f"Queued {det.get('label')} candidate conf={det.get('confidence', 0.0):.2f} "
            f"visit=({visit_n:.1f},{visit_e:.1f}) priority={det['priority']:.1f}"
        )

    def is_duplicate(self, north: float, east: float) -> bool:
        radius = float(self.get_parameter("candidate_duplicate_radius_m").value)
        for item in self.candidates + self.completed:
            ref_n = item.get("visit_north_m")
            ref_e = item.get("visit_east_m")
            if ref_n is None or ref_e is None:
                continue
            if math.hypot(north - float(ref_n), east - float(ref_e)) < radius:
                return True
        return False

    def score_candidate(self, det: Dict) -> float:
        confidence = float(det.get("confidence", 0.0))
        depth = det.get("depth_m")
        score = confidence * 10.0
        if depth is not None:
            score += max(0.0, 10.0 - float(depth)) * 0.4
        if det.get("label") == "red_fuel_barrel":
            score += float(self.get_parameter("red_priority_bonus").value)
        return score

    def publish_goal(self):
        if self.pose_msg is None:
            return

        current_n, current_e, _down, _yaw = pose_to_ne_down_yaw(self.pose_msg)
        reached_radius = float(self.get_parameter("candidate_reached_radius_m").value)

        while self.candidates:
            active = self.candidates[0]
            visit_n = float(active["visit_north_m"])
            visit_e = float(active["visit_east_m"])
            if math.hypot(visit_n - current_n, visit_e - current_e) <= reached_radius:
                self.completed.append(self.candidates.pop(0))
                continue

            down = (
                float(self.get_parameter("red_visit_down_m").value)
                if active.get("label") == "red_fuel_barrel"
                else float(self.get_parameter("yellow_visit_down_m").value)
            )
            yaw = math.atan2(visit_e - current_e, visit_n - current_n)
            self.goal_pub.publish(
                make_pose(
                    str(self.get_parameter("map_frame").value),
                    self.get_clock().now().to_msg(),
                    visit_n,
                    visit_e,
                    down,
                    yaw,
                )
            )
            return

        now_s = self.get_clock().now().nanoseconds * 1e-9
        frontier_timeout = float(self.get_parameter("frontier_goal_timeout_s").value)
        if self.frontier_goal is not None and now_s - self.frontier_goal_time_s <= frontier_timeout:
            self.goal_pub.publish(self.frontier_goal)
            return

        if bool(self.get_parameter("enable_internal_coverage").value):
            self.publish_internal_coverage_goal(current_n, current_e)

    def publish_internal_coverage_goal(self, current_n: float, current_e: float):
        if not self.coverage_route_initialized:
            self.build_coverage_route(current_n, current_e)

        if not self.coverage_route:
            return

        reached_radius = float(self.get_parameter("coverage_reached_radius_m").value)

        for _ in range(len(self.coverage_route)):
            goal_n, goal_e, goal_down = self.coverage_route[self.coverage_route_index]
            if self.goal_cell_blocked(goal_n, goal_e):
                self.coverage_route_index = (self.coverage_route_index + 1) % len(self.coverage_route)
                continue

            dist = math.hypot(goal_n - current_n, goal_e - current_e)
            if dist < reached_radius:
                self.coverage_route_index = (self.coverage_route_index + 1) % len(self.coverage_route)
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

    def build_coverage_route(self, current_n: float, current_e: float):
        half = float(self.get_parameter("arena_half_extent_m").value)
        spacing = max(0.5, float(self.get_parameter("lane_spacing_m").value))
        low_down = float(self.get_parameter("low_scan_down_m").value)
        high_down = float(self.get_parameter("high_scan_down_m").value)
        two_altitude = bool(self.get_parameter("two_altitude_pass").value)

        coords = []
        value = -half
        while value <= half + 1e-6:
            coords.append(round(value, 2))
            value += spacing
        if not coords or coords[-1] < half:
            coords.append(round(half, 2))

        low_route = []
        for i, north in enumerate(coords):
            east_order = coords if i % 2 == 0 else list(reversed(coords))
            for east in east_order:
                low_route.append((north, east, low_down))

        start_index = min(
            range(len(low_route)),
            key=lambda idx: math.hypot(low_route[idx][0] - current_n, low_route[idx][1] - current_e),
        )
        low_route = low_route[start_index:] + low_route[:start_index]

        self.coverage_route = low_route
        if two_altitude:
            self.coverage_route.extend((n, e, high_down) for n, e, _ in low_route)

        self.coverage_route_initialized = True
        self.coverage_route_index = 0
        self.get_logger().info(
            f"Internal coverage route initialized with {len(self.coverage_route)} goals."
        )

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
        width = int(info.width)
        height = int(info.height)
        if not (0 <= gx < width and 0 <= gy < height):
            return True

        return int(self.map_msg.data[gy * width + gx]) >= 60


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
