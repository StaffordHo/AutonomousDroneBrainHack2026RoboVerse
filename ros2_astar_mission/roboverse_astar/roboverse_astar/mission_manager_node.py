import json
import math
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from .common import make_pose, pose_to_ne_down_yaw


class MissionManagerNode(Node):
    """
    Chooses between exploration goals and barrel investigation goals.

    The detector does not directly control the drone. It contributes candidate
    target points; this node prioritizes those candidates, then falls back to
    full-world coverage goals from FrontierGoalNode.
    """

    def __init__(self):
        super().__init__("mission_manager_node")

        self.declare_parameter("pose_topic", "/roboverse/local_pose")
        self.declare_parameter("frontier_goal_topic", "/roboverse/frontier_goal")
        self.declare_parameter("detections_topic", "/roboverse/fuel_detections")
        self.declare_parameter("goal_topic", "/roboverse/goal")
        self.declare_parameter("map_frame", "px4_ned")
        self.declare_parameter("candidate_duplicate_radius_m", 1.6)
        self.declare_parameter("candidate_reached_radius_m", 1.2)
        self.declare_parameter("yellow_visit_down_m", -1.45)
        self.declare_parameter("red_visit_down_m", -2.35)
        self.declare_parameter("red_priority_bonus", 3.0)
        self.declare_parameter("publish_hz", 2.0)

        self.pose_msg: Optional[PoseStamped] = None
        self.frontier_goal: Optional[PoseStamped] = None
        self.candidates: List[Dict] = []
        self.completed: List[Dict] = []

        self.create_subscription(PoseStamped, str(self.get_parameter("pose_topic").value), self.pose_callback, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("frontier_goal_topic").value), self.frontier_callback, 10)
        self.create_subscription(String, str(self.get_parameter("detections_topic").value), self.detection_callback, 20)
        self.goal_pub = self.create_publisher(PoseStamped, str(self.get_parameter("goal_topic").value), 10)
        self.create_timer(1.0 / max(0.2, float(self.get_parameter("publish_hz").value)), self.publish_goal)

    def pose_callback(self, msg: PoseStamped):
        self.pose_msg = msg

    def frontier_callback(self, msg: PoseStamped):
        self.frontier_goal = msg

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

        if self.frontier_goal is not None:
            self.goal_pub.publish(self.frontier_goal)


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
