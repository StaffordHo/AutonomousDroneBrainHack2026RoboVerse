import json
import math
import time
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from .common import pose_to_ne_down_yaw


class RoboVerseVisualizerNode(Node):
    """Publishes RViz/Foxglove-friendly views of the MAVSDK-driven mission."""

    def __init__(self):
        super().__init__("roboverse_visualizer_node")

        self.declare_parameter("pose_topic", "/roboverse/local_pose")
        self.declare_parameter("status_topic", "/roboverse/follower_status")
        self.declare_parameter("detections_topic", "/roboverse/fuel_detections")
        self.declare_parameter("goal_topic", "/roboverse/goal")
        self.declare_parameter("next_waypoint_topic", "/roboverse/next_waypoint")
        self.declare_parameter("astar_path_topic", "/roboverse/astar_path")
        self.declare_parameter("flight_path_topic", "/roboverse/flight_path")
        self.declare_parameter("marker_topic", "/roboverse/markers")
        self.declare_parameter("map_frame", "px4_ned")
        self.declare_parameter("publish_hz", 3.0)
        self.declare_parameter("path_max_poses", 1500)
        self.declare_parameter("path_min_step_m", 0.12)
        self.declare_parameter("detection_hold_s", 180.0)
        self.declare_parameter("detection_duplicate_radius_m", 1.0)
        self.declare_parameter("command_arrow_scale_s", 5.0)
        self.declare_parameter("depth_ray_max_m", 8.0)
        self.declare_parameter("depth_ray_half_fov_deg", 28.0)
        self.declare_parameter("show_arena", True)
        self.declare_parameter("arena_min_n_m", 0.6)
        self.declare_parameter("arena_max_n_m", 38.0)
        self.declare_parameter("arena_min_e_m", 0.6)
        self.declare_parameter("arena_max_e_m", 38.0)

        self.pose_msg: Optional[PoseStamped] = None
        self.status: Dict = {}
        self.goal_msg: Optional[PoseStamped] = None
        self.next_waypoint_msg: Optional[PoseStamped] = None
        self.astar_path_msg: Optional[Path] = None
        self.detections: List[Dict] = []
        self.flight_path = Path()
        self.flight_path.header.frame_id = str(self.get_parameter("map_frame").value)

        self.create_subscription(PoseStamped, str(self.get_parameter("pose_topic").value), self.pose_callback, 20)
        self.create_subscription(String, str(self.get_parameter("status_topic").value), self.status_callback, 10)
        self.create_subscription(String, str(self.get_parameter("detections_topic").value), self.detection_callback, 50)
        self.create_subscription(PoseStamped, str(self.get_parameter("goal_topic").value), self.goal_callback, 10)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("next_waypoint_topic").value),
            self.next_waypoint_callback,
            10,
        )
        self.create_subscription(Path, str(self.get_parameter("astar_path_topic").value), self.astar_path_callback, 5)

        self.path_pub = self.create_publisher(Path, str(self.get_parameter("flight_path_topic").value), 5)
        self.marker_pub = self.create_publisher(MarkerArray, str(self.get_parameter("marker_topic").value), 5)

        self.create_timer(
            1.0 / max(0.2, float(self.get_parameter("publish_hz").value)),
            self.publish_visualization,
        )

    def pose_callback(self, msg: PoseStamped):
        self.pose_msg = msg
        if msg.header.frame_id:
            self.flight_path.header.frame_id = msg.header.frame_id
        self.append_flight_path(msg)

    def status_callback(self, msg: String):
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Ignoring malformed follower status JSON.")

    def detection_callback(self, msg: String):
        try:
            det = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        north = self.first_number(det, "target_north_m", "visit_north_m")
        east = self.first_number(det, "target_east_m", "visit_east_m")
        if north is None or east is None:
            return

        det["received_monotonic_s"] = time.monotonic()
        duplicate_radius = float(self.get_parameter("detection_duplicate_radius_m").value)
        for existing in self.detections:
            existing_n = self.first_number(existing, "target_north_m", "visit_north_m")
            existing_e = self.first_number(existing, "target_east_m", "visit_east_m")
            if existing_n is None or existing_e is None:
                continue
            if det.get("label") == existing.get("label") and math.hypot(north - existing_n, east - existing_e) <= duplicate_radius:
                existing.update(det)
                return
        self.detections.append(det)

    def goal_callback(self, msg: PoseStamped):
        self.goal_msg = msg

    def next_waypoint_callback(self, msg: PoseStamped):
        self.next_waypoint_msg = msg

    def astar_path_callback(self, msg: Path):
        self.astar_path_msg = msg

    def append_flight_path(self, msg: PoseStamped):
        if self.flight_path.poses:
            last = self.flight_path.poses[-1].pose.position
            current = msg.pose.position
            step_m = math.sqrt(
                (current.x - last.x) ** 2
                + (current.y - last.y) ** 2
                + (current.z - last.z) ** 2
            )
            if step_m < float(self.get_parameter("path_min_step_m").value):
                return

        pose_copy = PoseStamped()
        pose_copy.header = msg.header
        pose_copy.pose = msg.pose
        self.flight_path.poses.append(pose_copy)

        max_poses = max(10, int(self.get_parameter("path_max_poses").value))
        if len(self.flight_path.poses) > max_poses:
            self.flight_path.poses = self.flight_path.poses[-max_poses:]

    def publish_visualization(self):
        now_msg = self.get_clock().now().to_msg()
        self.prune_detections()

        self.flight_path.header.stamp = now_msg
        self.path_pub.publish(self.flight_path)

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        marker_id = 0

        def add(marker: Marker):
            nonlocal marker_id
            marker.id = marker_id
            marker_id += 1
            marker.header.stamp = now_msg
            if not marker.header.frame_id:
                marker.header.frame_id = self.frame_id()
            markers.markers.append(marker)

        if bool(self.get_parameter("show_arena").value):
            add(self.arena_marker())

        if self.pose_msg is not None:
            add(self.drone_marker())
            command_marker = self.command_marker()
            if command_marker is not None:
                add(command_marker)
            for ray_marker in self.depth_ray_markers():
                add(ray_marker)
            status_marker = self.status_text_marker()
            if status_marker is not None:
                add(status_marker)

        if self.astar_path_msg is not None:
            astar_marker = self.astar_path_marker()
            if astar_marker is not None:
                add(astar_marker)

        if self.goal_msg is not None:
            add(self.pose_sphere_marker("goal", self.goal_msg, self.color(0.15, 0.55, 1.0, 0.95), 0.55))
        if self.next_waypoint_msg is not None:
            add(self.pose_cube_marker("next_waypoint", self.next_waypoint_msg, self.color(0.95, 0.95, 1.0, 0.95), 0.38))

        for marker in self.danger_zone_markers():
            add(marker)
        for marker in self.detection_markers():
            add(marker)

        self.marker_pub.publish(markers)

    def frame_id(self) -> str:
        if self.pose_msg is not None and self.pose_msg.header.frame_id:
            return self.pose_msg.header.frame_id
        return str(self.status.get("frame_id") or self.get_parameter("map_frame").value)

    def arena_marker(self) -> Marker:
        arena = self.status.get("arena", {}) if isinstance(self.status.get("arena"), dict) else {}
        min_n = float(arena.get("min_n_m", self.get_parameter("arena_min_n_m").value))
        max_n = float(arena.get("max_n_m", self.get_parameter("arena_max_n_m").value))
        min_e = float(arena.get("min_e_m", self.get_parameter("arena_min_e_m").value))
        max_e = float(arena.get("max_e_m", self.get_parameter("arena_max_e_m").value))

        marker = self.base_marker("arena", Marker.LINE_STRIP)
        marker.scale.x = 0.08
        marker.color = self.color(0.70, 0.76, 0.82, 0.9)
        marker.points = [
            self.point(min_n, min_e, 0.0),
            self.point(max_n, min_e, 0.0),
            self.point(max_n, max_e, 0.0),
            self.point(min_n, max_e, 0.0),
            self.point(min_n, min_e, 0.0),
        ]
        return marker

    def drone_marker(self) -> Marker:
        north, east, down, yaw = pose_to_ne_down_yaw(self.pose_msg)
        marker = self.base_marker("drone", Marker.ARROW)
        marker.scale.x = 0.10
        marker.scale.y = 0.28
        marker.scale.z = 0.36
        marker.color = self.color(0.10, 0.85, 0.95, 0.95)
        marker.points = [
            self.point(north, east, down),
            self.point(north + math.cos(yaw) * 1.25, east + math.sin(yaw) * 1.25, down),
        ]
        return marker

    def command_marker(self) -> Optional[Marker]:
        command = self.status.get("command", {}) if isinstance(self.status.get("command"), dict) else {}
        north_v = self.number(command.get("ned_north_m_s"), 0.0)
        east_v = self.number(command.get("ned_east_m_s"), 0.0)
        down_v = self.number(command.get("ned_down_m_s"), 0.0)
        if math.sqrt(north_v * north_v + east_v * east_v + down_v * down_v) < 0.01:
            return None

        north, east, down, _yaw = pose_to_ne_down_yaw(self.pose_msg)
        scale_s = float(self.get_parameter("command_arrow_scale_s").value)
        marker = self.base_marker("command_velocity", Marker.ARROW)
        marker.scale.x = 0.07
        marker.scale.y = 0.20
        marker.scale.z = 0.28
        marker.color = self.color(1.0, 0.55, 0.10, 0.95)
        marker.points = [
            self.point(north, east, down),
            self.point(north + north_v * scale_s, east + east_v * scale_s, down + down_v * scale_s),
        ]
        return marker

    def depth_ray_markers(self) -> List[Marker]:
        depth = self.status.get("depth", {}) if isinstance(self.status.get("depth"), dict) else {}
        if not depth:
            return []

        north, east, down, yaw = pose_to_ne_down_yaw(self.pose_msg)
        half_fov = math.radians(float(self.get_parameter("depth_ray_half_fov_deg").value))
        max_range = float(self.get_parameter("depth_ray_max_m").value)
        rays = [
            ("depth_left", -half_fov, self.number(depth.get("left_m"), max_range)),
            ("depth_center", 0.0, self.number(depth.get("center_m"), max_range)),
            ("depth_right", half_fov, self.number(depth.get("right_m"), max_range)),
        ]

        markers = []
        for name, offset, distance in rays:
            distance = max(0.0, min(max_range, distance))
            marker = self.base_marker(name, Marker.LINE_LIST)
            marker.scale.x = 0.06
            marker.color = self.depth_color(distance)
            heading = yaw + offset
            marker.points = [
                self.point(north, east, down),
                self.point(north + math.cos(heading) * distance, east + math.sin(heading) * distance, down),
            ]
            markers.append(marker)
        return markers

    def status_text_marker(self) -> Optional[Marker]:
        if not self.status:
            return None
        north, east, down, _yaw = pose_to_ne_down_yaw(self.pose_msg)
        state = str(self.status.get("state", "unknown"))
        hard_stop = self.status.get("hard_stop", {}) if isinstance(self.status.get("hard_stop"), dict) else {}
        reason = str(hard_stop.get("reason", ""))
        if bool(hard_stop.get("active")) and reason:
            state = f"{state}: {reason[:64]}"

        marker = self.base_marker("status_text", Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = north
        marker.pose.position.y = east
        marker.pose.position.z = down - 0.65
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.34
        marker.color = self.color(1.0, 1.0, 1.0, 0.95)
        marker.text = state
        return marker

    def astar_path_marker(self) -> Optional[Marker]:
        if not self.astar_path_msg.poses:
            return None
        marker = self.base_marker("astar_path", Marker.LINE_STRIP)
        marker.header.frame_id = self.astar_path_msg.header.frame_id or self.frame_id()
        marker.scale.x = 0.06
        marker.color = self.color(0.30, 0.85, 0.35, 0.92)
        marker.points = [
            self.point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            for pose in self.astar_path_msg.poses
        ]
        return marker

    def pose_sphere_marker(self, namespace: str, pose: PoseStamped, color: ColorRGBA, size: float) -> Marker:
        marker = self.base_marker(namespace, Marker.SPHERE)
        marker.header.frame_id = pose.header.frame_id or self.frame_id()
        marker.pose = pose.pose
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color = color
        return marker

    def pose_cube_marker(self, namespace: str, pose: PoseStamped, color: ColorRGBA, size: float) -> Marker:
        marker = self.base_marker(namespace, Marker.CUBE)
        marker.header.frame_id = pose.header.frame_id or self.frame_id()
        marker.pose = pose.pose
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color = color
        return marker

    def danger_zone_markers(self) -> List[Marker]:
        zones = self.status.get("danger_zones", [])
        if not isinstance(zones, list):
            return []

        markers = []
        for zone in zones:
            if not isinstance(zone, dict):
                continue
            north = self.number(zone.get("north_m"), None)
            east = self.number(zone.get("east_m"), None)
            radius = self.number(zone.get("radius_m"), None)
            if north is None or east is None or radius is None:
                continue
            marker = self.base_marker("danger_zone", Marker.CYLINDER)
            marker.pose.position.x = north
            marker.pose.position.y = east
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = radius * 2.0
            marker.scale.y = radius * 2.0
            marker.scale.z = 0.12
            marker.color = self.color(1.0, 0.12, 0.08, 0.28 if not zone.get("static") else 0.42)
            markers.append(marker)
        return markers

    def detection_markers(self) -> List[Marker]:
        markers = []
        for det in self.detections:
            target_n = self.number(det.get("target_north_m"), None)
            target_e = self.number(det.get("target_east_m"), None)
            visit_n = self.number(det.get("visit_north_m"), None)
            visit_e = self.number(det.get("visit_east_m"), None)
            label = str(det.get("label", "fuel_barrel"))
            color = self.label_color(label)

            if target_n is not None and target_e is not None:
                marker = self.base_marker(f"{label}_target", Marker.SPHERE)
                marker.pose.position.x = target_n
                marker.pose.position.y = target_e
                marker.pose.position.z = -0.12
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.55
                marker.scale.y = 0.55
                marker.scale.z = 0.28
                marker.color = color
                markers.append(marker)

            if visit_n is not None and visit_e is not None:
                marker = self.base_marker(f"{label}_visit", Marker.CUBE)
                marker.pose.position.x = visit_n
                marker.pose.position.y = visit_e
                marker.pose.position.z = -0.20
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.36
                marker.scale.y = 0.36
                marker.scale.z = 0.20
                marker.color = self.color(color.r, color.g, color.b, 0.55)
                markers.append(marker)
        return markers

    def prune_detections(self):
        hold_s = max(1.0, float(self.get_parameter("detection_hold_s").value))
        now = time.monotonic()
        self.detections = [
            det for det in self.detections if now - float(det.get("received_monotonic_s", now)) <= hold_s
        ]

    def base_marker(self, namespace: str, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id()
        marker.ns = namespace
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def point(x: float, y: float, z: float) -> Point:
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    @staticmethod
    def color(r: float, g: float, b: float, a: float) -> ColorRGBA:
        color = ColorRGBA()
        color.r = float(r)
        color.g = float(g)
        color.b = float(b)
        color.a = float(a)
        return color

    def depth_color(self, distance_m: float) -> ColorRGBA:
        if distance_m <= 1.15:
            return self.color(1.0, 0.10, 0.08, 0.95)
        if distance_m <= 2.30:
            return self.color(1.0, 0.76, 0.12, 0.95)
        return self.color(0.10, 0.95, 0.45, 0.85)

    def label_color(self, label: str) -> ColorRGBA:
        if "red" in label:
            return self.color(1.0, 0.18, 0.12, 0.90)
        if "yellow" in label:
            return self.color(1.0, 0.86, 0.08, 0.90)
        return self.color(0.80, 0.80, 0.80, 0.80)

    @staticmethod
    def number(value, default):
        try:
            if value is None:
                return default
            result = float(value)
            if not math.isfinite(result):
                return default
            return result
        except (TypeError, ValueError):
            return default

    def first_number(self, data: Dict, *keys: str):
        for key in keys:
            value = self.number(data.get(key), None)
            if value is not None:
                return value
        return None


def main(args=None):
    rclpy.init(args=args)
    node = RoboVerseVisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
