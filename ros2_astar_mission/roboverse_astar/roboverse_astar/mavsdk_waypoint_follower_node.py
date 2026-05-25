import asyncio
import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from .common import make_pose, quaternion_to_yaw

try:
    from mavsdk import System
    from mavsdk.offboard import OffboardError, PositionNedYaw
except Exception:  # pragma: no cover - runtime dependency
    System = None
    OffboardError = None
    PositionNedYaw = None


class MavsdkWaypointFollowerNode(Node):
    """
    ROS2 wrapper around the already-working MAVSDK command path.

    Use this while px4_msgs is unavailable. It publishes `/roboverse/local_pose`
    for the ROS2 mapper/planner and follows `/roboverse/next_waypoint`.
    """

    def __init__(self):
        super().__init__("mavsdk_waypoint_follower_node")

        self.declare_parameter("next_waypoint_topic", "/roboverse/next_waypoint")
        self.declare_parameter("direct_goal_topic", "/roboverse/goal")
        self.declare_parameter("local_pose_topic", "/roboverse/local_pose")
        self.declare_parameter("system_address", "udpin://0.0.0.0:14540")
        self.declare_parameter("map_frame", "px4_ned")
        self.declare_parameter("arm_and_takeoff", True)
        self.declare_parameter("takeoff_down_m", -1.5)
        self.declare_parameter("command_hz", 10.0)
        self.declare_parameter("default_yaw_deg", 0.0)
        self.declare_parameter("disable_gcs_failsafe", True)
        self.declare_parameter("direct_goal_fallback_s", 1.5)
        self.declare_parameter("direct_goal_max_step_m", 0.8)

        self.next_waypoint: Optional[PoseStamped] = None
        self.direct_goal: Optional[PoseStamped] = None
        self.last_waypoint_time = 0.0
        self.last_offboard_error_log_time = 0.0
        self.current_n = 0.0
        self.current_e = 0.0
        self.current_d = float(self.get_parameter("takeoff_down_m").value)
        self.current_yaw_deg = float(self.get_parameter("default_yaw_deg").value)
        self.running = True

        self.local_pose_pub = self.create_publisher(PoseStamped, str(self.get_parameter("local_pose_topic").value), 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("next_waypoint_topic").value), self.waypoint_callback, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("direct_goal_topic").value), self.direct_goal_callback, 10)

        if System is None:
            self.get_logger().error("mavsdk is not importable; waypoint follower cannot run.")
            return

        self.thread = threading.Thread(target=self.run_asyncio, daemon=True)
        self.thread.start()

    def waypoint_callback(self, msg: PoseStamped):
        self.next_waypoint = msg
        self.last_waypoint_time = time.monotonic()

    def direct_goal_callback(self, msg: PoseStamped):
        self.direct_goal = msg

    def run_asyncio(self):
        asyncio.run(self.mavsdk_main())

    async def mavsdk_main(self):
        drone = System()
        system_address = self.normalized_system_address(
            str(self.get_parameter("system_address").value)
        )
        self.get_logger().info(f"Connecting MAVSDK using {system_address}")
        await drone.connect(system_address=system_address)

        self.get_logger().info("Waiting for MAVSDK vehicle connection...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                break

        self.get_logger().info("Waiting for local position estimate...")
        async for health in drone.telemetry.health():
            if health.is_local_position_ok:
                break

        await self.configure_px4_for_autonomy(drone)

        telemetry_task = asyncio.create_task(self.telemetry_loop(drone))

        takeoff_down = float(self.get_parameter("takeoff_down_m").value)
        if bool(self.get_parameter("arm_and_takeoff").value):
            await drone.action.set_takeoff_altitude(abs(takeoff_down))
            await drone.action.arm()
            await drone.action.takeoff()
            await asyncio.sleep(7.0)

        await drone.offboard.set_position_ned(
            PositionNedYaw(self.current_n, self.current_e, takeoff_down, self.current_yaw_deg)
        )
        try:
            await drone.offboard.start()
            self.get_logger().info("MAVSDK offboard started.")
        except OffboardError as exc:
            self.get_logger().error(f"Offboard start failed: {exc}")
            return

        period = 1.0 / max(2.0, float(self.get_parameter("command_hz").value))
        while self.running:
            target_setpoint = self.choose_setpoint(takeoff_down)
            try:
                await drone.offboard.set_position_ned(
                    target_setpoint
                )
            except Exception as exc:
                now = time.monotonic()
                if now - self.last_offboard_error_log_time > 2.0:
                    self.last_offboard_error_log_time = now
                    self.get_logger().warn(f"Offboard setpoint send failed: {exc}")
            await asyncio.sleep(period)

        telemetry_task.cancel()

    async def telemetry_loop(self, drone):
        attitude_task = asyncio.create_task(self.attitude_loop(drone))
        async for pv in drone.telemetry.position_velocity_ned():
            self.current_n = float(pv.position.north_m)
            self.current_e = float(pv.position.east_m)
            self.current_d = float(pv.position.down_m)
            pose = make_pose(
                str(self.get_parameter("map_frame").value),
                self.get_clock().now().to_msg(),
                self.current_n,
                self.current_e,
                self.current_d,
                math.radians(self.current_yaw_deg),
            )
            self.local_pose_pub.publish(pose)
            if not self.running:
                break
        attitude_task.cancel()

    async def attitude_loop(self, drone):
        async for attitude in drone.telemetry.attitude_euler():
            self.current_yaw_deg = float(attitude.yaw_deg)
            if not self.running:
                break

    @staticmethod
    def normalized_system_address(system_address: str) -> str:
        if system_address == "udpin://:14540":
            return "udpin://0.0.0.0:14540"
        if system_address == "udp://:14540":
            return "udpin://0.0.0.0:14540"
        return system_address

    async def configure_px4_for_autonomy(self, drone):
        if not bool(self.get_parameter("disable_gcs_failsafe").value):
            return

        updates = [
            ("NAV_DLL_ACT", 0, "int"),
            ("COM_DL_LOSS_T", 600.0, "float"),
        ]

        for name, value, preferred_kind in updates:
            await self.set_px4_param_best_effort(drone, name, value, preferred_kind)

    async def set_px4_param_best_effort(self, drone, name: str, value, preferred_kind: str):
        kinds = [preferred_kind]
        fallback = "float" if preferred_kind == "int" else "int"
        kinds.append(fallback)

        last_exc = None
        for kind in kinds:
            try:
                if kind == "int":
                    await drone.param.set_param_int(name, int(value))
                else:
                    await drone.param.set_param_float(name, float(value))
                self.get_logger().info(f"PX4 param {name} set to {value} as {kind}")
                return
            except Exception as exc:
                last_exc = exc

        self.get_logger().warn(f"Could not set PX4 param {name}: {last_exc}")

    def choose_setpoint(self, takeoff_down: float):
        if self.next_waypoint is not None:
            max_age = float(self.get_parameter("direct_goal_fallback_s").value)
            if time.monotonic() - self.last_waypoint_time <= max_age:
                return self.pose_to_setpoint(self.next_waypoint, takeoff_down)

        if self.direct_goal is not None:
            return self.direct_goal_step_setpoint(self.direct_goal, takeoff_down)

        return PositionNedYaw(
            self.current_n,
            self.current_e,
            takeoff_down,
            self.current_yaw_deg,
        )

    def pose_to_setpoint(self, pose: PoseStamped, fallback_down: float):
        yaw_deg = math.degrees(quaternion_to_yaw(pose.pose.orientation))
        down = float(pose.pose.position.z)
        if not math.isfinite(down):
            down = fallback_down
        return PositionNedYaw(
            float(pose.pose.position.x),
            float(pose.pose.position.y),
            down,
            yaw_deg,
        )

    def direct_goal_step_setpoint(self, goal: PoseStamped, fallback_down: float):
        goal_n = float(goal.pose.position.x)
        goal_e = float(goal.pose.position.y)
        goal_down = float(goal.pose.position.z)
        if not math.isfinite(goal_down):
            goal_down = fallback_down

        dn = goal_n - self.current_n
        de = goal_e - self.current_e
        dist = math.hypot(dn, de)
        max_step = max(0.1, float(self.get_parameter("direct_goal_max_step_m").value))

        if dist > max_step:
            scale = max_step / dist
            target_n = self.current_n + dn * scale
            target_e = self.current_e + de * scale
        else:
            target_n = goal_n
            target_e = goal_e

        yaw_deg = self.current_yaw_deg
        if dist > 0.05:
            yaw_deg = math.degrees(math.atan2(de, dn))

        return PositionNedYaw(target_n, target_e, goal_down, yaw_deg)

    def destroy_node(self):
        self.running = False
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MavsdkWaypointFollowerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
