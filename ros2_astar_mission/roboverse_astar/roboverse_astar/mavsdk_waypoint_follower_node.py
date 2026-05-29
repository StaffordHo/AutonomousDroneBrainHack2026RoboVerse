import asyncio
import math
import os
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image

from .common import image_msg_to_depth, make_pose, quaternion_to_yaw

try:
    from mavsdk import System
    from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw
except Exception:  # pragma: no cover - runtime dependency
    System = None
    OffboardError = None
    PositionNedYaw = None
    VelocityNedYaw = None


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
        self.declare_parameter("offboard_control_mode", "position")
        self.declare_parameter("velocity_source", "pattern")
        self.declare_parameter("command_hz", 4.0)
        self.declare_parameter("local_pose_publish_hz", 5.0)
        self.declare_parameter("mavsdk_position_rate_hz", 5.0)
        self.declare_parameter("mavsdk_attitude_rate_hz", 5.0)
        self.declare_parameter("set_mavsdk_stream_rates", False)
        self.declare_parameter("default_yaw_deg", 0.0)
        self.declare_parameter("disable_gcs_failsafe", True)
        self.declare_parameter("direct_goal_fallback_s", 1.5)
        self.declare_parameter("direct_goal_max_step_m", 0.8)
        self.declare_parameter("instance_lock_path", "/tmp/roboverse_mavsdk_waypoint_follower.lock")
        self.declare_parameter("enable_follower_coverage", False)
        self.declare_parameter("follower_coverage_half_extent_m", 6.0)
        self.declare_parameter("follower_coverage_lane_spacing_m", 3.0)
        self.declare_parameter("follower_coverage_reached_radius_m", 0.8)
        self.declare_parameter("follower_velocity_speed_m_s", 0.35)
        self.declare_parameter("follower_velocity_leg_s", 4.0)
        self.declare_parameter("follower_velocity_pause_s", 1.0)
        self.declare_parameter("follower_velocity_yaw_deg", 0.0)
        self.declare_parameter("depth_topic", "/depth_camera")
        self.declare_parameter("depth_process_hz", 3.0)
        self.declare_parameter("depth_max_range_m", 11.5)
        self.declare_parameter("depth_min_range_m", 0.25)
        self.declare_parameter("depth_stale_timeout_s", 2.0)
        self.declare_parameter("depth_safe_distance_m", 2.2)
        self.declare_parameter("depth_slow_distance_m", 4.0)
        self.declare_parameter("depth_critical_distance_m", 1.05)
        self.declare_parameter("depth_strafe_speed_m_s", 0.18)
        self.declare_parameter("depth_reverse_speed_m_s", 0.10)
        self.declare_parameter("depth_side_gain", 0.45)
        self.declare_parameter("depth_yaw_bias_deg", 14.0)
        self.declare_parameter("depth_turn_hysteresis_m", 0.35)
        self.declare_parameter("velocity_smoothing_alpha", 0.45)
        self.declare_parameter("velocity_altitude_hold", True)
        self.declare_parameter("velocity_altitude_p", 0.35)
        self.declare_parameter("velocity_max_down_speed_m_s", 0.30)

        self.next_waypoint: Optional[PoseStamped] = None
        self.direct_goal: Optional[PoseStamped] = None
        self.coverage_route: List[Tuple[float, float]] = []
        self.coverage_route_index = 0
        self.coverage_route_initialized = False
        self.last_waypoint_time = 0.0
        self.last_offboard_error_log_time = 0.0
        self.last_pose_publish_time = 0.0
        self.last_coverage_log_time = 0.0
        self.last_depth_process_time = 0.0
        self.last_depth_log_time = 0.0
        self.current_n = 0.0
        self.current_e = 0.0
        self.current_d = float(self.get_parameter("takeoff_down_m").value)
        self.current_yaw_deg = float(self.get_parameter("default_yaw_deg").value)
        self.depth_left_m = float(self.get_parameter("depth_max_range_m").value)
        self.depth_center_m = float(self.get_parameter("depth_max_range_m").value)
        self.depth_right_m = float(self.get_parameter("depth_max_range_m").value)
        self.depth_min_m = float(self.get_parameter("depth_max_range_m").value)
        self.last_depth_time = 0.0
        self.depth_turn_sign = 1.0
        self.filtered_north_m_s = 0.0
        self.filtered_east_m_s = 0.0
        self.running = True
        self.lock_path = str(self.get_parameter("instance_lock_path").value)
        self.offboard_start_time = 0.0

        if not self.acquire_instance_lock():
            raise RuntimeError("Another MAVSDK waypoint follower appears to be running.")

        self.local_pose_pub = self.create_publisher(PoseStamped, str(self.get_parameter("local_pose_topic").value), 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("next_waypoint_topic").value), self.waypoint_callback, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("direct_goal_topic").value), self.direct_goal_callback, 10)
        self.create_subscription(Image, str(self.get_parameter("depth_topic").value), self.depth_callback, 1)

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

    def depth_callback(self, msg: Image):
        if self.velocity_source() != "depth":
            return

        now = time.monotonic()
        period = 1.0 / max(0.2, float(self.get_parameter("depth_process_hz").value))
        if now - self.last_depth_process_time < period:
            return
        self.last_depth_process_time = now

        try:
            depth = image_msg_to_depth(msg)
            left_m, center_m, right_m, min_m = self.depth_regions(depth)
        except Exception as exc:
            if now - self.last_depth_log_time > 2.0:
                self.last_depth_log_time = now
                self.get_logger().warn(f"Depth steering conversion failed: {exc}")
            return

        self.depth_left_m = left_m
        self.depth_center_m = center_m
        self.depth_right_m = right_m
        self.depth_min_m = min_m
        self.last_depth_time = now

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
        if bool(self.get_parameter("set_mavsdk_stream_rates").value):
            await self.configure_mavsdk_stream_rates(drone)

        telemetry_task = asyncio.create_task(self.telemetry_loop(drone))

        takeoff_down = float(self.get_parameter("takeoff_down_m").value)
        if bool(self.get_parameter("arm_and_takeoff").value):
            ok = await self.arm_and_takeoff_if_needed(drone, takeoff_down)
            if not ok:
                telemetry_task.cancel()
                return

        velocity_mode = self.use_velocity_control()
        if velocity_mode:
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, 0.0, self.current_yaw_deg)
            )
        else:
            await drone.offboard.set_position_ned(
                PositionNedYaw(self.current_n, self.current_e, takeoff_down, self.current_yaw_deg)
            )
        try:
            await drone.offboard.start()
            self.get_logger().info("MAVSDK offboard started.")
            self.offboard_start_time = time.monotonic()
        except OffboardError as exc:
            self.get_logger().error(f"Offboard start failed: {exc}")
            return

        period = 1.0 / max(2.0, float(self.get_parameter("command_hz").value))
        while self.running:
            try:
                if velocity_mode:
                    await asyncio.wait_for(
                        drone.offboard.set_velocity_ned(self.choose_velocity_setpoint()),
                        timeout=0.8,
                    )
                else:
                    await asyncio.wait_for(
                        drone.offboard.set_position_ned(self.choose_setpoint(takeoff_down)),
                        timeout=0.8,
                    )
            except asyncio.TimeoutError:
                now = time.monotonic()
                if now - self.last_offboard_error_log_time > 2.0:
                    self.last_offboard_error_log_time = now
                    self.get_logger().warn("Offboard setpoint send timed out.")
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
            now = time.monotonic()
            pose_period = 1.0 / max(0.5, float(self.get_parameter("local_pose_publish_hz").value))
            if now - self.last_pose_publish_time < pose_period:
                if not self.running:
                    break
                continue
            self.last_pose_publish_time = now
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
            ("COM_DL_LOSS_T", 600, "int"),
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

    async def configure_mavsdk_stream_rates(self, drone):
        updates = [
            ("position_velocity_ned", float(self.get_parameter("mavsdk_position_rate_hz").value)),
            ("attitude_euler", float(self.get_parameter("mavsdk_attitude_rate_hz").value)),
            ("in_air", 1.0),
            ("armed", 1.0),
            ("health", 1.0),
        ]

        for stream_name, rate_hz in updates:
            setter = getattr(drone.telemetry, f"set_rate_{stream_name}", None)
            if setter is None:
                continue
            try:
                await setter(max(0.2, rate_hz))
            except Exception as exc:
                self.get_logger().debug(f"Could not set MAVSDK telemetry rate for {stream_name}: {exc}")

    async def arm_and_takeoff_if_needed(self, drone, takeoff_down: float) -> bool:
        in_air = await self.read_telemetry_once(drone.telemetry.in_air, False)
        armed = await self.read_telemetry_once(drone.telemetry.armed, False)

        if in_air:
            self.get_logger().warn("Vehicle already in air; skipping arm/takeoff.")
            return True

        await drone.action.set_takeoff_altitude(abs(takeoff_down))

        if not armed:
            try:
                await drone.action.arm()
            except Exception as exc:
                in_air_after_arm = await self.read_telemetry_once(drone.telemetry.in_air, False)
                if in_air_after_arm:
                    self.get_logger().warn(f"Arm command was denied, but vehicle is already airborne: {exc}")
                    return True
                self.get_logger().error(f"Arm failed: {exc}")
                return False

        try:
            await drone.action.takeoff()
        except Exception as exc:
            in_air_after_takeoff = await self.read_telemetry_once(drone.telemetry.in_air, False)
            if in_air_after_takeoff:
                self.get_logger().warn(f"Takeoff command was denied, but vehicle is already airborne: {exc}")
            else:
                self.get_logger().error(f"Takeoff failed: {exc}")
                return False

        await asyncio.sleep(7.0)
        return True

    @staticmethod
    async def read_telemetry_once(stream_factory, default=None, timeout_s: float = 2.0):
        stream = stream_factory()
        try:
            return await asyncio.wait_for(stream.__anext__(), timeout_s)
        except Exception:
            return default
        finally:
            try:
                await stream.aclose()
            except Exception:
                pass

    def choose_setpoint(self, takeoff_down: float):
        if self.next_waypoint is not None:
            max_age = float(self.get_parameter("direct_goal_fallback_s").value)
            if time.monotonic() - self.last_waypoint_time <= max_age:
                return self.pose_to_setpoint(self.next_waypoint, takeoff_down)

        if self.direct_goal is not None:
            return self.direct_goal_step_setpoint(self.direct_goal, takeoff_down)

        if bool(self.get_parameter("enable_follower_coverage").value):
            return self.follower_coverage_setpoint(takeoff_down)

        return PositionNedYaw(
            self.current_n,
            self.current_e,
            takeoff_down,
            self.current_yaw_deg,
        )

    def choose_velocity_setpoint(self):
        if self.velocity_source() == "depth":
            return self.depth_velocity_setpoint()

        if bool(self.get_parameter("enable_follower_coverage").value):
            return self.follower_velocity_pattern_setpoint()

        return self.hold_velocity_setpoint()

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

    def follower_coverage_setpoint(self, fallback_down: float):
        if not self.coverage_route_initialized:
            self.build_follower_coverage_route()

        if not self.coverage_route:
            return PositionNedYaw(
                self.current_n,
                self.current_e,
                fallback_down,
                self.current_yaw_deg,
            )

        reached_radius = max(0.2, float(self.get_parameter("follower_coverage_reached_radius_m").value))
        for _ in range(len(self.coverage_route)):
            goal_n, goal_e = self.coverage_route[self.coverage_route_index]
            if math.hypot(goal_n - self.current_n, goal_e - self.current_e) > reached_radius:
                break
            self.coverage_route_index = (self.coverage_route_index + 1) % len(self.coverage_route)

        goal_n, goal_e = self.coverage_route[self.coverage_route_index]
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

        now = time.monotonic()
        if now - self.last_coverage_log_time > 10.0:
            self.last_coverage_log_time = now
            self.get_logger().info(
                f"Follower coverage target {self.coverage_route_index + 1}/{len(self.coverage_route)} "
                f"goal=({goal_n:.1f},{goal_e:.1f}) step=({target_n:.1f},{target_e:.1f})"
            )

        return PositionNedYaw(target_n, target_e, fallback_down, yaw_deg)

    def follower_velocity_pattern_setpoint(self):
        speed = max(0.05, float(self.get_parameter("follower_velocity_speed_m_s").value))
        leg_s = max(1.0, float(self.get_parameter("follower_velocity_leg_s").value))
        pause_s = max(0.0, float(self.get_parameter("follower_velocity_pause_s").value))
        yaw_deg = float(self.get_parameter("follower_velocity_yaw_deg").value)

        # A tiny square pattern, deliberately time-based so it does not depend
        # on LOCAL_POSITION_NED being trustworthy during the smoke test.
        phases = [
            (speed, 0.0, leg_s, "north"),
            (0.0, 0.0, pause_s, "pause"),
            (0.0, speed, leg_s, "east"),
            (0.0, 0.0, pause_s, "pause"),
            (-speed, 0.0, leg_s, "south"),
            (0.0, 0.0, pause_s, "pause"),
            (0.0, -speed, leg_s, "west"),
            (0.0, 0.0, pause_s, "pause"),
        ]
        total_s = sum(duration for _vn, _ve, duration, _label in phases)
        if total_s <= 0.0:
            return VelocityNedYaw(0.0, 0.0, 0.0, yaw_deg)

        elapsed_s = (time.monotonic() - self.offboard_start_time) % total_s
        cursor_s = 0.0
        selected = phases[-1]
        for phase in phases:
            cursor_s += phase[2]
            if elapsed_s <= cursor_s:
                selected = phase
                break

        north_m_s, east_m_s, _duration_s, label = selected
        now = time.monotonic()
        if now - self.last_coverage_log_time > 10.0:
            self.last_coverage_log_time = now
            self.get_logger().info(
                f"Follower velocity pattern phase={label} "
                f"vel=({north_m_s:.2f},{east_m_s:.2f},0.00) "
                f"pos=({self.current_n:.1f},{self.current_e:.1f},{self.current_d:.1f})"
            )

        return VelocityNedYaw(north_m_s, east_m_s, self.altitude_down_velocity(), yaw_deg)

    def depth_velocity_setpoint(self):
        now = time.monotonic()
        stale_s = max(0.2, float(self.get_parameter("depth_stale_timeout_s").value))
        yaw_deg = self.current_yaw_deg

        if now - self.last_depth_time > stale_s:
            self.filtered_north_m_s *= 0.5
            self.filtered_east_m_s *= 0.5
            if now - self.last_depth_log_time > 2.0:
                self.last_depth_log_time = now
                self.get_logger().warn("Depth velocity source has no recent depth frame; holding.")
            return VelocityNedYaw(
                self.filtered_north_m_s,
                self.filtered_east_m_s,
                self.altitude_down_velocity(),
                yaw_deg,
            )

        max_forward = max(0.05, float(self.get_parameter("follower_velocity_speed_m_s").value))
        max_strafe = max(0.05, float(self.get_parameter("depth_strafe_speed_m_s").value))
        reverse_speed = max(0.0, float(self.get_parameter("depth_reverse_speed_m_s").value))
        side_gain = max(0.0, float(self.get_parameter("depth_side_gain").value))
        safe_m = max(0.5, float(self.get_parameter("depth_safe_distance_m").value))
        slow_m = max(safe_m + 0.1, float(self.get_parameter("depth_slow_distance_m").value))
        critical_m = max(0.25, float(self.get_parameter("depth_critical_distance_m").value))
        yaw_bias_deg = max(0.0, float(self.get_parameter("depth_yaw_bias_deg").value))
        turn_hysteresis_m = max(0.0, float(self.get_parameter("depth_turn_hysteresis_m").value))

        left_m = self.depth_left_m
        center_m = self.depth_center_m
        right_m = self.depth_right_m
        min_m = self.depth_min_m
        clearer_sign = self.depth_clearer_side_sign(left_m, right_m, turn_hysteresis_m)
        side_balance = self.clamp((right_m - left_m) / max(safe_m, 1e-6), -1.0, 1.0)

        if center_m <= critical_m or min_m <= critical_m * 0.75:
            forward_m_s = -reverse_speed
            right_m_s = clearer_sign * max_strafe
            yaw_target_deg = self.current_yaw_deg + clearer_sign * yaw_bias_deg * 1.5
            state = "critical"
        elif center_m < safe_m:
            forward_m_s = 0.0
            right_m_s = clearer_sign * max_strafe
            yaw_target_deg = self.current_yaw_deg + clearer_sign * yaw_bias_deg
            state = "blocked"
        else:
            openness = self.clamp((center_m - safe_m) / (slow_m - safe_m), 0.25, 1.0)
            forward_m_s = max_forward * openness
            right_m_s = side_gain * max_strafe * side_balance
            yaw_target_deg = self.current_yaw_deg
            state = "clear"

        target_north_m_s, target_east_m_s = self.body_velocity_to_ned(
            forward_m_s,
            right_m_s,
            self.current_yaw_deg,
        )
        target_north_m_s, target_east_m_s = self.smooth_horizontal_velocity(
            target_north_m_s,
            target_east_m_s,
        )

        if now - self.last_depth_log_time > 2.0:
            self.last_depth_log_time = now
            self.get_logger().info(
                f"Depth velocity {state} L/C/R/min="
                f"{left_m:.1f}/{center_m:.1f}/{right_m:.1f}/{min_m:.1f} "
                f"body=({forward_m_s:.2f},{right_m_s:.2f}) "
                f"ned=({target_north_m_s:.2f},{target_east_m_s:.2f}) "
                f"down_v={self.altitude_down_velocity():.2f} yaw={yaw_target_deg:.1f}"
            )

        return VelocityNedYaw(
            target_north_m_s,
            target_east_m_s,
            self.altitude_down_velocity(),
            yaw_target_deg,
        )

    def hold_velocity_setpoint(self):
        return VelocityNedYaw(
            0.0,
            0.0,
            self.altitude_down_velocity(),
            float(self.get_parameter("follower_velocity_yaw_deg").value),
        )

    def depth_clearer_side_sign(self, left_m: float, right_m: float, hysteresis_m: float) -> float:
        diff_m = right_m - left_m
        if abs(diff_m) >= hysteresis_m:
            self.depth_turn_sign = 1.0 if diff_m >= 0.0 else -1.0
        return self.depth_turn_sign

    def depth_regions(self, depth):
        import numpy as np

        max_range = float(self.get_parameter("depth_max_range_m").value)
        min_range = float(self.get_parameter("depth_min_range_m").value)
        clean = depth.astype(np.float32, copy=False)
        clean = clean.copy()
        clean[(~np.isfinite(clean)) | (clean > max_range)] = max_range
        clean[clean < min_range] = max_range

        h, w = clean.shape[:2]
        if h <= 0 or w <= 0:
            return max_range, max_range, max_range, max_range

        y1 = int(h * 0.30)
        y2 = int(h * 0.68)
        band = clean[max(0, y1):max(y1 + 1, y2), :]
        third = max(1, w // 3)

        left = self.region_depth_percentile(band[:, :third], max_range)
        center = self.region_depth_percentile(band[:, third:2 * third], max_range)
        right = self.region_depth_percentile(band[:, 2 * third:], max_range)
        min_depth = float(np.min(band)) if band.size else max_range
        return left, center, right, min_depth

    @staticmethod
    def region_depth_percentile(region, fallback: float) -> float:
        import numpy as np

        if region.size == 0:
            return fallback
        valid = region[np.isfinite(region)]
        if valid.size < max(5, int(region.size * 0.02)):
            return fallback
        return float(np.percentile(valid, 20))

    @staticmethod
    def body_velocity_to_ned(forward_m_s: float, right_m_s: float, yaw_deg: float):
        yaw_rad = math.radians(yaw_deg)
        north_m_s = forward_m_s * math.cos(yaw_rad) - right_m_s * math.sin(yaw_rad)
        east_m_s = forward_m_s * math.sin(yaw_rad) + right_m_s * math.cos(yaw_rad)
        return north_m_s, east_m_s

    def smooth_horizontal_velocity(self, north_m_s: float, east_m_s: float):
        alpha = self.clamp(float(self.get_parameter("velocity_smoothing_alpha").value), 0.05, 1.0)
        self.filtered_north_m_s = alpha * north_m_s + (1.0 - alpha) * self.filtered_north_m_s
        self.filtered_east_m_s = alpha * east_m_s + (1.0 - alpha) * self.filtered_east_m_s
        return self.filtered_north_m_s, self.filtered_east_m_s

    def altitude_down_velocity(self) -> float:
        if not bool(self.get_parameter("velocity_altitude_hold").value):
            return 0.0

        target_down = float(self.get_parameter("takeoff_down_m").value)
        gain = max(0.0, float(self.get_parameter("velocity_altitude_p").value))
        max_down_speed = max(0.05, float(self.get_parameter("velocity_max_down_speed_m_s").value))
        return self.clamp((target_down - self.current_d) * gain, -max_down_speed, max_down_speed)

    @staticmethod
    def clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def build_follower_coverage_route(self):
        half = max(1.0, float(self.get_parameter("follower_coverage_half_extent_m").value))
        spacing = max(0.8, float(self.get_parameter("follower_coverage_lane_spacing_m").value))

        coords = []
        value = -half
        while value <= half + 1e-6:
            coords.append(round(value, 2))
            value += spacing
        if not coords or coords[-1] < half:
            coords.append(round(half, 2))

        route = []
        for i, north in enumerate(coords):
            east_order = coords if i % 2 == 0 else list(reversed(coords))
            for east in east_order:
                route.append((north, east))

        if route:
            start_index = min(
                range(len(route)),
                key=lambda idx: math.hypot(route[idx][0] - self.current_n, route[idx][1] - self.current_e),
            )
            route = route[start_index:] + route[:start_index]

        self.coverage_route = route
        self.coverage_route_index = 0
        self.coverage_route_initialized = True
        self.get_logger().info(
            f"Follower internal coverage route initialized with {len(route)} goals "
            f"half_extent={half:.1f}m spacing={spacing:.1f}m."
        )

    def use_velocity_control(self) -> bool:
        return str(self.get_parameter("offboard_control_mode").value).strip().lower() == "velocity"

    def velocity_source(self) -> str:
        return str(self.get_parameter("velocity_source").value).strip().lower()

    def destroy_node(self):
        self.running = False
        self.release_instance_lock()
        return super().destroy_node()

    def acquire_instance_lock(self) -> bool:
        while True:
            try:
                fd = os.open(self.lock_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
                with os.fdopen(fd, "w") as handle:
                    handle.write(str(os.getpid()))
                return True
            except FileExistsError:
                pid = self.read_lock_pid()
                if pid is not None and self.pid_is_running(pid):
                    self.get_logger().error(
                        f"Refusing to start: MAVSDK follower lock exists at {self.lock_path} "
                        f"for running PID {pid}. Stop the previous launch or restart PX4 first."
                    )
                    return False
                try:
                    os.unlink(self.lock_path)
                except FileNotFoundError:
                    continue
                except Exception as exc:
                    self.get_logger().error(f"Could not remove stale lock {self.lock_path}: {exc}")
                    return False

    def read_lock_pid(self):
        try:
            with open(self.lock_path, "r", encoding="utf-8") as handle:
                return int(handle.read().strip())
        except Exception:
            return None

    @staticmethod
    def pid_is_running(pid: int) -> bool:
        try:
            os.kill(pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    def release_instance_lock(self):
        pid = self.read_lock_pid()
        if pid != os.getpid():
            return
        try:
            os.unlink(self.lock_path)
        except FileNotFoundError:
            pass
        except Exception as exc:
            self.get_logger().warn(f"Could not remove lock {self.lock_path}: {exc}")


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
