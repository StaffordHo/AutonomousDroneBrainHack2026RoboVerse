import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from .common import make_pose, quaternion_to_yaw

try:
    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
except Exception:  # pragma: no cover - depends on ROS2 workspace setup
    OffboardControlMode = None
    TrajectorySetpoint = None
    VehicleCommand = None
    VehicleLocalPosition = None
    VehicleStatus = None


class PX4OffboardNode(Node):
    """
    Pure ROS2 PX4 offboard follower.

    This node requires `px4_msgs` and uXRCE-DDS `/fmu/*` topics. On this machine
    px4_msgs was not importable when generated, so the MAVSDK follower is the
    immediate fallback until px4_msgs is installed into the ROS2 workspace.
    """

    def __init__(self):
        super().__init__("px4_offboard_node")

        self.declare_parameter("next_waypoint_topic", "/roboverse/next_waypoint")
        self.declare_parameter("local_pose_topic", "/roboverse/local_pose")
        self.declare_parameter("map_frame", "px4_ned")
        self.declare_parameter("arm_and_offboard", True)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("takeoff_down_m", -1.5)

        self.current_pose: Optional[PoseStamped] = None
        self.next_waypoint: Optional[PoseStamped] = None
        self.vehicle_status = None
        self.offboard_counter = 0

        self.local_pose_pub = self.create_publisher(PoseStamped, str(self.get_parameter("local_pose_topic").value), 10)

        if OffboardControlMode is None:
            self.get_logger().error(
                "px4_msgs is not importable. Install px4_msgs and source the ROS2 workspace, "
                "or run mavsdk_waypoint_follower_node instead."
            )
            return

        self.offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.local_position_callback, 10)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.status_callback, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("next_waypoint_topic").value), self.waypoint_callback, 10)

        self.create_timer(1.0 / max(2.0, float(self.get_parameter("publish_hz").value)), self.control_tick)

    def timestamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def local_position_callback(self, msg):
        yaw = float(getattr(msg, "heading", 0.0))
        pose = make_pose(
            str(self.get_parameter("map_frame").value),
            self.get_clock().now().to_msg(),
            float(msg.x),
            float(msg.y),
            float(msg.z),
            yaw,
        )
        self.current_pose = pose
        self.local_pose_pub.publish(pose)

    def status_callback(self, msg):
        self.vehicle_status = msg

    def waypoint_callback(self, msg: PoseStamped):
        self.next_waypoint = msg

    def publish_vehicle_command(self, command: int, **params):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()
        msg.command = command
        msg.param1 = float(params.get("param1", 0.0))
        msg.param2 = float(params.get("param2", 0.0))
        msg.param3 = float(params.get("param3", 0.0))
        msg.param4 = float(params.get("param4", 0.0))
        msg.param5 = float(params.get("param5", 0.0))
        msg.param6 = float(params.get("param6", 0.0))
        msg.param7 = float(params.get("param7", 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def engage_offboard(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def control_tick(self):
        if OffboardControlMode is None:
            return

        control = OffboardControlMode()
        control.timestamp = self.timestamp_us()
        control.position = True
        control.velocity = False
        control.acceleration = False
        control.attitude = False
        control.body_rate = False
        self.offboard_pub.publish(control)

        target = self.next_waypoint
        if target is None and self.current_pose is not None:
            target = self.current_pose

        if target is None:
            return

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = self.timestamp_us()
        setpoint.position = [
            float(target.pose.position.x),
            float(target.pose.position.y),
            float(target.pose.position.z),
        ]
        setpoint.yaw = quaternion_to_yaw(target.pose.orientation)
        self.setpoint_pub.publish(setpoint)

        if bool(self.get_parameter("arm_and_offboard").value):
            self.offboard_counter += 1
            if self.offboard_counter == 20:
                self.engage_offboard()
                self.arm()


def main(args=None):
    rclpy.init(args=args)
    node = PX4OffboardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
