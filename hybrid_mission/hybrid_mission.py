import asyncio
import sys
import os
# Add parent directory to sys.path to resolve imports of supporting modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import json
import math
import subprocess
import time
import warnings
from collections import deque
from datetime import datetime

import grpc
import cv2
import numpy as np

import contextlib
from mission_logger import MissionLogger

os.environ.setdefault("YOLO_CONFIG_DIR", "/tmp")

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import (
    OffboardError,
    PositionNedYaw,
    VelocityBodyYawspeed,
    VelocityNedYaw,
)
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

from obstacle_monitor import ObstacleMonitor
from small_fuel_detector import barrel_label_for_colour, detect_small_fuel_barrels
from target_memory import TargetMemory, normalize_angle_deg
from exploration_memory import ExplorationMemory
from gzphotodetectorsaver import GZPhotoDetectorSaver, load_yolo_class
from ros2_sensor_bridge import ROS2_AVAILABLE, Ros2SensorBridge


# ============================================================
# Qualifier mission configuration
# ============================================================

MISSION_TIME_LIMIT_S = 9 * 60
LANDING_BUFFER_S = 18
EXTRA_SCORING_AFTER_ELIGIBILITY = (
    os.getenv("EXTRA_SCORING_AFTER_ELIGIBILITY", "0") == "1"
)

# NED down: negative = up
LOW_SCAN_ALT_D = -1.4
HIGH_SCAN_ALT_D = -2.4
DEFAULT_ALT_D = LOW_SCAN_ALT_D

# Movement
MOVE_STEP_M = float(os.getenv("MOVE_STEP_M", "0.30"))
FAST_OPEN_STEP_M = float(os.getenv("FAST_OPEN_STEP_M", "0.40"))
FAST_OPEN_MIN_FRONT_M = float(os.getenv("FAST_OPEN_MIN_FRONT_M", "5.0"))
FAST_OPEN_MIN_SIDE_M = float(os.getenv("FAST_OPEN_MIN_SIDE_M", "2.2"))
FAST_OPEN_MIN_LOWER_M = float(os.getenv("FAST_OPEN_MIN_LOWER_M", "1.35"))
RETURN_STEP_M = float(os.getenv("RETURN_STEP_M", "0.45"))
MOVE_TIMEOUT_S = 1.2
MOVE_REACHED_RADIUS_M = 0.10
CORRIDOR_MOVE_TIMEOUT_S = 0.75

# Local exploration with return-home guard. Keep inside the arena margin.
SOFT_RANGE_LIMIT_M = float(os.getenv("SOFT_RANGE_LIMIT_M", "19.0"))
HARD_RANGE_LIMIT_M = float(os.getenv("HARD_RANGE_LIMIT_M", "23.0"))
RESUME_RANGE_M = float(os.getenv("RESUME_RANGE_M", "15.2"))

# Exploration headings relative to start yaw
LOW_PASS_HEADINGS_DEG = [0, 45, -45, 90, -90, 135, -135, 180]
HIGH_PASS_HEADINGS_DEG = [22.5, 67.5, -22.5, -67.5, 112.5, -112.5, 157.5, -157.5]
SWEEP_HEADINGS_DEG = [0, 60, 120, 180, -120, -60, 30, 90, 150, -150, -90, -30]

LOW_PASS_MAX_STEPS_PER_HEADING = 8
HIGH_PASS_MAX_STEPS_PER_HEADING = 7
SWEEP_STEPS_PER_HEADING = 5
MAX_FULL_SWEEP_CYCLES = 3
SWEEP_HEADING_ROTATION_DEG = 20.0
GLOBAL_COVERAGE_RINGS_M = [4.0, 8.0, 12.0, 16.0]
GLOBAL_COVERAGE_HEADINGS_DEG = [0, 45, -45, 90, -90, 135, -135, 180]
GLOBAL_GOAL_REACHED_RADIUS_M = 1.6
MAX_GLOBAL_GOAL_STEPS = 14
USE_FIXED_RING_COVERAGE = os.getenv("USE_FIXED_RING_COVERAGE", "0") == "1"
CONTINUE_FRONTIER_AFTER_ELIGIBILITY = (
    os.getenv("CONTINUE_FRONTIER_AFTER_ELIGIBILITY", "1") == "1"
)
FRONTIER_LOW_STRIDES = 10
FRONTIER_HIGH_STRIDES = 8
FRONTIER_STEPS_PER_STRIDE = 9
FRONTIER_SEARCH_STRIDES = 5
FRONTIER_SEARCH_STEPS_PER_STRIDE = 7
FRONTIER_ESCAPE_STEPS = 5
FRONTIER_MACRO_HEADINGS_DEG = [0, 45, -45, 90, -90, 135, -135, 180]
FRONTIER_POST_ELIGIBILITY_STRIDES = int(os.getenv("FRONTIER_POST_ELIGIBILITY_STRIDES", "6"))
FRONTIER_SECTOR_RESET_RANGE_M = float(os.getenv("FRONTIER_SECTOR_RESET_RANGE_M", "16.2"))
FRONTIER_SECTOR_RESUME_RANGE_M = float(os.getenv("FRONTIER_SECTOR_RESUME_RANGE_M", "15.2"))
FRONTIER_RECENTER_STEPS = 7
FRONTIER_MIN_DIRECTION_CLEARANCE_M = 2.05
FRONTIER_TIGHT_TURN_SIDE_CLEARANCE_M = 1.30
BLIND_TIGHT_TURN_SIDE_CLEARANCE_M = 1.50
FRONTIER_DELIBERATION_ENABLED = os.getenv("FRONTIER_DELIBERATION_ENABLED", "1") == "1"
FRONTIER_DELIBERATION_LOG = os.getenv("FRONTIER_DELIBERATION_LOG", "0") == "1"
FRONTIER_LOOKAHEAD_STEPS = int(os.getenv("FRONTIER_LOOKAHEAD_STEPS", "5"))
FRONTIER_LOOKAHEAD_STEP_M = float(os.getenv("FRONTIER_LOOKAHEAD_STEP_M", "1.3"))
FRONTIER_BEAM_TOP_K = int(os.getenv("FRONTIER_BEAM_TOP_K", "3"))
DEEP_EXPLORATION_AFTER_ELIGIBILITY = (
    os.getenv("DEEP_EXPLORATION_AFTER_ELIGIBILITY", "1") == "1"
)
DEEP_EXPLORATION_TARGET_RANGE_M = float(os.getenv("DEEP_EXPLORATION_TARGET_RANGE_M", "18.2"))
DEEP_EXPLORATION_RANGE_BONUS = float(os.getenv("DEEP_EXPLORATION_RANGE_BONUS", "3.6"))
DEEP_EXPLORATION_HOME_BIAS_SCALE = float(os.getenv("DEEP_EXPLORATION_HOME_BIAS_SCALE", "0.25"))
CANDIDATE_REVISIT_STANDOFF_M = 2.2
MAX_CANDIDATE_REVISITS = 6
MAX_REVISIT_GOAL_STEPS = 8

# Sensors / perception
DEPTH_TOPIC = "/depth_camera"
IMAGE_TOPIC_FALLBACK = "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image"
USE_ROS2_SENSOR_BRIDGE = os.getenv("USE_ROS2_SENSOR_BRIDGE", "0") == "1"
ROS2_IMAGE_TOPIC = os.getenv("ROS2_IMAGE_TOPIC", IMAGE_TOPIC_FALLBACK)
ROS2_DEPTH_TOPIC = os.getenv("ROS2_DEPTH_TOPIC", DEPTH_TOPIC)

DETECTION_CONFIDENCE_THRESHOLD = 0.52
IMX214_HFOV_DEG = 69.0
EVIDENCE_DIR = "competition_evidence"
PHOTO_BURST_DIR = "competition_photos"

# Continuous perception frequency. Larger values lower CPU at the cost of slower
# visual candidate pickup.
PERCEPTION_PERIOD_S = float(os.getenv("PERCEPTION_PERIOD_S", "0.20"))
STOP_CAPTURE_BURST_FRAMES = 1
CONFIRM_CAPTURE_BURST_FRAMES = 6
MIN_PHOTO_BURST_INTERVAL_S = 2.0
YOLO_MODEL_PATH = os.getenv("YOLO_MODEL_PATH", "Codes/yolov10n.pt")
YOLO_BURST_ENABLED = os.getenv("YOLO_BURST_ENABLED", "0") == "1"
YOLO_BURST_FRAMES = int(os.getenv("YOLO_BURST_FRAMES", "2"))
YOLO_IMGSZ = int(os.getenv("YOLO_IMGSZ", "416"))
YOLO_DEVICE = os.getenv("YOLO_DEVICE", "cpu")
MIN_YOLO_BURST_INTERVAL_S = float(os.getenv("MIN_YOLO_BURST_INTERVAL_S", "12.0"))

# Mission YOLO feeds the target memory/path-finder loop directly. It is
# throttled separately from HSV detection to keep CPU load predictable.
DEFAULT_MISSION_YOLO_MODEL_PATH = next(
    (
        path
        for path in (
            "Codes/8s_2.0.pt",
            "Codes/yolov8s_roboverse.pt",
            "Codes/yolov8n_roboverse.pt",
            "Codes/yolov8n.pt",
            "Codes/best.pt",
        )
        if os.path.exists(path)
    ),
    "Codes/yolov8n_roboverse.pt",
)
MISSION_YOLO_ENABLED = os.getenv("MISSION_YOLO_ENABLED", "1") == "1"
MISSION_YOLO_MODEL_PATH = os.getenv("MISSION_YOLO_MODEL_PATH", DEFAULT_MISSION_YOLO_MODEL_PATH)
MISSION_YOLO_CONFIDENCE_THRESHOLD = float(
    os.getenv("MISSION_YOLO_CONFIDENCE_THRESHOLD", str(DETECTION_CONFIDENCE_THRESHOLD))
)
MISSION_YOLO_IMGSZ = int(os.getenv("MISSION_YOLO_IMGSZ", str(YOLO_IMGSZ)))
MISSION_YOLO_DEVICE = os.getenv("MISSION_YOLO_DEVICE", YOLO_DEVICE)
if YOLO_DEVICE.lower() == "cpu" and MISSION_YOLO_DEVICE.lower() == "cpu":
    os.environ.setdefault("CUDA_VISIBLE_DEVICES", "-1")
    warnings.filterwarnings(
        "ignore",
        message="CUDA initialization: The NVIDIA driver on your system is too old.*",
        category=UserWarning,
    )
MISSION_YOLO_PERIOD_S = float(os.getenv("MISSION_YOLO_PERIOD_S", "0.60"))
MISSION_YOLO_MAX_DETECTIONS = int(os.getenv("MISSION_YOLO_MAX_DETECTIONS", "8"))
MOVE_STOP_SCAN_INTERVAL_STEPS = 2
MOVE_STOP_SCAN_FRAMES = 1

# Optional manual override channel. A separate bridge can publish JSON UDP
# commands from a joystick/controller while autonomy pauses at motion gates.
TELEOP_ENABLED = os.getenv("TELEOP_ENABLED", "0") == "1"
TELEOP_UDP_HOST = os.getenv("TELEOP_UDP_HOST", "127.0.0.1")
TELEOP_UDP_PORT = int(os.getenv("TELEOP_UDP_PORT", "14591"))
TELEOP_TIMEOUT_S = 0.35
TELEOP_SEND_PERIOD_S = 0.05
TELEOP_MAX_FORWARD_M_S = float(os.getenv("TELEOP_MAX_FORWARD_M_S", "0.8"))
TELEOP_MAX_RIGHT_M_S = float(os.getenv("TELEOP_MAX_RIGHT_M_S", "0.8"))
TELEOP_MAX_DOWN_M_S = float(os.getenv("TELEOP_MAX_DOWN_M_S", "0.45"))
TELEOP_MAX_YAW_RATE_DEG_S = float(os.getenv("TELEOP_MAX_YAW_RATE_DEG_S", "55.0"))
TELEOP_DEADBAND = 0.06

# Investigation
INVESTIGATION_YAW_OFFSETS_DEG = [-16, -8, 0, 8, 16]
INVESTIGATION_SETTLE_S = 0.45
INVESTIGATE_COOLDOWN_S = 3.0
SCAN_FRAMES_PER_VIEW = 3
SCAN_FRAME_INTERVAL_S = 0.12
YAW_SETTLE_TOLERANCE_DEG = 8.0
YAW_SETTLE_TIMEOUT_S = 1.4

# Obstacle / safety
MAX_ATTITUDE_DEG = 8.0
MID_STEP_MAX_ATTITUDE_DEG = 12.0
CRITICAL_ATTITUDE_DEG = 25.0
RECOVERY_ATTITUDE_TOLERANCE_DEG = 15.0
RECOVERY_HARD_LIMIT_DEG = 35.0
MIN_FRONT_MOVE_CLEARANCE_M = 0.90
MIN_SIDE_MOVE_CLEARANCE_M = 0.45
MIN_LOWER_MOVE_CLEARANCE_M = 0.45
MID_STEP_ABORT_CLEARANCE_M = 0.80
YAW_MIN_FRONT_CLEARANCE_M = 0.80
YAW_MIN_SIDE_CLEARANCE_M = 0.40
NARROW_CORRIDOR_ENABLED = os.getenv("NARROW_CORRIDOR_ENABLED", "1") == "1"
NARROW_CORRIDOR_MIN_FRONT_M = 0.90
NARROW_CORRIDOR_MIN_SIDE_M = 0.40
NARROW_CORRIDOR_STEP_M = 0.20
NARROW_CORRIDOR_STEER_GAIN_DEG_PER_M = 7.0
NARROW_CORRIDOR_STEER_I_GAIN_DEG_PER_M_S = 0.35
NARROW_CORRIDOR_STEER_D_GAIN_DEG_S_PER_M = 0.9
NARROW_CORRIDOR_PID_INTEGRAL_LIMIT = 3.0
NARROW_CORRIDOR_MAX_STEER_DEG = 12.0
NARROW_CORRIDOR_FRONTIER_BONUS = 8.0

ALT_TOLERANCE_M = 1.0
CRITICAL_ALTITUDE_DEVIATION_M = 3.5

MAX_BLOCKED_STREAK = 4
MAX_RETURN_FAIL_STREAK = 6
MAX_YAW_STEP_DEG = 25.0
SAFE_POSE_HISTORY_MAX = 18
MIN_RECOVERY_RETREAT_M = 0.75
MAX_FAILED_STEPS_PER_HEADING = 2
MAX_FAILED_STEPS_PER_GOAL = 3
MAX_FAILED_HEADINGS_PER_PASS = 4

# Score
YELLOW_SCORE = 50
RED_SCORE = 100


# ============================================================
# Shared runtime state
# ============================================================

latest_frame = None
latest_position_ned = None

latest_attitude = {
    "pitch": 0.0,
    "roll": 0.0,
    "yaw": 0.0,
}

monitor = ObstacleMonitor(
    obstacle_distance_m=0.90,
    warning_distance_m=1.50,
    critical_lower_distance_m=MIN_LOWER_MOVE_CLEARANCE_M,
)

target_memory = TargetMemory(
    min_confidence=DETECTION_CONFIDENCE_THRESHOLD,
    min_depth_m=0.35,
    max_depth_m=9.0,
    min_confirm_count=3,
    min_confirm_age_s=0.5,
    min_yaw_span_deg=5.0,
    strong_confirm_count=7,
    strong_confirm_confidence=0.78,
    strong_confirm_age_s=0.9,
    strong_confirm_position_spread_m=2.6,
    strong_confirm_depth_spread_m=3.2,
    duplicate_distance_m=1.8,
    duplicate_bearing_deg=12.0,
    stale_candidate_s=MISSION_TIME_LIMIT_S,
)

exploration_memory = ExplorationMemory(
    cell_size_m=float(os.getenv("EXPLORATION_CELL_SIZE_M", "0.8")),
    lookahead_m=float(os.getenv("EXPLORATION_LOOKAHEAD_M", "6.0")),
    novelty_weight=float(os.getenv("EXPLORATION_NOVELTY_WEIGHT", "6.0")),
    revisit_penalty=float(os.getenv("EXPLORATION_REVISIT_PENALTY", "2.6")),
    blocked_penalty=90.0,
    turn_penalty=0.03,
    preferred_heading_penalty=0.012,
)

mission_logger = MissionLogger(log_dir="bc_logs")
camera_photo_saver = None

mission_start_time = None
start_n = 0.0
start_e = 0.0
start_yaw = 0.0

blocked_streak = 0
return_fail_streak = 0
last_safe_position = None
safe_position_history = deque(maxlen=SAFE_POSE_HISTORY_MAX)
corridor_pid_integral = 0.0
corridor_pid_previous_error = 0.0
corridor_pid_last_time = None

active_allowed_colours = ("yellow",)
active_target_alt_d = LOW_SCAN_ALT_D
active_investigation_enabled = True

new_candidate_event = asyncio.Event()
confirmed_event = asyncio.Event()

last_investigation_time = 0.0
mission_should_stop = False
frame_processing_lock = asyncio.Lock()
last_photo_burst_time = 0.0
last_yolo_burst_time = 0.0
mission_yolo_model = None
mission_yolo_load_attempted = False
mission_yolo_last_run_time = 0.0
teleop_last_update = 0.0
teleop_command = {
    "enabled": False,
    "forward": 0.0,
    "right": 0.0,
    "down": 0.0,
    "yaw_rate": 0.0,
}


# ============================================================
# Gazebo callbacks
# ============================================================

def update_latest_frame_bgr(frame_bgr):
    global latest_frame
    latest_frame = frame_bgr


def update_latest_depth(depth_m):
    monitor.update_depth(depth_m)


def image_callback(msg: Image):
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    update_latest_frame_bgr(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))


def depth_callback(msg: Image):
    depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    update_latest_depth(depth)


# ============================================================
# Optional teleop override
# ============================================================

def apply_deadband(value, deadband=TELEOP_DEADBAND):
    value = float(value)
    if abs(value) < deadband:
        return 0.0
    return value


def update_teleop_command(payload):
    global teleop_last_update

    enabled = bool(payload.get("enabled", payload.get("manual", True)))

    teleop_command["enabled"] = enabled
    teleop_command["forward"] = clamp(
        apply_deadband(payload.get("forward", payload.get("pitch", 0.0))),
        -TELEOP_MAX_FORWARD_M_S,
        TELEOP_MAX_FORWARD_M_S,
    )
    teleop_command["right"] = clamp(
        apply_deadband(payload.get("right", payload.get("roll", 0.0))),
        -TELEOP_MAX_RIGHT_M_S,
        TELEOP_MAX_RIGHT_M_S,
    )
    teleop_command["down"] = clamp(
        apply_deadband(payload.get("down", 0.0)),
        -TELEOP_MAX_DOWN_M_S,
        TELEOP_MAX_DOWN_M_S,
    )
    teleop_command["yaw_rate"] = clamp(
        apply_deadband(payload.get("yaw_rate", payload.get("yaw", 0.0))),
        -TELEOP_MAX_YAW_RATE_DEG_S,
        TELEOP_MAX_YAW_RATE_DEG_S,
    )
    teleop_last_update = time.time()


class TeleopUdpProtocol(asyncio.DatagramProtocol):
    def datagram_received(self, data, addr):
        try:
            payload = json.loads(data.decode("utf-8"))
            update_teleop_command(payload)
        except Exception as error:
            print(f"⚠️ Ignoring malformed teleop packet from {addr}: {error}")


def teleop_active():
    if not TELEOP_ENABLED or not teleop_command["enabled"]:
        return False

    return time.time() - teleop_last_update <= TELEOP_TIMEOUT_S


async def start_teleop_udp_listener():
    if not TELEOP_ENABLED:
        return None, None

    loop = asyncio.get_running_loop()
    transport, protocol = await loop.create_datagram_endpoint(
        lambda: TeleopUdpProtocol(),
        local_addr=(TELEOP_UDP_HOST, TELEOP_UDP_PORT),
    )
    print(f"🎮 Teleop UDP listener active on {TELEOP_UDP_HOST}:{TELEOP_UDP_PORT}")
    return transport, protocol


async def teleop_override_task(drone):
    was_active = False

    while not mission_should_stop:
        active = teleop_active()

        try:
            if active:
                if not was_active:
                    print("🎮 Manual teleop override active; autonomy is pausing motion commands.")
                    log_mission_action(
                        action_type="TELEOP_ACTIVE",
                        label="teleop",
                        target_down=active_target_alt_d,
                        extra={"source": "udp"},
                    )

                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(
                        teleop_command["forward"],
                        teleop_command["right"],
                        teleop_command["down"],
                        teleop_command["yaw_rate"],
                    )
                )

            elif was_active:
                print("🎮 Manual teleop override released; autonomy resuming.")
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
                )
        except Exception as error:
            print(f"⚠️ Teleop command rejected: {error}")

        was_active = active
        await asyncio.sleep(TELEOP_SEND_PERIOD_S)


async def wait_for_teleop_release(drone, target_down):
    if not TELEOP_ENABLED:
        return

    waited = False

    while teleop_active():
        waited = True

        if target_down is not None and critical_vehicle_state(target_down):
            raise RuntimeError("critical_state")

        await asyncio.sleep(TELEOP_SEND_PERIOD_S)

    if waited and latest_position_ned is not None:
        await drone.offboard.set_position_ned(
            PositionNedYaw(
                latest_position_ned.north_m,
                latest_position_ned.east_m,
                target_down if target_down is not None else latest_position_ned.down_m,
                latest_attitude["yaw"],
            )
        )


# ============================================================
# Helper functions
# ============================================================

def elapsed_s():
    if mission_start_time is None:
        return 0.0

    return time.time() - mission_start_time


def timed_out():
    return elapsed_s() > MISSION_TIME_LIMIT_S


def search_time_remaining():
    return elapsed_s() < (MISSION_TIME_LIMIT_S - LANDING_BUFFER_S)


def get_score(summary):
    return summary["red"] * RED_SCORE + summary["yellow"] * YELLOW_SCORE


def eligibility_met(summary):
    return summary["red"] >= 1 and summary["yellow"] >= 1


def deep_exploration_active():
    return (
        DEEP_EXPLORATION_AFTER_ELIGIBILITY
        and CONTINUE_FRONTIER_AFTER_ELIGIBILITY
        and eligibility_met(target_memory.summary())
    )


def missing_colours(summary):
    missing = []

    if summary["yellow"] == 0:
        missing.append("yellow")

    if summary["red"] == 0:
        missing.append("red")

    return missing


def distance_from_start():
    if latest_position_ned is None:
        return 0.0

    return math.hypot(
        latest_position_ned.north_m - start_n,
        latest_position_ned.east_m - start_e,
    )


def heading_to_start_deg():
    if latest_position_ned is None:
        return latest_attitude["yaw"]

    return normalize_angle_deg(
        math.degrees(
            math.atan2(
                start_e - latest_position_ned.east_m,
                start_n - latest_position_ned.north_m,
            )
        )
    )


def heading_to_point_deg(target_n, target_e):
    if latest_position_ned is None:
        return latest_attitude["yaw"]

    return normalize_angle_deg(
        math.degrees(
            math.atan2(
                target_e - latest_position_ned.east_m,
                target_n - latest_position_ned.north_m,
            )
        )
    )


def distance_to_point_m(target_n, target_e):
    if latest_position_ned is None:
        return 0.0

    return math.hypot(
        latest_position_ned.north_m - target_n,
        latest_position_ned.east_m - target_e,
    )


def angle_diff_deg(a, b):
    return abs(normalize_angle_deg(a - b))


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def altitude_error_m(target_down):
    if latest_position_ned is None:
        return 0.0

    return abs(latest_position_ned.down_m - target_down)


def find_image_topic():
    try:
        topics = subprocess.check_output(["gz", "topic", "-l"], timeout=3).decode().split()

        for topic in topics:
            if "x500_vision_0" in topic and "IMX214/image" in topic:
                return topic

        for topic in topics:
            if "IMX214/image" in topic:
                return topic

    except Exception:
        pass

    return IMAGE_TOPIC_FALLBACK


def mark_current_cell_visited():
    if latest_position_ned is None:
        return

    exploration_memory.mark_visited(
        latest_position_ned.north_m,
        latest_position_ned.east_m,
    )


def mark_path_segment_visited(start_n_value, start_e_value, end_n_value, end_e_value):
    distance = math.hypot(end_n_value - start_n_value, end_e_value - start_e_value)
    if distance <= 0.01:
        return

    spacing = max(0.20, exploration_memory.cell_size_m * 0.45)
    steps = max(1, int(math.ceil(distance / spacing)))

    for index in range(steps + 1):
        t = index / steps
        exploration_memory.mark_visited(
            start_n_value + (end_n_value - start_n_value) * t,
            start_e_value + (end_e_value - start_e_value) * t,
        )


def remember_safe_position():
    global last_safe_position

    if latest_position_ned is None:
        return

    c = monitor.get_directional_clearance()
    safe, _ = clearance_safe_for_motion(c, allow_corridor=NARROW_CORRIDOR_ENABLED)

    if not safe or not vehicle_stable(active_target_alt_d, attitude_limit_deg=15.0):
        return

    pose = (
        latest_position_ned.north_m,
        latest_position_ned.east_m,
        latest_position_ned.down_m,
        latest_attitude["yaw"],
    )

    last_safe_position = pose

    if not safe_position_history:
        safe_position_history.append(pose)
        return

    prev_n, prev_e, _, prev_yaw = safe_position_history[-1]

    if (
        math.hypot(pose[0] - prev_n, pose[1] - prev_e) >= 0.20
        or abs(angle_diff_deg(pose[3], prev_yaw)) >= 20.0
    ):
        safe_position_history.append(pose)


def select_recovery_pose(prefer_latest=False):
    if last_safe_position is None:
        return None, "none"

    if prefer_latest:
        return last_safe_position, "latest"

    if latest_position_ned is None or not safe_position_history:
        return last_safe_position, "latest"

    current_n = latest_position_ned.north_m
    current_e = latest_position_ned.east_m

    for pose in reversed(safe_position_history):
        pose_n, pose_e, _, _ = pose

        if math.hypot(current_n - pose_n, current_e - pose_e) >= MIN_RECOVERY_RETREAT_M:
            return pose, "older"

    if len(safe_position_history) >= 3:
        return safe_position_history[0], "oldest"

    return last_safe_position, "latest"


async def recover_to_last_safe(drone, target_down, reason, prefer_latest=False):
    global blocked_streak

    blocked_streak += 1

    await wait_for_teleop_release(drone, target_down)

    recovery_pose, recovery_kind = select_recovery_pose(prefer_latest=prefer_latest)

    if recovery_pose is None or latest_position_ned is None:
        print(f"🧯 Recovery hold: {reason}")
        await hold_position(drone, target_down, duration_s=0.8)
        return False

    safe_n, safe_e, _, safe_yaw = recovery_pose

    print(
        f"🧯 Recovering to {recovery_kind} safe pose: {reason} "
        f"N={safe_n:.2f} E={safe_e:.2f} yaw={safe_yaw:.1f}"
    )

    # Clamp target step distance to at most 1.0m to prevent aggressive acceleration/tilt spike
    curr_n = latest_position_ned.north_m
    curr_e = latest_position_ned.east_m
    dist_to_safe = math.hypot(safe_n - curr_n, safe_e - curr_e)
    if dist_to_safe > 1.0:
        ratio = 1.0 / dist_to_safe
        safe_n = curr_n + (safe_n - curr_n) * ratio
        safe_e = curr_e + (safe_e - curr_e) * ratio

    await drone.offboard.set_position_ned(
        PositionNedYaw(
            safe_n,
            safe_e,
            target_down,
            normalize_angle_deg(safe_yaw),
        )
    )

    deadline = time.time() + 1.2

    while time.time() < deadline:
        await asyncio.sleep(0.05)

        pitch = abs(latest_attitude["pitch"])
        roll = abs(latest_attitude["roll"])

        # Truly catastrophic attitude – abort mission immediately.
        if pitch > RECOVERY_HARD_LIMIT_DEG or roll > RECOVERY_HARD_LIMIT_DEG:
            raise RuntimeError("critical_state")

        # Moderate attitude spike – the repositioning maneuver itself is
        # causing instability.  Cancel the flight-to-safe-pose and hold
        # the current position so the controller can level out.
        if pitch > RECOVERY_ATTITUDE_TOLERANCE_DEG or roll > RECOVERY_ATTITUDE_TOLERANCE_DEG:
            print(
                f"⚠️ Recovery abort – attitude spike: "
                f"pitch={latest_attitude['pitch']:.1f} "
                f"roll={latest_attitude['roll']:.1f}; holding position"
            )
            await drone.offboard.set_position_ned(
                PositionNedYaw(
                    latest_position_ned.north_m,
                    latest_position_ned.east_m,
                    target_down,
                    latest_attitude["yaw"],
                )
            )
            # Give the controller time to stabilise before returning.
            await asyncio.sleep(0.6)

            # If still unstable after hold, climb higher to escape
            # ground-level obstacles (e.g. barrels the drone is sitting on).
            post_pitch = abs(latest_attitude["pitch"])
            post_roll = abs(latest_attitude["roll"])
            if post_pitch > RECOVERY_ATTITUDE_TOLERANCE_DEG or post_roll > RECOVERY_ATTITUDE_TOLERANCE_DEG:
                escape_alt = min(target_down, HIGH_SCAN_ALT_D) - 0.5
                print(
                    f"⚠️ Recovery still unstable after hold: "
                    f"pitch={latest_attitude['pitch']:.1f} "
                    f"roll={latest_attitude['roll']:.1f}; "
                    f"climbing to alt_d={escape_alt:.1f}"
                )
                await drone.offboard.set_position_ned(
                    PositionNedYaw(
                        latest_position_ned.north_m,
                        latest_position_ned.east_m,
                        escape_alt,
                        latest_attitude["yaw"],
                    )
                )
                await asyncio.sleep(1.0)
            break

        if math.hypot(
            latest_position_ned.north_m - safe_n,
            latest_position_ned.east_m - safe_e,
        ) < 0.18:
            break

    return False
    
def log_mission_action(
    action_type,
    label="",
    target_down=None,
    preferred_heading=None,
    selected_heading=None,
    heading_score=None,
    extra=None,
):
    if latest_position_ned is None:
        return

    clearances = monitor.get_directional_clearance()
    memory_debug = exploration_memory.debug_summary()
    summary = target_memory.summary()

    mission_logger.log(
        action_type=action_type,
        label=label,
        north_m=latest_position_ned.north_m,
        east_m=latest_position_ned.east_m,
        down_m=latest_position_ned.down_m,
        yaw_deg=latest_attitude["yaw"],
        pitch_deg=latest_attitude["pitch"],
        roll_deg=latest_attitude["roll"],
        target_down_m=target_down,
        range_from_start_m=distance_from_start(),
        preferred_heading_deg=preferred_heading,
        selected_heading_deg=selected_heading,
        heading_score=heading_score,
        clearances=clearances,
        visited_cells=memory_debug["visited_cells"],
        blocked_cells=memory_debug["blocked_cells"],
        red_confirmed=summary["red"],
        yellow_confirmed=summary["yellow"],
        candidate_count=len(summary["candidates"]),
        blocked_streak=blocked_streak,
        return_fail_streak=return_fail_streak,
        extra=extra,
    )


def critical_vehicle_state(target_down):
    if latest_position_ned is None:
        return False

    pitch = abs(latest_attitude["pitch"])
    roll = abs(latest_attitude["roll"])
    alt_err = altitude_error_m(target_down)

    if pitch > CRITICAL_ATTITUDE_DEG or roll > CRITICAL_ATTITUDE_DEG:
        print(
            f"🚨 Critical attitude: pitch={latest_attitude['pitch']:.1f}, "
            f"roll={latest_attitude['roll']:.1f}"
        )
        return True

    if alt_err > CRITICAL_ALTITUDE_DEVIATION_M:
        print(
            f"🚨 Critical altitude deviation: down={latest_position_ned.down_m:.2f}, "
            f"target={target_down:.2f}, error={alt_err:.2f}"
        )
        return True

    if distance_from_start() > HARD_RANGE_LIMIT_M:
        print(f"🚨 Hard range limit exceeded: {distance_from_start():.1f} m")
        return True

    return False


def vehicle_stable(target_down, attitude_limit_deg=MAX_ATTITUDE_DEG):
    if latest_position_ned is None:
        return False

    if abs(latest_attitude["pitch"]) > attitude_limit_deg:
        return False

    if abs(latest_attitude["roll"]) > attitude_limit_deg:
        return False

    if altitude_error_m(target_down) > ALT_TOLERANCE_M:
        return False

    return True


def narrow_corridor_passable(clearances):
    if not NARROW_CORRIDOR_ENABLED:
        return False

    front = clearances["center"]
    lower = clearances["lower_center"]
    side = min(clearances["left"], clearances["right"])

    if front < NARROW_CORRIDOR_MIN_FRONT_M:
        return False

    if lower < MIN_LOWER_MOVE_CLEARANCE_M:
        return False

    if side < NARROW_CORRIDOR_MIN_SIDE_M:
        return False

    return True


def narrow_corridor_active(clearances):
    return (
        narrow_corridor_passable(clearances)
        and min(clearances["left"], clearances["right"]) < MIN_SIDE_MOVE_CLEARANCE_M
    )


def reset_corridor_pid():
    global corridor_pid_integral
    global corridor_pid_previous_error
    global corridor_pid_last_time

    corridor_pid_integral = 0.0
    corridor_pid_previous_error = 0.0
    corridor_pid_last_time = None


def corridor_steer_adjustment_deg(clearances):
    global corridor_pid_integral
    global corridor_pid_previous_error
    global corridor_pid_last_time

    if not narrow_corridor_passable(clearances):
        reset_corridor_pid()
        return 0.0

    # Positive yaw steers toward the image/right side. If left clearance is
    # smaller than right clearance, this gently biases the path to the right.
    error = clearances["right"] - clearances["left"]
    now = time.time()

    if corridor_pid_last_time is None:
        dt = 0.12
        derivative = 0.0
    else:
        dt = clamp(now - corridor_pid_last_time, 0.05, 0.35)
        derivative = (error - corridor_pid_previous_error) / dt

    corridor_pid_integral = clamp(
        corridor_pid_integral + error * dt,
        -NARROW_CORRIDOR_PID_INTEGRAL_LIMIT,
        NARROW_CORRIDOR_PID_INTEGRAL_LIMIT,
    )
    corridor_pid_previous_error = error
    corridor_pid_last_time = now

    steer = (
        error * NARROW_CORRIDOR_STEER_GAIN_DEG_PER_M
        + corridor_pid_integral * NARROW_CORRIDOR_STEER_I_GAIN_DEG_PER_M_S
        + derivative * NARROW_CORRIDOR_STEER_D_GAIN_DEG_S_PER_M
    )

    return clamp(
        steer,
        -NARROW_CORRIDOR_MAX_STEER_DEG,
        NARROW_CORRIDOR_MAX_STEER_DEG,
    )


def corridor_steer_estimate_deg(clearances):
    if not narrow_corridor_passable(clearances):
        return 0.0

    error = clearances["right"] - clearances["left"]

    return clamp(
        error * NARROW_CORRIDOR_STEER_GAIN_DEG_PER_M,
        -NARROW_CORRIDOR_MAX_STEER_DEG,
        NARROW_CORRIDOR_MAX_STEER_DEG,
    )


def clearance_safe_for_yaw(clearances):
    front = clearances["center"]
    lower = clearances["lower_center"]
    side = min(clearances["left"], clearances["right"])

    if front < YAW_MIN_FRONT_CLEARANCE_M:
        return False, f"yaw_front={front:.2f}"

    if side < YAW_MIN_SIDE_CLEARANCE_M:
        return False, (
            f"yaw_side left={clearances['left']:.2f} "
            f"right={clearances['right']:.2f}"
        )

    if lower < MIN_LOWER_MOVE_CLEARANCE_M:
        return False, f"yaw_lower={lower:.2f}"

    return True, "clear"


def clearance_safe_for_motion(clearances, allow_corridor=False):
    front = clearances["center"]
    lower = clearances["lower_center"]
    side = min(clearances["left"], clearances["right"])

    if front < MIN_FRONT_MOVE_CLEARANCE_M:
        return False, f"front={front:.2f}"

    if side < MIN_SIDE_MOVE_CLEARANCE_M:
        if allow_corridor and narrow_corridor_passable(clearances):
            return True, (
                f"corridor side left={clearances['left']:.2f} "
                f"right={clearances['right']:.2f}"
            )

        return False, (
            f"side left={clearances['left']:.2f} "
            f"right={clearances['right']:.2f}"
        )

    if lower < MIN_LOWER_MOVE_CLEARANCE_M:
        return False, f"lower={lower:.2f}"

    return True, "clear"


# ============================================================
# MAVSDK / PX4 helpers
# ============================================================

async def wait_for_connection(drone):
    print("Waiting for drone connection...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected.")
            return


async def wait_for_local_position(drone):
    """
    GNSS-free health check.

    We require local position only.
    We do not require global position.
    """
    print("Waiting for local position estimate. GNSS/global position is not required...")

    async for health in drone.telemetry.health():
        print(
            f"health: local={health.is_local_position_ok}, "
            f"global={health.is_global_position_ok}, "
            f"home={health.is_home_position_ok}"
        )

        if health.is_local_position_ok:
            print("Local position OK. Continuing without GNSS/global position.")
            return

        await asyncio.sleep(0.5)


async def wait_for_telemetry():
    print("Waiting for local telemetry...")

    while latest_position_ned is None:
        await asyncio.sleep(0.1)

    print("Local telemetry ready.")


async def wait_for_camera_depth(timeout_s=8.0):
    print("Waiting for RGB camera and depth frames...")

    start = time.time()

    while time.time() - start < timeout_s:
        if latest_frame is not None and monitor.latest_depth is not None:
            print("RGB camera and depth camera ready.")
            return True

        await asyncio.sleep(0.1)

    print("Warning: RGB/depth frames not fully ready yet.")
    return False


async def telemetry_task(drone):
    async def read_pos():
        global latest_position_ned

        async for pos in drone.telemetry.position_velocity_ned():
            latest_position_ned = pos.position

    async def read_att():
        global latest_attitude

        async for att in drone.telemetry.attitude_euler():
            latest_attitude = {
                "pitch": att.pitch_deg,
                "roll": att.roll_deg,
                "yaw": att.yaw_deg,
            }

    await asyncio.gather(read_pos(), read_att())


async def arm_with_retry(drone, attempts=10):
    for i in range(1, attempts + 1):
        try:
            print(f"Arming attempt {i}/{attempts}...")
            await drone.action.arm()
            print("Armed.")
            return True

        except ActionError as error:
            print(f"Arming denied: {error}. Retrying...")
            await asyncio.sleep(2)

        except grpc.aio.AioRpcError as error:
            print(f"MAVSDK connection error during arming: {error.code()} {error.details()}")
            await asyncio.sleep(3)

    return False


async def prime_and_start_offboard(drone, target_down):
    print("Priming Offboard with zero velocity setpoints...")

    yaw = latest_attitude["yaw"]

    for _ in range(25):
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, yaw)
        )
        await asyncio.sleep(0.05)

    print("Starting Offboard mode...")
    await drone.offboard.start()

    if latest_position_ned is not None:
        await drone.offboard.set_position_ned(
            PositionNedYaw(
                latest_position_ned.north_m,
                latest_position_ned.east_m,
                target_down,
                yaw,
            )
        )
        await asyncio.sleep(0.5)


async def stop_and_land(drone, reason):
    print(f"🛑 Landing: {reason}")

    try:
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, latest_attitude["yaw"])
        )
        await asyncio.sleep(0.2)
    except Exception:
        pass

    try:
        await drone.offboard.stop()
    except Exception:
        pass

    await drone.action.land()


# ============================================================
# Perception and target memory
# ============================================================

def _normalise_label_text(label):
    return str(label or "").strip().lower().replace("-", "_").replace(" ", "_")


def _colour_from_yolo_label(label):
    label = _normalise_label_text(label)
    if not label:
        return None

    ignored_tokens = (
        "poison",
        "toxic",
        "hazard",
        "decoy",
        "nonfuel",
        "non_fuel",
        "not_fuel",
    )
    if any(token in label for token in ignored_tokens):
        return "ignore"

    if "yellow" in label:
        return "yellow"

    if "red" in label:
        return "red"

    return None


def _yolo_class_name(names, class_id):
    if names is None:
        return str(class_id)

    if isinstance(names, dict):
        return str(names.get(class_id, names.get(str(class_id), class_id)))

    if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
        return str(names[class_id])

    return str(class_id)


def _tensor_to_list(value):
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value).tolist()


def _tensor_to_float(value):
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "item"):
        return float(value.item())
    return float(value)


def _clamp_xyxy_to_bbox(x1, y1, x2, y2, frame_shape):
    img_h, img_w = frame_shape[:2]

    x1 = int(round(max(0, min(img_w - 1, x1))))
    y1 = int(round(max(0, min(img_h - 1, y1))))
    x2 = int(round(max(0, min(img_w, x2))))
    y2 = int(round(max(0, min(img_h, y2))))

    if x2 <= x1 or y2 <= y1:
        return None

    return (x1, y1, x2 - x1, y2 - y1)


def _colour_from_bbox_pixels(frame, bbox):
    x, y, w, h = bbox
    roi = frame[y:y + h, x:x + w]
    if roi.size == 0:
        return None

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    red1 = cv2.inRange(hsv, np.array([0, 55, 55]), np.array([16, 255, 255]))
    red2 = cv2.inRange(hsv, np.array([165, 55, 55]), np.array([180, 255, 255]))
    orange_red = cv2.inRange(hsv, np.array([8, 45, 70]), np.array([26, 255, 255]))
    yellow = cv2.inRange(hsv, np.array([18, 55, 70]), np.array([45, 255, 255]))

    red_pixels = int(np.count_nonzero(cv2.bitwise_or(cv2.bitwise_or(red1, red2), orange_red)))
    yellow_pixels = int(np.count_nonzero(yellow))
    min_pixels = max(8, int(0.015 * w * h))

    if red_pixels < min_pixels and yellow_pixels < min_pixels:
        return None

    if yellow_pixels > red_pixels * 1.15:
        return "yellow"

    return "red"


def initialize_mission_yolo_model():
    global mission_yolo_model
    global mission_yolo_load_attempted

    if mission_yolo_load_attempted:
        return mission_yolo_model

    mission_yolo_load_attempted = True

    if not MISSION_YOLO_ENABLED:
        print("Mission YOLO detector disabled. Fast HSV detector remains active.")
        return None

    if not os.path.exists(MISSION_YOLO_MODEL_PATH):
        print(
            "WARNING: Mission YOLO model not found: "
            f"{MISSION_YOLO_MODEL_PATH}. Using HSV detector only. "
            "Place the trained YOLOv8n weight there or set MISSION_YOLO_MODEL_PATH."
        )
        return None

    YOLO = load_yolo_class()
    if YOLO is None:
        print("WARNING: ultralytics is not installed. Mission YOLO disabled; HSV remains active.")
        return None

    try:
        mission_yolo_model = YOLO(MISSION_YOLO_MODEL_PATH)
    except Exception as exc:
        print(f"WARNING: failed to load mission YOLO model '{MISSION_YOLO_MODEL_PATH}': {exc}")
        mission_yolo_model = None
        return None

    names = getattr(mission_yolo_model, "names", None)
    print(
        f"Mission YOLO detector loaded: {MISSION_YOLO_MODEL_PATH} "
        f"(conf={MISSION_YOLO_CONFIDENCE_THRESHOLD}, imgsz={MISSION_YOLO_IMGSZ}, "
        f"device={MISSION_YOLO_DEVICE}, period={MISSION_YOLO_PERIOD_S:.2f}s, names={names})"
    )
    return mission_yolo_model


def detect_yolo_frame_candidates(frame):
    global mission_yolo_model
    global mission_yolo_last_run_time

    model = initialize_mission_yolo_model()
    if model is None:
        return []

    now = time.time()
    if now - mission_yolo_last_run_time < MISSION_YOLO_PERIOD_S:
        return []

    mission_yolo_last_run_time = now

    try:
        results = model(
            frame,
            conf=MISSION_YOLO_CONFIDENCE_THRESHOLD,
            imgsz=MISSION_YOLO_IMGSZ,
            device=MISSION_YOLO_DEVICE,
            max_det=MISSION_YOLO_MAX_DETECTIONS,
            verbose=False,
        )
    except Exception as exc:
        print(f"WARNING: mission YOLO inference failed: {exc}. Continuing with HSV only.")
        mission_yolo_model = None
        return []

    result = results[0] if results else None
    if result is None or result.boxes is None:
        return []

    names = getattr(result, "names", None) or getattr(model, "names", None)
    detections = []

    for box in result.boxes:
        xyxy = _tensor_to_list(box.xyxy[0])
        if len(xyxy) < 4:
            continue

        bbox = _clamp_xyxy_to_bbox(xyxy[0], xyxy[1], xyxy[2], xyxy[3], frame.shape)
        if bbox is None:
            continue

        x, y, w, h = bbox
        if w < 6 or h < 8:
            continue

        class_id = int(_tensor_to_float(box.cls[0])) if box.cls is not None else -1
        model_label = _yolo_class_name(names, class_id)
        colour = _colour_from_yolo_label(model_label)
        if colour == "ignore":
            continue

        colour = colour or _colour_from_bbox_pixels(frame, bbox)
        if colour not in ("red", "yellow"):
            continue

        confidence = _tensor_to_float(box.conf[0]) if box.conf is not None else 0.0
        detections.append(
            {
                "colour": colour,
                "label": barrel_label_for_colour(colour),
                "source": "yolov8n",
                "model_label": model_label,
                "bbox": bbox,
                "center": (x + w // 2, y + h // 2),
                "area": float(w * h),
                "confidence": confidence,
            }
        )

    return detections


def _bbox_iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b

    ax2 = ax + aw
    ay2 = ay + ah
    bx2 = bx + bw
    by2 = by + bh

    inter_x1 = max(ax, bx)
    inter_y1 = max(ay, by)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)

    inter_w = max(0, inter_x2 - inter_x1)
    inter_h = max(0, inter_y2 - inter_y1)
    intersection = inter_w * inter_h
    union = aw * ah + bw * bh - intersection

    if union <= 0:
        return 0.0

    return intersection / union


def _dedupe_detections(detections, iou_threshold=0.45):
    kept = []

    for det in sorted(detections, key=lambda item: item.get("confidence", 0.0), reverse=True):
        overlaps_existing = any(
            det["colour"] == existing["colour"]
            and _bbox_iou(det["bbox"], existing["bbox"]) >= iou_threshold
            for existing in kept
        )
        if not overlaps_existing:
            kept.append(det)

    return kept


def estimate_bearing_deg(det, frame_shape):
    cx, _ = det["center"]
    _, width = frame_shape[:2]

    norm_x = (cx - width / 2.0) / max(width, 1)
    camera_offset = norm_x * IMX214_HFOV_DEG

    return normalize_angle_deg(latest_attitude["yaw"] + camera_offset)


def localize_detection(det, frame_shape):
    depth_m = monitor.sample_depth_for_rgb_bbox(det["bbox"], frame_shape)
    bearing_deg = estimate_bearing_deg(det, frame_shape)

    if latest_position_ned is None or depth_m is None:
        return bearing_deg, None, None, depth_m

    yaw_rad = math.radians(bearing_deg)

    target_n = latest_position_ned.north_m + depth_m * math.cos(yaw_rad)
    target_e = latest_position_ned.east_m + depth_m * math.sin(yaw_rad)

    return bearing_deg, target_n, target_e, depth_m


def detect_frame_candidates(frame, allowed_colours):
    hsv_detections, _, _, _ = detect_small_fuel_barrels(frame)
    detections = list(hsv_detections)
    detections.extend(detect_yolo_frame_candidates(frame))
    detections = _dedupe_detections(detections)

    return [
        det
        for det in detections
        if det.get("confidence", 0.0) >= DETECTION_CONFIDENCE_THRESHOLD
        and det["colour"] in allowed_colours
    ]


def save_confirmation_evidence(frame, det, confirmed, summary):
    os.makedirs(EVIDENCE_DIR, exist_ok=True)

    draw = frame.copy()
    x, y, w, h = det["bbox"]
    colour = det["colour"]
    bbox_label = det.get("label") or barrel_label_for_colour(colour)
    box_colour = (0, 0, 255) if colour == "red" else (0, 255, 255)

    cv2.rectangle(draw, (x, y), (x + w, y + h), box_colour, 3)
    cv2.putText(
        draw,
        bbox_label,
        (x, max(20, y - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        box_colour,
        2,
    )
    cv2.putText(
        draw,
        f"CONFIRMED {bbox_label} score={get_score(summary)}",
        (20, 55),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.2,
        box_colour,
        3,
    )

    if det.get("depth_m") is not None:
        cv2.putText(
            draw,
            f"depth={det['depth_m']:.2f}m bearing={det.get('bearing_deg', 0.0):.1f}",
            (20, 95),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2,
        )

    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    path = os.path.join(EVIDENCE_DIR, f"CONFIRMED_{bbox_label}_{ts}.png")
    cv2.imwrite(path, draw)
    print(f"Saved confirmation evidence: {path}")
    return path


def trigger_photo_burst(num_frames=STOP_CAPTURE_BURST_FRAMES):
    global last_photo_burst_time

    if camera_photo_saver is None:
        return

    now = time.time()
    if now - last_photo_burst_time < MIN_PHOTO_BURST_INTERVAL_S:
        return

    last_photo_burst_time = now
    camera_photo_saver.trigger_capture_burst(num_frames)


def trigger_yolo_detection_burst(num_frames=CONFIRM_CAPTURE_BURST_FRAMES):
    global last_yolo_burst_time

    if not YOLO_BURST_ENABLED:
        return

    if camera_photo_saver is None or camera_photo_saver.model is None:
        return

    now = time.time()
    if now - last_yolo_burst_time < MIN_YOLO_BURST_INTERVAL_S:
        return

    last_yolo_burst_time = now
    camera_photo_saver.trigger_detection_burst(num_frames)


async def scan_current_view(label, frames=SCAN_FRAMES_PER_VIEW):
    newly_confirmed = None

    for _ in range(frames):
        confirmed = await process_latest_frame_async(label=label)
        if confirmed is not None:
            newly_confirmed = confirmed

        await asyncio.sleep(SCAN_FRAME_INTERVAL_S)

    return newly_confirmed


async def process_latest_frame_async(label="background"):
    """
    Processes the latest RGB frame once.

    Detection does not immediately count as score.
    It updates target memory. Only persistent, viewpoint-consistent candidates confirm.
    """
    if latest_frame is None or latest_position_ned is None:
        return None

    frame = latest_frame.copy()
    allowed_colours = tuple(active_allowed_colours)
    loop = asyncio.get_running_loop()

    async with frame_processing_lock:
        detections = await loop.run_in_executor(
            None,
            detect_frame_candidates,
            frame,
            allowed_colours,
        )

        if detections:
            print(
                f"👀 [{label}] visual candidates={len(detections)} "
                f"allowed={allowed_colours}"
            )

        newly_confirmed = None

        for det in detections:
            bearing, target_n, target_e, depth_m = localize_detection(det, frame.shape)

            if depth_m is None or not (0.35 <= depth_m <= 9.0):
                continue

            enriched = det.copy()
            enriched["bearing_deg"] = bearing
            enriched["target_n"] = target_n
            enriched["target_e"] = target_e
            enriched["depth_m"] = depth_m

            confirmed = target_memory.add_detection(
                enriched,
                observer_yaw_deg=latest_attitude["yaw"],
            )

            if confirmed is not None:
                summary = target_memory.summary()
                confirmation_reason = confirmed.get("confirmation_reason", "normal")
                confirmed_label = confirmed.get("label") or barrel_label_for_colour(confirmed["colour"])
                print(
                    f"🎯 CONFIRMED {confirmed_label} | "
                    f"red={summary['red']} yellow={summary['yellow']} "
                    f"score={get_score(summary)} reason={confirmation_reason}"
                )
                newly_confirmed = confirmed
                await loop.run_in_executor(
                    None,
                    save_confirmation_evidence,
                    frame,
                    enriched,
                    confirmed,
                    summary,
                )

                if active_investigation_enabled:
                    trigger_photo_burst(CONFIRM_CAPTURE_BURST_FRAMES)

                confirmed_event.set()

            else:
                if active_investigation_enabled:
                    trigger_yolo_detection_burst(num_frames=YOLO_BURST_FRAMES)
                new_candidate_event.set()

        return newly_confirmed


async def perception_task():
    global mission_should_stop

    while not mission_should_stop:
        await process_latest_frame_async(label="background")
        await asyncio.sleep(PERCEPTION_PERIOD_S)


async def investigate_candidate(drone, target_down):
    global last_investigation_time

    if not active_investigation_enabled:
        return

    now = time.time()

    if now - last_investigation_time < INVESTIGATE_COOLDOWN_S:
        return

    candidate = target_memory.get_best_unconfirmed_candidate(
        allowed_colours=active_allowed_colours
    )

    if candidate is None:
        return

    last_investigation_time = now

    if "bearings" in candidate and candidate["bearings"]:
        base_yaw = sum(candidate["bearings"]) / len(candidate["bearings"])
    else:
        base_yaw = latest_attitude["yaw"]

    base_yaw = normalize_angle_deg(base_yaw)

    print(
        f"🔎 Investigating {candidate['colour']} candidate: "
        f"count={candidate['count']} conf={candidate['confidence']:.2f} "
        f"base_yaw={base_yaw:.1f}"
    )

    log_mission_action(
        action_type="INVESTIGATE_CANDIDATE",
        label="investigate",
        target_down=target_down,
        selected_heading=base_yaw,
        extra={
            "candidate_colour": candidate["colour"],
            "candidate_count": candidate["count"],
            "candidate_confidence": candidate["confidence"],
        },
    )

    trigger_photo_burst(CONFIRM_CAPTURE_BURST_FRAMES)

    for offset in INVESTIGATION_YAW_OFFSETS_DEG:
        if timed_out():
            return

        await wait_for_teleop_release(drone, target_down)

        if critical_vehicle_state(target_down):
            raise RuntimeError("critical_state")

        yaw = normalize_angle_deg(base_yaw + offset)

        await drone.offboard.set_position_ned(
            PositionNedYaw(
                latest_position_ned.north_m,
                latest_position_ned.east_m,
                target_down,
                yaw,
            )
        )
        await asyncio.sleep(INVESTIGATION_SETTLE_S)

        await scan_current_view(label="investigate", frames=SCAN_FRAMES_PER_VIEW)

        if eligibility_met(target_memory.summary()):
            return


async def handle_candidate_event(drone, target_down):
    if not new_candidate_event.is_set():
        return

    new_candidate_event.clear()

    if active_investigation_enabled:
        await investigate_candidate(drone, target_down)


# ============================================================
# Motion
# ============================================================

async def hold_position(drone, target_down, duration_s=0.5):
    if latest_position_ned is None:
        await asyncio.sleep(duration_s)
        return

    await wait_for_teleop_release(drone, target_down)

    if critical_vehicle_state(target_down):
        raise RuntimeError("critical_state")

    await drone.offboard.set_position_ned(
        PositionNedYaw(
            latest_position_ned.north_m,
            latest_position_ned.east_m,
            target_down,
            latest_attitude["yaw"],
        )
    )

    await asyncio.sleep(duration_s)


async def set_yaw(drone, yaw_deg, target_down, duration_s=0.35):
    if latest_position_ned is None:
        await asyncio.sleep(duration_s)
        return

    await wait_for_teleop_release(drone, target_down)

    if critical_vehicle_state(target_down):
        raise RuntimeError("critical_state")

    target_yaw = normalize_angle_deg(yaw_deg)
    current_yaw = latest_attitude["yaw"]
    remaining = normalize_angle_deg(target_yaw - current_yaw)

    yaw_steps = []
    while abs(remaining) > MAX_YAW_STEP_DEG:
        current_yaw = normalize_angle_deg(
            current_yaw + math.copysign(MAX_YAW_STEP_DEG, remaining)
        )
        yaw_steps.append(current_yaw)
        remaining = normalize_angle_deg(target_yaw - current_yaw)

    yaw_steps.append(target_yaw)

    for step_yaw in yaw_steps:
        await wait_for_teleop_release(drone, target_down)

        c = monitor.get_directional_clearance()
        safe, reason = clearance_safe_for_yaw(c)

        if not safe:
            raise RuntimeError(f"yaw_blocked:{reason}")

        await drone.offboard.set_position_ned(
            PositionNedYaw(
                latest_position_ned.north_m,
                latest_position_ned.east_m,
                target_down,
                normalize_angle_deg(step_yaw),
            )
        )

        deadline = time.time() + max(duration_s, YAW_SETTLE_TIMEOUT_S)

        while time.time() < deadline:
            await wait_for_teleop_release(drone, target_down)

            if critical_vehicle_state(target_down):
                raise RuntimeError("critical_state")

            if angle_diff_deg(latest_attitude["yaw"], step_yaw) <= YAW_SETTLE_TOLERANCE_DEG:
                break

            await asyncio.sleep(0.05)

        await asyncio.sleep(min(duration_s, 0.08))


async def move_to_position_step(
    drone,
    target_n,
    target_e,
    target_down,
    yaw_deg,
    label,
    allow_corridor=False,
):
    """
    Move to a specific local N/E point using continuous VFH local planning.

    Used by both exploration and return-home.
    """
    if latest_position_ned is None:
        return False

    await wait_for_teleop_release(drone, target_down)

    start_time = time.time()
    timeout_s = MOVE_TIMEOUT_S
    
    # Scale arrived threshold dynamically based on step distance to avoid zero-second step completion
    curr_n_init = latest_position_ned.north_m
    curr_e_init = latest_position_ned.east_m
    initial_dist = math.hypot(target_n - curr_n_init, target_e - curr_e_init)
    arrived_threshold = max(0.08, min(0.20, initial_dist * 0.40))

    while time.time() - start_time < timeout_s:
        await asyncio.sleep(0.05)

        await wait_for_teleop_release(drone, target_down)

        if critical_vehicle_state(target_down):
            raise RuntimeError("critical_state")

        if not vehicle_stable(
            target_down,
            attitude_limit_deg=MID_STEP_MAX_ATTITUDE_DEG,
        ):
            reason = (
                f"mid-step unstable pitch={latest_attitude['pitch']:.1f} "
                f"roll={latest_attitude['roll']:.1f}"
            )
            print(
                f"🚨 [{label}] mid-step abort unstable: "
                f"pitch={latest_attitude['pitch']:.1f} roll={latest_attitude['roll']:.1f}"
            )
            await recover_to_last_safe(
                drone,
                target_down,
                reason,
                prefer_latest=False,
            )
            return False

        curr_n = latest_position_ned.north_m
        curr_e = latest_position_ned.east_m
        dist_to_target = math.hypot(target_n - curr_n, target_e - curr_e)

        if dist_to_target < arrived_threshold:
            mark_current_cell_visited()
            remember_safe_position()
            if active_investigation_enabled:
                trigger_photo_burst(STOP_CAPTURE_BURST_FRAMES)
            return True

        target_heading = math.degrees(math.atan2(target_e - curr_e, target_n - curr_n))
        depth_frame = monitor.latest_depth

        best_heading = target_heading

        if depth_frame is not None:
            try:
                h, w = depth_frame.shape[:2]
                pitch_deg = latest_attitude["pitch"]
                # Dynamic pitch compensation crop: shift crop window up if looking down, down if looking up
                y_center = int(h * 0.38) + int(pitch_deg * 8.0)
                y1 = max(0, y_center - 80)
                y2 = min(h, y_center + 80)
                cropped_depth = depth_frame[y1:y2, :]

                num_bins = 24
                best_score = -1e9

                for i in range(num_bins):
                    x_start = int(i * w / num_bins)
                    x_end = int((i + 1) * w / num_bins)
                    bin_depths = cropped_depth[:, x_start:x_end]

                    if np.all(np.isnan(bin_depths)):
                        d = 6.0
                    else:
                        d = np.nanpercentile(bin_depths, 20)
                        if np.isnan(d) or np.isinf(d):
                            d = 6.0

                    # Calculate camera-relative bin angle
                    u_center = (x_start + x_end) / 2.0
                    angle_rad = math.atan((u_center - (w / 2.0)) / 433.0)
                    angle_deg = math.degrees(angle_rad)

                    # Candidate absolute NED heading
                    candidate_heading = normalize_angle_deg(latest_attitude["yaw"] + angle_deg)

                    # 1. Alignment penalty
                    alignment_penalty = -0.4 * angle_diff_deg(candidate_heading, target_heading)

                    # 2. Revisit memory penalty
                    chk_n = curr_n + 2.0 * math.cos(math.radians(candidate_heading))
                    chk_e = curr_e + 2.0 * math.sin(math.radians(candidate_heading))
                    cell = exploration_memory._cell(chk_n, chk_e)
                    if cell in exploration_memory.blocked_cells:
                        memory_score = -80.0
                    else:
                        memory_score = -1.5 * exploration_memory.visited_cells.get(cell, 0)

                    # 3. Obstacle cost penalty
                    cost = 1.0 / (d + 1e-3) if d > 0.8 else 1.0
                    obstacle_penalty = -25.0 * cost

                    # 4. Clearance score
                    clearance_score = min(d, 6.0) * 1.5

                    score = alignment_penalty + clearance_score + obstacle_penalty + memory_score
                    if score > best_score:
                        best_score = score
                        best_heading = candidate_heading
            except Exception as e:
                print(f"⚠️ Error running VFH local planner: {e}. Defaulting to straight path.")
                best_heading = target_heading

        # Smooth heading transitions (clamp deviation from straight path)
        heading_delta = normalize_angle_deg(best_heading - target_heading)
        heading_delta = max(-35.0, min(35.0, heading_delta))
        best_heading = normalize_angle_deg(target_heading + heading_delta)

        # Project a running setpoint 1.25m ahead in the selected heading
        step_size = min(1.25, dist_to_target)
        cmd_n = curr_n + step_size * math.cos(math.radians(best_heading))
        cmd_e = curr_e + step_size * math.sin(math.radians(best_heading))
        cmd_yaw = best_heading

        await drone.offboard.set_position_ned(
            PositionNedYaw(
                cmd_n,
                cmd_e,
                target_down,
                normalize_angle_deg(cmd_yaw),
            )
        )

        # Quick directional clearance check for collision emergency
        c = monitor.get_directional_clearance()
        if c["center"] < 0.65 or c["lower_center"] < 0.40:
            print(
                f"🚨 [{label}] VFH emergency stop: center={c['center']:.2f} "
                f"L={c['left']:.2f} R={c['right']:.2f} LC={c['lower_center']:.2f}"
            )
            exploration_memory.mark_blocked_ray(
                curr_n,
                curr_e,
                latest_attitude["yaw"],
                min(c["center"], c["lower_center"]),
            )
            await recover_to_last_safe(
                drone,
                target_down,
                f"VFH center too close: {c['center']:.2f}",
                prefer_latest=False,
            )
            return False

    mark_current_cell_visited()
    remember_safe_position()
    if active_investigation_enabled:
        trigger_photo_burst(STOP_CAPTURE_BURST_FRAMES)
    return True


async def return_home_step(drone, target_down):
    """
    Dedicated return-home step.

    When outside SOFT_RANGE_LIMIT_M, normal exploration pauses and the drone
    moves toward the local start point until it is back under RESUME_RANGE_M.
    """
    global return_fail_streak
    global blocked_streak

    if latest_position_ned is None:
        return False

    if critical_vehicle_state(target_down):
        raise RuntimeError("critical_state")

    current_range = distance_from_start()

    if current_range <= RESUME_RANGE_M:
        return_fail_streak = 0
        return True

    home_yaw = heading_to_start_deg()
    c = monitor.get_directional_clearance()
    front = c["center"]

    print(
        f"🏠 Return-home mode: range={current_range:.1f}m, "
        f"home_yaw={home_yaw:.1f}, front={front:.2f}"
    )

    log_mission_action(
        action_type="RETURN_HOME_START",
        label="return_home",
        target_down=target_down,
        preferred_heading=home_yaw,
        selected_heading=home_yaw,
        extra={
            "current_range_m": current_range,
            "front_m": front,
        },
    )

    try:
        await set_yaw(drone, home_yaw, target_down, duration_s=0.35)
    except RuntimeError as error:
        if str(error).startswith("yaw_blocked:"):
            return_fail_streak += 1
            print(f"🏠 Return yaw blocked: {error}. return_fail_streak={return_fail_streak}")
            if return_fail_streak >= MAX_RETURN_FAIL_STREAK:
                raise RuntimeError("return_home_failed")
            return False
        raise

    c = monitor.get_directional_clearance()
    front = c["center"]

    if front < monitor.obstacle_distance_m:
        exploration_memory.mark_blocked_ray(
            latest_position_ned.north_m,
            latest_position_ned.east_m,
            home_yaw,
            front,
        )

        turn = 45 if c["right"] >= c["left"] else -45
        home_yaw = normalize_angle_deg(home_yaw + turn)

        print(
            f"🏠 Home path blocked front={front:.2f}. "
            f"Trying detour yaw={home_yaw:.1f}"
        )

        try:
            await set_yaw(drone, home_yaw, target_down, duration_s=0.35)
        except RuntimeError as error:
            if str(error).startswith("yaw_blocked:"):
                return_fail_streak += 1
                print(f"🏠 Return detour yaw blocked: {error}. return_fail_streak={return_fail_streak}")
                if return_fail_streak >= MAX_RETURN_FAIL_STREAK:
                    raise RuntimeError("return_home_failed")
                return False
            raise

        c = monitor.get_directional_clearance()
        front = c["center"]

        if front < monitor.obstacle_distance_m:
            exploration_memory.mark_blocked_ray(
                latest_position_ned.north_m,
                latest_position_ned.east_m,
                home_yaw,
                front,
            )

            return_fail_streak += 1
            print(f"🏠 Return detour blocked. return_fail_streak={return_fail_streak}")

            log_mission_action(
                action_type="RETURN_HOME_BLOCKED",
                label="return_home",
                target_down=target_down,
                preferred_heading=home_yaw,
                selected_heading=home_yaw,
                extra={
                    "front_m": front,
                    "return_fail_streak": return_fail_streak,
                },
            )

            if return_fail_streak >= MAX_RETURN_FAIL_STREAK:
                raise RuntimeError("return_home_failed")

            return False

    old_range = distance_from_start()

    yaw_rad = math.radians(home_yaw)
    target_n = latest_position_ned.north_m + RETURN_STEP_M * math.cos(yaw_rad)
    target_e = latest_position_ned.east_m + RETURN_STEP_M * math.sin(yaw_rad)

    print(
        f"🏠 Returning {RETURN_STEP_M:.2f}m toward start: "
        f"range={old_range:.1f}, yaw={home_yaw:.1f}"
    )

    moved = await move_to_position_step(
        drone,
        target_n,
        target_e,
        target_down,
        home_yaw,
        label="return_home",
    )

    new_range = distance_from_start()

    if moved and new_range < old_range:
        print(f"🏠 Range reduced {old_range:.1f} -> {new_range:.1f}")

        return_fail_streak = 0
        blocked_streak = 0

        log_mission_action(
            action_type="RETURN_HOME_SUCCESS",
            label="return_home",
            target_down=target_down,
            selected_heading=home_yaw,
            extra={
                "old_range_m": old_range,
                "new_range_m": new_range,
            },
        )

        return True

    return_fail_streak += 1
    print(f"🏠 Return did not reduce range. return_fail_streak={return_fail_streak}")

    log_mission_action(
        action_type="RETURN_HOME_FAILED_STEP",
        label="return_home",
        target_down=target_down,
        selected_heading=home_yaw,
        extra={
            "old_range_m": old_range,
            "new_range_m": new_range,
            "return_fail_streak": return_fail_streak,
        },
    )

    if return_fail_streak >= MAX_RETURN_FAIL_STREAK:
        raise RuntimeError("return_home_failed")

    return False


async def move_in_heading(
    drone,
    preferred_heading_deg,
    target_down,
    label,
    allow_memory_nudge=True,
):
    global blocked_streak
    global return_fail_streak

    if latest_position_ned is None:
        return False

    if critical_vehicle_state(target_down):
        raise RuntimeError("critical_state")

    # Wait up to 0.4s for attitude to settle if temporarily tilted from previous stride deceleration
    settle_start = time.time()
    while time.time() - settle_start < 0.40:
        if vehicle_stable(target_down, attitude_limit_deg=12.0):
            break
        await asyncio.sleep(0.05)

    if not vehicle_stable(target_down, attitude_limit_deg=12.0):
        print(
            f"⚠️ Not stable: pitch={latest_attitude['pitch']:.1f}, "
            f"roll={latest_attitude['roll']:.1f}, "
            f"down={latest_position_ned.down_m:.2f}; retreating."
        )
        return await recover_to_last_safe(
            drone,
            target_down,
            (
                f"pre-move unstable pitch={latest_attitude['pitch']:.1f} "
                f"roll={latest_attitude['roll']:.1f}"
            ),
            prefer_latest=False,
        )

    mark_current_cell_visited()

    if distance_from_start() > SOFT_RANGE_LIMIT_M:
        await return_home_step(drone, target_down)
        return False

    c = monitor.get_directional_clearance()
    pre_move_safe, pre_move_reason = clearance_safe_for_motion(
        c,
        allow_corridor=NARROW_CORRIDOR_ENABLED,
    )

    preferred_heading_deg = normalize_angle_deg(preferred_heading_deg)

    if allow_memory_nudge:
        selected_heading, heading_score = exploration_memory.choose_heading(
            current_n=latest_position_ned.north_m,
            current_e=latest_position_ned.east_m,
            current_yaw_deg=latest_attitude["yaw"],
            preferred_heading_deg=preferred_heading_deg,
            clearances=c,
        )
    else:
        selected_heading = preferred_heading_deg
        heading_score = exploration_memory.heading_score(
            current_n=latest_position_ned.north_m,
            current_e=latest_position_ned.east_m,
            candidate_heading_deg=selected_heading,
            current_yaw_deg=latest_attitude["yaw"],
            preferred_heading_deg=preferred_heading_deg,
            clearances=c,
        )

    log_mission_action(
        action_type="HEADING_SELECTED",
        label=label,
        target_down=target_down,
        preferred_heading=preferred_heading_deg,
        selected_heading=selected_heading,
        heading_score=heading_score,
        extra={
            "pre_select_clearance_safe": pre_move_safe,
            "pre_select_clearance_reason": pre_move_reason,
            "allow_memory_nudge": allow_memory_nudge,
        },
    )

    heading_delta = abs(normalize_angle_deg(selected_heading - preferred_heading_deg))

    if allow_memory_nudge and heading_delta > 35:
        print(
            f"⚠️ Memory tried excessive heading change: "
            f"preferred={preferred_heading_deg:.1f}, selected={selected_heading:.1f}. "
            f"Clamping back to preferred."
        )
        selected_heading = preferred_heading_deg

    elif allow_memory_nudge and heading_delta > 10:
        print(
            f"🧠 Path memory nudged heading: "
            f"preferred={preferred_heading_deg:.1f}, selected={selected_heading:.1f}, "
            f"score={heading_score:.1f}"
        )

    turn_delta = angle_diff_deg(selected_heading, latest_attitude["yaw"])
    side_clearance = min(c["left"], c["right"])

    if not pre_move_safe:
        yaw_safe, yaw_reason = clearance_safe_for_yaw(c)

        if not yaw_safe or turn_delta <= YAW_SETTLE_TOLERANCE_DEG:
            exploration_memory.mark_blocked_ray(
                latest_position_ned.north_m,
                latest_position_ned.east_m,
                latest_attitude["yaw"],
                min(c["center"], c["lower_center"]),
            )
            return await recover_to_last_safe(
                drone,
                target_down,
                f"pre-move unsafe {pre_move_reason}; yaw={yaw_reason}",
                prefer_latest=False,
            )

        print(
            f"🧭 [{label}] current view unsafe ({pre_move_reason}); "
            f"yawing {turn_delta:.1f} deg toward planned opening."
        )
        log_mission_action(
            action_type="PRE_YAW_UNSAFE_CONTINUE",
            label=label,
            target_down=target_down,
            preferred_heading=preferred_heading_deg,
            selected_heading=selected_heading,
            heading_score=heading_score,
            extra={
                "pre_move_reason": pre_move_reason,
                "yaw_reason": yaw_reason,
                "turn_delta_deg": turn_delta,
                "side_clearance_m": side_clearance,
            },
        )

    try:
        await set_yaw(drone, selected_heading, target_down, duration_s=0.3)
    except RuntimeError as error:
        if str(error).startswith("yaw_blocked:"):
            return await recover_to_last_safe(
                drone,
                target_down,
                str(error),
                prefer_latest=False,
            )
        raise

    c = monitor.get_directional_clearance()
    front = c["center"]
    safe, reason = clearance_safe_for_motion(
        c,
        allow_corridor=NARROW_CORRIDOR_ENABLED,
    )
    corridor_mode = narrow_corridor_active(c)
    if not corridor_mode:
        reset_corridor_pid()

    if not safe:
        exploration_memory.mark_blocked_ray(
            latest_position_ned.north_m,
            latest_position_ned.east_m,
            latest_attitude["yaw"],
            front,
        )

        print(
            f"⚠️ [{label}] blocked {reason} "
            f"front={front:.2f}m "
            f"left={c['left']:.2f}m right={c['right']:.2f}m "
            f"blocked_streak={blocked_streak + 1}"
        )

        if blocked_streak >= MAX_BLOCKED_STREAK:
            print("🧭 Too many blocked states. Recovering before selecting a new heading.")
            blocked_streak = 0

        log_mission_action(
            action_type="BLOCKED_RECOVER",
            label=label,
            target_down=target_down,
            preferred_heading=preferred_heading_deg,
            selected_heading=latest_attitude["yaw"],
            heading_score=heading_score,
            extra={
                "front_m": front,
                "left_m": c["left"],
                "right_m": c["right"],
                "lower_center_m": c["lower_center"],
                "blocked_reason": reason,
                "turn_decision": "recover_to_last_safe",
            },
        )

        return await recover_to_last_safe(
            drone,
            target_down,
            f"post-yaw unsafe {reason}",
            prefer_latest=False,
        )

    blocked_streak = 0

    command_heading = normalize_angle_deg(selected_heading)
    corridor_steer_deg = 0.0

    if corridor_mode:
        corridor_steer_deg = corridor_steer_adjustment_deg(c)

        if abs(corridor_steer_deg) > 1.0:
            command_heading = normalize_angle_deg(command_heading + corridor_steer_deg)

            print(
                f"🧵 [{label}] corridor blend: "
                f"left={c['left']:.2f} right={c['right']:.2f} "
                f"steer={corridor_steer_deg:+.1f} yaw={command_heading:.1f}"
            )

            try:
                await set_yaw(drone, command_heading, target_down, duration_s=0.2)
            except RuntimeError as error:
                if str(error).startswith("yaw_blocked:"):
                    return await recover_to_last_safe(
                        drone,
                        target_down,
                        str(error),
                        prefer_latest=False,
                    )
                raise

            c = monitor.get_directional_clearance()
            front = c["center"]
            safe, reason = clearance_safe_for_motion(
                c,
                allow_corridor=NARROW_CORRIDOR_ENABLED,
            )
            corridor_mode = narrow_corridor_active(c)

            if not safe:
                return await recover_to_last_safe(
                    drone,
                    target_down,
                    f"corridor blend unsafe {reason}",
                    prefer_latest=False,
                )

    if corridor_mode:
        step_m = NARROW_CORRIDOR_STEP_M
    elif (
        front >= FAST_OPEN_MIN_FRONT_M
        and min(c["left"], c["right"]) >= FAST_OPEN_MIN_SIDE_M
        and c["lower_center"] >= FAST_OPEN_MIN_LOWER_M
    ):
        step_m = FAST_OPEN_STEP_M
    else:
        step_m = MOVE_STEP_M

    yaw_rad = math.radians(command_heading)

    start_move_n = latest_position_ned.north_m
    start_move_e = latest_position_ned.east_m
    target_n = latest_position_ned.north_m + step_m * math.cos(yaw_rad)
    target_e = latest_position_ned.east_m + step_m * math.sin(yaw_rad)

    debug = exploration_memory.debug_summary()

    print(
        f"➡️ [{label}] move {step_m:.2f}m yaw={command_heading:.1f} "
        f"actual_yaw={latest_attitude['yaw']:.1f} "
        f"range={distance_from_start():.1f} front={front:.2f} "
        f"lower={c['lower_center']:.2f} "
        f"visited={debug['visited_cells']} blocked={debug['blocked_cells']}"
    )

    log_mission_action(
        action_type="MOVE_FORWARD",
        label=label,
        target_down=target_down,
        preferred_heading=preferred_heading_deg,
        selected_heading=command_heading,
        heading_score=heading_score,
        extra={
            "move_step_m": step_m,
            "front_m": front,
            "left_m": c["left"],
            "right_m": c["right"],
            "lower_center_m": c["lower_center"],
            "corridor_mode": corridor_mode,
            "corridor_steer_deg": corridor_steer_deg,
        },
    )

    moved = await move_to_position_step(
        drone,
        target_n,
        target_e,
        target_down,
        command_heading,
        label=label,
        allow_corridor=NARROW_CORRIDOR_ENABLED,
    )

    if moved and latest_position_ned is not None:
        return_fail_streak = 0
        mark_path_segment_visited(
            start_move_n,
            start_move_e,
            latest_position_ned.north_m,
            latest_position_ned.east_m,
        )

    return moved


# ============================================================
# Mission logic
# ============================================================

async def exploration_pass(
    drone,
    target_down,
    pass_name,
    relative_headings,
    steps_per_heading,
    allowed_colours,
):
    global active_allowed_colours
    global active_target_alt_d

    active_allowed_colours = allowed_colours
    active_target_alt_d = target_down

    print(f"\n===== {pass_name} alt_d={target_down:.1f}, targets={allowed_colours} =====")

    await hold_position(drone, target_down, duration_s=1.0)
    await scan_current_view(label=f"{pass_name}_start", frames=SCAN_FRAMES_PER_VIEW)

    failed_headings = 0

    for heading_index, rel_heading in enumerate(relative_headings):
        if not search_time_remaining():
            break

        heading = normalize_angle_deg(start_yaw + rel_heading)

        print(f"🧭 {pass_name}: preferred heading {heading:.1f} deg")

        failed_steps = 0

        for step in range(steps_per_heading):
            if not search_time_remaining():
                break

            moved = await move_in_heading(
                drone,
                heading,
                target_down,
                label=f"{pass_name}_{heading_index:02d}_{step:02d}",
            )

            if moved:
                failed_steps = 0
                if (
                    active_investigation_enabled
                    and (
                        step % MOVE_STOP_SCAN_INTERVAL_STEPS == 0
                        or new_candidate_event.is_set()
                    )
                ):
                    await scan_current_view(
                        label=f"{pass_name}_{heading_index:02d}_{step:02d}_stop",
                        frames=MOVE_STOP_SCAN_FRAMES,
                    )
            else:
                failed_steps += 1

            await handle_candidate_event(drone, target_down)

            if (
                active_investigation_enabled
                and not moved
                and blocked_streak >= 3
            ):
                await investigate_candidate(drone, target_down)

            while (
                distance_from_start() > SOFT_RANGE_LIMIT_M
                and search_time_remaining()
            ):
                await return_home_step(drone, target_down)

            if failed_steps >= MAX_FAILED_STEPS_PER_HEADING:
                failed_headings += 1
                print(
                    f"🧭 {pass_name}: skipping blocked heading after "
                    f"{failed_steps} failed moves."
                )
                break

        if failed_steps < MAX_FAILED_STEPS_PER_HEADING:
            failed_headings = 0

        if failed_headings >= MAX_FAILED_HEADINGS_PER_PASS:
            print(
                f"🧭 {pass_name}: local pocket is too constrained; "
                "retreating toward open space before the next pass."
            )

            for _ in range(3):
                if distance_from_start() <= 1.0 or not search_time_remaining():
                    break

                moved_home = await return_home_step(drone, target_down)

                if not moved_home:
                    await recover_to_last_safe(
                        drone,
                        target_down,
                        "breadth escape after repeated blocked headings",
                    )
                    break

            break

    return target_memory.summary()


async def full_world_sweep(drone):
    sweep_index = 0

    while search_time_remaining():
        low_rotation = sweep_index * SWEEP_HEADING_ROTATION_DEG
        high_rotation = low_rotation + 30.0

        await exploration_pass(
            drone,
            target_down=LOW_SCAN_ALT_D,
            pass_name=f"SWEEP_LOW_BOTH_{sweep_index:02d}",
            relative_headings=[
                normalize_angle_deg(heading + low_rotation)
                for heading in SWEEP_HEADINGS_DEG
            ],
            steps_per_heading=SWEEP_STEPS_PER_HEADING,
            allowed_colours=("yellow", "red"),
        )

        if eligibility_met(target_memory.summary()) and not EXTRA_SCORING_AFTER_ELIGIBILITY:
            print(
                "===== EXTRA_SWEEP_STOPPED =====\n"
                "Eligibility reached during the low sweep. Landing instead of "
                "starting another risky scoring pass."
            )
            break

        if not search_time_remaining():
            break

        await exploration_pass(
            drone,
            target_down=HIGH_SCAN_ALT_D,
            pass_name=f"SWEEP_HIGH_BOTH_{sweep_index:02d}",
            relative_headings=[
                normalize_angle_deg(heading + high_rotation)
                for heading in SWEEP_HEADINGS_DEG
            ],
            steps_per_heading=SWEEP_STEPS_PER_HEADING,
            allowed_colours=("yellow", "red"),
        )

        if eligibility_met(target_memory.summary()) and not EXTRA_SCORING_AFTER_ELIGIBILITY:
            print(
                "===== EXTRA_SWEEP_STOPPED =====\n"
                "Eligibility reached during the high sweep. Landing instead of "
                "continuing extra scoring cycles."
            )
            break

        sweep_index += 1

        if sweep_index >= MAX_FULL_SWEEP_CYCLES:
            break


def clearance_for_relative_heading(clearances, relative_heading_deg):
    relative_heading_deg = normalize_angle_deg(relative_heading_deg)

    if abs(relative_heading_deg) <= 30.0:
        return clearances["center"]

    if relative_heading_deg > 0:
        return clearances["right"]

    return clearances["left"]


def choose_frontier_heading(preferred_heading_deg=None):
    if latest_position_ned is None:
        return latest_attitude["yaw"], 0.0

    clearances = monitor.get_directional_clearance()
    current_yaw = latest_attitude["yaw"]

    offsets = [0, 15, -15, 30, -30, 50, -50, 75, -75, 105, -105, 140, -140, 180]
    candidates = [normalize_angle_deg(current_yaw + offset) for offset in offsets]

    if preferred_heading_deg is not None:
        candidates.extend(
            normalize_angle_deg(preferred_heading_deg + offset)
            for offset in [0, 20, -20, 40, -40]
        )

    if distance_from_start() > RESUME_RANGE_M:
        home_heading = heading_to_start_deg()
        candidates.extend(
            normalize_angle_deg(home_heading + offset)
            for offset in [0, 20, -20, 45, -45]
        )

    best_heading = current_yaw
    best_score = -1e9
    seen = set()
    ranked = []

    for heading in candidates:
        heading_key = round(heading, 1)

        if heading_key in seen:
            continue

        seen.add(heading_key)

        turn_delta = angle_diff_deg(heading, current_yaw)
        relative_heading = normalize_angle_deg(heading - current_yaw)
        direction_clearance = clearance_for_relative_heading(clearances, relative_heading)
        side_clearance = min(clearances["left"], clearances["right"])
        corridor_heading = (
            NARROW_CORRIDOR_ENABLED
            and abs(relative_heading) <= 90.0
            and narrow_corridor_passable(clearances)
        )

        if direction_clearance < FRONTIER_MIN_DIRECTION_CLEARANCE_M and not corridor_heading:
            continue

        if (
            turn_delta > 55.0
            and side_clearance < FRONTIER_TIGHT_TURN_SIDE_CLEARANCE_M
            and not corridor_heading
        ):
            continue

        score = exploration_memory.heading_score(
            current_n=latest_position_ned.north_m,
            current_e=latest_position_ned.east_m,
            candidate_heading_deg=heading,
            current_yaw_deg=current_yaw,
            preferred_heading_deg=heading,
            clearances=clearances,
        )

        projected_score = 0.0
        projected_debug = {"new_cells": 0, "revisits": 0, "blocked": 0}

        if FRONTIER_DELIBERATION_ENABLED:
            projected_score, projected_debug = exploration_memory.projected_path_score(
                current_n=latest_position_ned.north_m,
                current_e=latest_position_ned.east_m,
                candidate_heading_deg=heading,
                step_m=FRONTIER_LOOKAHEAD_STEP_M,
                steps=FRONTIER_LOOKAHEAD_STEPS,
            )
            score += projected_score

        score += min(direction_clearance, 10.0) * 2.2
        score -= 0.035 * turn_delta

        if corridor_heading and side_clearance < MIN_SIDE_MOVE_CLEARANCE_M:
            score += NARROW_CORRIDOR_FRONTIER_BONUS
            score -= abs(corridor_steer_estimate_deg(clearances)) * 0.08

        if preferred_heading_deg is not None:
            preferred_delta = angle_diff_deg(heading, preferred_heading_deg)
            score -= 0.08 * preferred_delta

            if preferred_delta <= 35.0:
                score += 6.0

        current_range = distance_from_start()
        deep_mode = deep_exploration_active()

        if current_range > RESUME_RANGE_M:
            home_bias_scale = 1.0
            if deep_mode and current_range < DEEP_EXPLORATION_TARGET_RANGE_M:
                home_bias_scale = DEEP_EXPLORATION_HOME_BIAS_SCALE
            score -= home_bias_scale * 0.05 * angle_diff_deg(heading, heading_to_start_deg())
        elif current_range < 4.0:
            away_heading = normalize_angle_deg(
                math.degrees(
                    math.atan2(
                        latest_position_ned.east_m - start_e,
                        latest_position_ned.north_m - start_n,
                    )
                )
            )
            score -= 0.015 * angle_diff_deg(heading, away_heading)

        heading_rad = math.radians(heading)
        projected_n = latest_position_ned.north_m + (
            FRONTIER_LOOKAHEAD_STEP_M * math.cos(heading_rad)
        )
        projected_e = latest_position_ned.east_m + (
            FRONTIER_LOOKAHEAD_STEP_M * math.sin(heading_rad)
        )
        projected_range = math.hypot(projected_n - start_n, projected_e - start_e)
        range_gain = projected_range - current_range

        if current_range < SOFT_RANGE_LIMIT_M - 1.0:
            range_bonus = 1.5
            if deep_mode and current_range < DEEP_EXPLORATION_TARGET_RANGE_M:
                range_bonus = DEEP_EXPLORATION_RANGE_BONUS
            score += clamp(range_gain, -1.0, 2.5) * range_bonus
        else:
            score -= max(0.0, range_gain) * 6.0

        ranked.append(
            {
                "heading": heading,
                "score": score,
                "turn": turn_delta,
                "clearance": direction_clearance,
                "new_cells": projected_debug["new_cells"],
                "revisits": projected_debug["revisits"],
                "blocked": projected_debug["blocked"],
                "range_gain": range_gain,
            }
        )

        if score > best_score:
            best_score = score
            best_heading = heading

    if FRONTIER_DELIBERATION_LOG and ranked:
        top = sorted(ranked, key=lambda item: item["score"], reverse=True)[
            :FRONTIER_BEAM_TOP_K
        ]
        summary = " | ".join(
            (
                f"{item['heading']:.0f}deg score={item['score']:.1f} "
                f"new={item['new_cells']} clr={item['clearance']:.1f} "
                f"turn={item['turn']:.0f}"
            )
            for item in top
        )
        print(f"🧮 frontier deliberation: {summary}")

    if best_score <= -1e8:
        side_escape = 70.0 if clearances["right"] >= clearances["left"] else -70.0
        best_heading = normalize_angle_deg(current_yaw + side_escape)
        best_score = -250.0

    return normalize_angle_deg(best_heading), best_score


async def frontier_stride(
    drone,
    target_down,
    label,
    steps_per_stride,
    preferred_heading=None,
):
    moved_steps = 0
    failed_steps = 0

    for step in range(steps_per_stride):
        if not search_time_remaining():
            break

        if distance_from_start() > SOFT_RANGE_LIMIT_M:
            moved_home = await return_home_step(drone, target_down)
            if not moved_home:
                break
            continue

        heading, heading_score = choose_frontier_heading(
            preferred_heading_deg=preferred_heading
        )

        print(
            f"🧭 [{label}_{step:02d}] frontier heading "
            f"{heading:.1f} score={heading_score:.1f}"
        )

        moved = await move_in_heading(
            drone,
            heading,
            target_down,
            label=f"{label}_{step:02d}",
            allow_memory_nudge=False,
        )

        await handle_candidate_event(drone, target_down)

        if moved:
            moved_steps += 1
            failed_steps = 0
            preferred_heading = heading

            if (
                active_investigation_enabled
                and (
                    step % MOVE_STOP_SCAN_INTERVAL_STEPS == 0
                    or new_candidate_event.is_set()
                )
            ):
                await scan_current_view(
                    label=f"{label}_{step:02d}_stop",
                    frames=MOVE_STOP_SCAN_FRAMES,
                )
        else:
            failed_steps += 1

        if failed_steps >= MAX_FAILED_STEPS_PER_GOAL:
            break

    return moved_steps


async def recenter_for_new_sector(drone, target_down, label):
    global return_fail_streak
    return_fail_streak = 0
    moved_steps = 0
    sector_resume_range = max(FRONTIER_SECTOR_RESUME_RANGE_M, RESUME_RANGE_M)

    while (
        search_time_remaining()
        and distance_from_start() > sector_resume_range
        and moved_steps < FRONTIER_RECENTER_STEPS
    ):
        moved = await return_home_step(drone, target_down)

        if not moved:
            await recover_to_last_safe(
                drone,
                target_down,
                f"{label} recenter blocked",
                prefer_latest=False,
            )
            break

        moved_steps += 1

    return moved_steps


async def open_space_escape(drone, target_down, label):
    print(f"🧭 [{label}] open-space escape stride")

    preferred_heading = None

    if distance_from_start() > 4.0:
        preferred_heading = heading_to_start_deg()

    return await frontier_stride(
        drone,
        target_down,
        label=f"{label}_ESCAPE",
        steps_per_stride=FRONTIER_ESCAPE_STEPS,
        preferred_heading=preferred_heading,
    )


async def frontier_coverage_pass(
    drone,
    target_down,
    pass_name,
    stride_count,
    steps_per_stride,
    allowed_colours=("yellow", "red"),
    investigate=False,
    macro_headings=None,
    stop_on_eligibility=True,
):
    global active_allowed_colours
    global active_target_alt_d
    global active_investigation_enabled

    active_allowed_colours = tuple(allowed_colours)
    active_target_alt_d = target_down
    active_investigation_enabled = investigate
    new_candidate_event.clear()

    print(
        f"\n===== {pass_name} FRONTIER alt_d={target_down:.1f}, "
        f"strides={stride_count}, steps={steps_per_stride}, "
        f"targets={active_allowed_colours}, investigate={investigate} ====="
    )

    await hold_position(drone, target_down, duration_s=0.5)

    if investigate:
        await scan_current_view(label=f"{pass_name}_start", frames=MOVE_STOP_SCAN_FRAMES)

    blocked_strides = 0
    macro_headings = macro_headings or FRONTIER_MACRO_HEADINGS_DEG
    previous_macro_heading = None

    for stride_index in range(stride_count):
        if not search_time_remaining():
            break

        macro_heading = None

        if macro_headings:
            macro_heading = normalize_angle_deg(
                start_yaw + macro_headings[stride_index % len(macro_headings)]
            )

            if (
                previous_macro_heading is not None
                and distance_from_start() > FRONTIER_SECTOR_RESET_RANGE_M
                and angle_diff_deg(macro_heading, previous_macro_heading) > 55.0
            ):
                print(
                    f"🧭 {pass_name}: recentering before sector "
                    f"{stride_index:02d} heading={macro_heading:.1f}"
                )
                await recenter_for_new_sector(
                    drone,
                    target_down,
                    label=f"{pass_name}_{stride_index:02d}",
                )

            previous_macro_heading = macro_heading

            print(
                f"🧭 {pass_name}: macro sector {stride_index:02d} "
                f"heading={macro_heading:.1f}"
            )

        moved_steps = await frontier_stride(
            drone,
            target_down,
            label=f"{pass_name}_{stride_index:02d}",
            steps_per_stride=steps_per_stride,
            preferred_heading=macro_heading,
        )

        if moved_steps > 0:
            blocked_strides = 0
        else:
            blocked_strides += 1

        if blocked_strides >= 2:
            escaped_steps = await open_space_escape(
                drone,
                target_down,
                label=f"{pass_name}_{stride_index:02d}",
            )
            blocked_strides = 0 if escaped_steps > 0 else blocked_strides

        if (
            stop_on_eligibility
            and eligibility_met(target_memory.summary())
            and not EXTRA_SCORING_AFTER_ELIGIBILITY
        ):
            break

    return target_memory.summary()


def global_coverage_goals(rotation_deg=0.0):
    goals = []

    for rel_heading in GLOBAL_COVERAGE_HEADINGS_DEG:
        for ring_m in GLOBAL_COVERAGE_RINGS_M:
            heading = normalize_angle_deg(start_yaw + rel_heading + rotation_deg)
            yaw_rad = math.radians(heading)
            goals.append(
                (
                    start_n + ring_m * math.cos(yaw_rad),
                    start_e + ring_m * math.sin(yaw_rad),
                    ring_m,
                    heading,
                )
            )

    return goals


async def navigate_to_coverage_goal(
    drone,
    goal_n,
    goal_e,
    target_down,
    label,
    max_steps=MAX_GLOBAL_GOAL_STEPS,
):
    failed_steps = 0
    no_progress_steps = 0
    best_goal_distance = distance_to_point_m(goal_n, goal_e)

    for step in range(max_steps):
        if not search_time_remaining():
            break

        goal_distance = distance_to_point_m(goal_n, goal_e)

        if goal_distance <= GLOBAL_GOAL_REACHED_RADIUS_M:
            print(f"🗺️ [{label}] reached sector goal dist={goal_distance:.1f}m")
            return True

        heading = heading_to_point_deg(goal_n, goal_e)

        moved = await move_in_heading(
            drone,
            heading,
            target_down,
            label=f"{label}_{step:02d}",
        )

        if moved:
            failed_steps = 0

            if (
                active_investigation_enabled
                and (
                    step % MOVE_STOP_SCAN_INTERVAL_STEPS == 0
                    or new_candidate_event.is_set()
                )
            ):
                await scan_current_view(
                    label=f"{label}_{step:02d}_stop",
                    frames=MOVE_STOP_SCAN_FRAMES,
                )

            new_goal_distance = distance_to_point_m(goal_n, goal_e)

            if new_goal_distance < best_goal_distance - 0.20:
                best_goal_distance = new_goal_distance
                no_progress_steps = 0
            else:
                no_progress_steps += 1

        else:
            failed_steps += 1
            no_progress_steps += 1

        await handle_candidate_event(drone, target_down)

        if failed_steps >= MAX_FAILED_STEPS_PER_GOAL:
            print(f"🗺️ [{label}] sector blocked after {failed_steps} failed moves.")
            break

        if no_progress_steps >= MAX_FAILED_STEPS_PER_GOAL + 1:
            print(f"🗺️ [{label}] no useful progress; moving to next sector.")
            break

    return False


def candidate_revisit_goals(allowed_colours=("yellow", "red")):
    goals = []
    confirmed_summary = target_memory.summary()
    missing_colours = set()

    if confirmed_summary["yellow"] == 0:
        missing_colours.add("yellow")

    if confirmed_summary["red"] == 0:
        missing_colours.add("red")

    focus_colours = set(allowed_colours)

    if missing_colours and not EXTRA_SCORING_AFTER_ELIGIBILITY:
        focus_colours &= missing_colours

    for candidate in confirmed_summary["candidates"]:
        if candidate["colour"] not in focus_colours:
            continue

        if not candidate.get("target_n_list") or not candidate.get("target_e_list"):
            continue

        target_n = sum(candidate["target_n_list"]) / len(candidate["target_n_list"])
        target_e = sum(candidate["target_e_list"]) / len(candidate["target_e_list"])
        bearing_from_start = math.atan2(target_e - start_e, target_n - start_n)
        visit_n = target_n - CANDIDATE_REVISIT_STANDOFF_M * math.cos(bearing_from_start)
        visit_e = target_e - CANDIDATE_REVISIT_STANDOFF_M * math.sin(bearing_from_start)

        yaw_span_bonus = min(
            len(set(round(yaw / 10.0) for yaw in candidate.get("observer_yaws", []))),
            3,
        )
        score = (
            candidate.get("count", 0) * 2.0
            + candidate.get("confidence", 0.0) * 3.0
            + yaw_span_bonus
        )

        if candidate["colour"] in missing_colours:
            score += 8.0

        goals.append(
            {
                "colour": candidate["colour"],
                "score": score,
                "target_n": target_n,
                "target_e": target_e,
                "visit_n": visit_n,
                "visit_e": visit_e,
                "count": candidate.get("count", 0),
                "confidence": candidate.get("confidence", 0.0),
            }
        )

    goals.sort(key=lambda goal: goal["score"], reverse=True)
    return goals[:MAX_CANDIDATE_REVISITS]


async def revisit_candidate_waypoints(drone):
    global active_allowed_colours
    global active_target_alt_d
    global active_investigation_enabled

    previous_allowed_colours = active_allowed_colours
    active_investigation_enabled = True
    active_allowed_colours = ("yellow", "red")

    goals = candidate_revisit_goals(allowed_colours=active_allowed_colours)

    if not goals:
        print("\n===== CANDIDATE_REVISIT skipped: no stored candidate waypoints =====")
        active_allowed_colours = previous_allowed_colours
        return

    print(f"\n===== CANDIDATE_REVISIT waypoints={len(goals)} =====")

    for index, goal in enumerate(goals):
        if not search_time_remaining():
            break

        target_down = HIGH_SCAN_ALT_D if goal["colour"] == "red" else LOW_SCAN_ALT_D
        active_target_alt_d = target_down
        active_allowed_colours = (goal["colour"],)
        new_candidate_event.clear()

        print(
            f"🎯 REVISIT {index:02d}: {goal['colour']} "
            f"count={goal['count']} conf={goal['confidence']:.2f} "
            f"target_N={goal['target_n']:.1f} target_E={goal['target_e']:.1f}"
        )

        await hold_position(drone, target_down, duration_s=0.4)
        await navigate_to_coverage_goal(
            drone,
            goal["visit_n"],
            goal["visit_e"],
            target_down,
            label=f"REVISIT_{index:02d}_{goal['colour']}",
            max_steps=MAX_REVISIT_GOAL_STEPS,
        )

        if not search_time_remaining():
            break

        yaw_to_target = heading_to_point_deg(goal["target_n"], goal["target_e"])

        await drone.offboard.set_position_ned(
            PositionNedYaw(
                latest_position_ned.north_m,
                latest_position_ned.east_m,
                target_down,
                yaw_to_target,
            )
        )
        await asyncio.sleep(INVESTIGATION_SETTLE_S)
        await scan_current_view(
            label=f"REVISIT_{index:02d}_{goal['colour']}_face",
            frames=SCAN_FRAMES_PER_VIEW,
        )
        await investigate_candidate(drone, target_down)

        if eligibility_met(target_memory.summary()) and not EXTRA_SCORING_AFTER_ELIGIBILITY:
            break

    active_allowed_colours = previous_allowed_colours


async def global_coverage_sweep(drone, defer_investigation=False):
    global active_allowed_colours
    global active_target_alt_d
    global active_investigation_enabled

    previous_investigation_enabled = active_investigation_enabled

    if not USE_FIXED_RING_COVERAGE:
        print("\n===== GLOBAL_FRONTIER_COVERAGE_SWEEP =====")

        try:
            if search_time_remaining():
                await frontier_coverage_pass(
                    drone,
                    target_down=LOW_SCAN_ALT_D,
                    pass_name="FRONTIER_LOW_BOTH_00",
                    stride_count=FRONTIER_LOW_STRIDES,
                    steps_per_stride=FRONTIER_STEPS_PER_STRIDE,
                    allowed_colours=("yellow", "red"),
                    investigate=not defer_investigation,
                    macro_headings=FRONTIER_MACRO_HEADINGS_DEG,
                    stop_on_eligibility=False,
                )

            if search_time_remaining():
                await frontier_coverage_pass(
                    drone,
                    target_down=HIGH_SCAN_ALT_D,
                    pass_name="FRONTIER_HIGH_BOTH_00",
                    stride_count=FRONTIER_HIGH_STRIDES,
                    steps_per_stride=FRONTIER_STEPS_PER_STRIDE,
                    allowed_colours=("yellow", "red"),
                    investigate=not defer_investigation,
                    macro_headings=[
                        normalize_angle_deg(heading + 22.5)
                        for heading in FRONTIER_MACRO_HEADINGS_DEG
                    ],
                    stop_on_eligibility=False,
                )

        finally:
            active_investigation_enabled = previous_investigation_enabled

        return

    print("\n===== GLOBAL_RING_COVERAGE_SWEEP =====")

    active_investigation_enabled = not defer_investigation

    coverage_passes = [
        (LOW_SCAN_ALT_D, "GLOBAL_LOW_BOTH_00", 0.0),
        (HIGH_SCAN_ALT_D, "GLOBAL_HIGH_BOTH_00", 22.5),
    ]

    try:
        for target_down, pass_name, rotation in coverage_passes:
            if not search_time_remaining():
                break

            active_allowed_colours = ("yellow", "red")
            active_target_alt_d = target_down

            print(
                f"\n===== {pass_name} alt_d={target_down:.1f}, "
                f"rings={GLOBAL_COVERAGE_RINGS_M} "
                f"defer_investigation={defer_investigation} ====="
            )

            await hold_position(drone, target_down, duration_s=0.8)

            if active_investigation_enabled:
                await scan_current_view(
                    label=f"{pass_name}_start",
                    frames=SCAN_FRAMES_PER_VIEW,
                )

            goals = global_coverage_goals(rotation_deg=rotation)

            for goal_index, (goal_n, goal_e, ring_m, heading) in enumerate(goals):
                if not search_time_remaining():
                    break

                if distance_from_start() > SOFT_RANGE_LIMIT_M:
                    await return_home_step(drone, target_down)
                    continue

                print(
                    f"🗺️ {pass_name}: sector {goal_index:02d} "
                    f"ring={ring_m:.1f} heading={heading:.1f} "
                    f"goal_N={goal_n:.1f} goal_E={goal_e:.1f}"
                )

                await navigate_to_coverage_goal(
                    drone,
                    goal_n,
                    goal_e,
                    target_down,
                    label=f"{pass_name}_{goal_index:02d}",
                )

                await handle_candidate_event(drone, target_down)

    finally:
        active_investigation_enabled = previous_investigation_enabled


async def main():
    global mission_start_time
    global start_n
    global start_e
    global start_yaw
    global mission_should_stop
    global camera_photo_saver
    global active_allowed_colours
    global active_target_alt_d
    global active_investigation_enabled

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    await wait_for_connection(drone)

    image_topic = find_image_topic()
    print(f"Using image topic: {image_topic}")
    print(f"Using depth topic: {DEPTH_TOPIC}")

    ros2_sensor_bridge = None

    if USE_ROS2_SENSOR_BRIDGE:
        if ROS2_AVAILABLE:
            ros2_sensor_bridge = Ros2SensorBridge(
                ROS2_IMAGE_TOPIC,
                ROS2_DEPTH_TOPIC,
                update_latest_frame_bgr,
                update_latest_depth,
            )

            if ros2_sensor_bridge.start():
                print(
                    "Using ROS2 sensor bridge: "
                    f"image={ROS2_IMAGE_TOPIC}, depth={ROS2_DEPTH_TOPIC}"
                )
            else:
                ros2_sensor_bridge = None
                print("ROS2 sensor bridge unavailable. Falling back to Gazebo transport.")
        else:
            print("rclpy/sensor_msgs unavailable. Falling back to Gazebo transport.")

    if ros2_sensor_bridge is None:
        node = Node()
        node.subscribe(Image, image_topic, image_callback)
        node.subscribe(Image, DEPTH_TOPIC, depth_callback)
        print("Using Gazebo transport for RGB/depth sensor topics.")

    camera_photo_saver = GZPhotoDetectorSaver(
        topic=image_topic,
        save_dir=PHOTO_BURST_DIR,
        model_path=YOLO_MODEL_PATH,
        burst_size=STOP_CAPTURE_BURST_FRAMES,
        threshold=DETECTION_CONFIDENCE_THRESHOLD,
        enable_yolo=YOLO_BURST_ENABLED,
        imgsz=YOLO_IMGSZ,
        device=YOLO_DEVICE,
    )
    initialize_mission_yolo_model()
    camera_task = asyncio.create_task(camera_photo_saver.run())

    asyncio.create_task(telemetry_task(drone))

    await wait_for_local_position(drone)
    await wait_for_telemetry()
    await wait_for_camera_depth(timeout_s=8.0)

    print("Arming & takeoff...")

    armed = await arm_with_retry(drone)
    if not armed:
        print("Mission aborted: arming failed.")
        if ros2_sensor_bridge is not None:
            ros2_sensor_bridge.stop()
        camera_photo_saver.running = False
        camera_task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await camera_task
        return

    await drone.action.set_takeoff_altitude(abs(DEFAULT_ALT_D))
    await drone.action.takeoff()
    await asyncio.sleep(8)

    await wait_for_telemetry()

    start_n = latest_position_ned.north_m
    start_e = latest_position_ned.east_m
    start_yaw = latest_attitude["yaw"]

    exploration_memory.initialize(start_n, start_e)
    exploration_memory.mark_visited(start_n, start_e)
    remember_safe_position()

    mission_start_time = time.time()

    try:
        await prime_and_start_offboard(drone, DEFAULT_ALT_D)

    except OffboardError as error:
        print(f"Offboard start failed: {error}. Landing.")
        if ros2_sensor_bridge is not None:
            ros2_sensor_bridge.stop()
        camera_photo_saver.running = False
        camera_task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await camera_task
        await drone.action.land()
        return

    teleop_transport = None
    teleop_task = None

    if TELEOP_ENABLED:
        teleop_transport, _ = await start_teleop_udp_listener()
        teleop_task = asyncio.create_task(teleop_override_task(drone))

    print("\n===== GNSS-FREE X500_VISION QUALIFIER MISSION STARTED =====")
    print("Mode: fast open-frontier coverage first, candidate revisit second")
    print("A colour blob is remembered during coverage, then actively investigated later.")
    print("Exploration: open headings and local visited-path memory prefer new ground.")
    print(
        "Deep coverage profile: "
        f"soft={SOFT_RANGE_LIMIT_M:.1f}m resume={RESUME_RANGE_M:.1f}m "
        f"hard={HARD_RANGE_LIMIT_M:.1f}m fast_step={FAST_OPEN_STEP_M:.2f}m "
        f"post_strides={FRONTIER_POST_ELIGIBILITY_STRIDES}\n"
    )

    perception = asyncio.create_task(perception_task())

    safety_reason = "mission complete"

    try:
        active_allowed_colours = ("yellow", "red")
        active_target_alt_d = LOW_SCAN_ALT_D
        active_investigation_enabled = False

        summary = target_memory.summary()

        if search_time_remaining():
            await global_coverage_sweep(drone, defer_investigation=True)

        summary = target_memory.summary()

        if search_time_remaining():
            active_investigation_enabled = True
            await revisit_candidate_waypoints(drone)

        summary = target_memory.summary()

        if not eligibility_met(summary) and search_time_remaining():
            missing = missing_colours(summary)

            print(f"\n===== FRONTIER_FALLBACK_SEARCH missing={missing} =====")

            for colour in missing:
                if not search_time_remaining():
                    break

                await frontier_coverage_pass(
                    drone,
                    target_down=HIGH_SCAN_ALT_D if colour == "red" else LOW_SCAN_ALT_D,
                    pass_name=f"FRONTIER_{colour.upper()}_SEARCH",
                    stride_count=FRONTIER_SEARCH_STRIDES,
                    steps_per_stride=FRONTIER_SEARCH_STEPS_PER_STRIDE,
                    allowed_colours=(colour,),
                    investigate=True,
                    macro_headings=FRONTIER_MACRO_HEADINGS_DEG,
                    stop_on_eligibility=True,
                )

                if eligibility_met(target_memory.summary()):
                    break

        summary = target_memory.summary()

        if search_time_remaining():
            if eligibility_met(summary) and not EXTRA_SCORING_AFTER_ELIGIBILITY:
                if CONTINUE_FRONTIER_AFTER_ELIGIBILITY:
                    print(
                        "\n===== POST_ELIGIBILITY_FRONTIER_EXPLORE =====\n"
                        "Eligibility is met. Continuing passive frontier coverage "
                        "to map more of RoboVerse."
                    )

                    cycle = 0

                    while search_time_remaining():
                        rotation = cycle * 22.5
                        macro_headings = [
                            normalize_angle_deg(heading + rotation)
                            for heading in FRONTIER_MACRO_HEADINGS_DEG
                        ]

                        await frontier_coverage_pass(
                            drone,
                            target_down=LOW_SCAN_ALT_D if cycle % 2 == 0 else HIGH_SCAN_ALT_D,
                            pass_name=f"FRONTIER_WORLD_EXPLORE_{cycle:02d}",
                            stride_count=FRONTIER_POST_ELIGIBILITY_STRIDES,
                            steps_per_stride=FRONTIER_SEARCH_STEPS_PER_STRIDE,
                            allowed_colours=("yellow", "red"),
                            investigate=False,
                            macro_headings=macro_headings,
                            stop_on_eligibility=False,
                        )

                        cycle += 1
                else:
                    print(
                        "\n===== EXTRA_SWEEP_SKIPPED =====\n"
                        "Eligibility is met. Landing is preferred over risking a late "
                        "critical-state crash. Set EXTRA_SCORING_AFTER_ELIGIBILITY=1 "
                        "to opt into the old full-world scoring sweep."
                    )
            elif not EXTRA_SCORING_AFTER_ELIGIBILITY:
                print("\n===== FRONTIER_SCORE_SEARCH =====")

                while search_time_remaining() and not eligibility_met(target_memory.summary()):
                    for colour in missing_colours(target_memory.summary()):
                        if not search_time_remaining():
                            break

                        await frontier_coverage_pass(
                            drone,
                            target_down=HIGH_SCAN_ALT_D if colour == "red" else LOW_SCAN_ALT_D,
                            pass_name=f"FRONTIER_SCORE_{colour.upper()}",
                            stride_count=2,
                            steps_per_stride=FRONTIER_SEARCH_STEPS_PER_STRIDE,
                            allowed_colours=(colour,),
                            investigate=True,
                            macro_headings=FRONTIER_MACRO_HEADINGS_DEG,
                            stop_on_eligibility=True,
                        )

                        if eligibility_met(target_memory.summary()):
                            break
            else:
                print("\n===== FULL_WORLD_SCORE_SWEEP =====")
                active_investigation_enabled = True
                await full_world_sweep(drone)

        summary = target_memory.summary()
        memory_debug = exploration_memory.debug_summary()

        print("\n==============================")
        print("MISSION SUMMARY")
        print(f"Red: {summary['red']}")
        print(f"Yellow: {summary['yellow']}")
        print(f"Total confirmed: {summary['total']}")
        print(f"Score: {get_score(summary)}")
        print(f"Eligibility met: {eligibility_met(summary)}")
        print(f"Unconfirmed candidates left: {len(summary['candidates'])}")
        print(f"Visited cells: {memory_debug['visited_cells']}")
        print(f"Blocked cells: {memory_debug['blocked_cells']}")
        print("==============================")

        if not search_time_remaining():
            safety_reason = "time budget reached"
        elif eligibility_met(summary):
            safety_reason = "full-world sweep complete with eligibility"
        else:
            safety_reason = "mission complete"

    except RuntimeError as error:
        safety_reason = str(error)
        print(f"Safety stop: {safety_reason}")

    finally:
        mission_should_stop = True
        if ros2_sensor_bridge is not None:
            ros2_sensor_bridge.stop()
        if teleop_transport is not None:
            teleop_transport.close()
        if teleop_task is not None:
            teleop_task.cancel()
        perception.cancel()
        camera_photo_saver.running = False
        camera_task.cancel()

        with contextlib.suppress(asyncio.CancelledError):
            await perception

        if teleop_task is not None:
            with contextlib.suppress(asyncio.CancelledError):
                await teleop_task

        with contextlib.suppress(asyncio.CancelledError):
            await camera_task

        mission_logger.save()

        await stop_and_land(drone, safety_reason)



if __name__ == "__main__":
    asyncio.run(main())
