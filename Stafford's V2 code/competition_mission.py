import asyncio
import math
import os
import subprocess
import time
from collections import deque
from datetime import datetime

import grpc
import cv2
import numpy as np

import contextlib
from mission_logger import MissionLogger

os.environ.setdefault("YOLO_CONFIG_DIR", "/tmp/Ultralytics")

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

from obstacle_monitor import ObstacleMonitor
from small_fuel_detector import detect_small_fuel_barrels
from target_memory import TargetMemory, normalize_angle_deg
from exploration_memory import ExplorationMemory
from gzphotodetectorsaver import GZPhotoDetectorSaver
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
MOVE_STEP_M = 0.30
RETURN_STEP_M = 0.45
MOVE_TIMEOUT_S = 1.2
MOVE_REACHED_RADIUS_M = 0.10
CORRIDOR_MOVE_TIMEOUT_S = 0.75

# Local exploration with return-home guard. Keep inside the arena margin.
SOFT_RANGE_LIMIT_M = 17.5
HARD_RANGE_LIMIT_M = 22.0
RESUME_RANGE_M = 13.5

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
FRONTIER_POST_ELIGIBILITY_STRIDES = 4
FRONTIER_SECTOR_RESET_RANGE_M = 10.0
FRONTIER_SECTOR_RESUME_RANGE_M = 7.0
FRONTIER_RECENTER_STEPS = 7
FRONTIER_MIN_DIRECTION_CLEARANCE_M = 2.05
FRONTIER_TIGHT_TURN_SIDE_CLEARANCE_M = 1.30
BLIND_TIGHT_TURN_SIDE_CLEARANCE_M = 1.50
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

# Continuous perception frequency
PERCEPTION_PERIOD_S = 0.20
STOP_CAPTURE_BURST_FRAMES = 1
CONFIRM_CAPTURE_BURST_FRAMES = 10
MIN_PHOTO_BURST_INTERVAL_S = 2.0
MIN_YOLO_BURST_INTERVAL_S = 6.0
MOVE_STOP_SCAN_INTERVAL_STEPS = 2
MOVE_STOP_SCAN_FRAMES = 1

# Investigation
INVESTIGATION_YAW_OFFSETS_DEG = [-16, -8, 0, 8, 16]
INVESTIGATION_SETTLE_S = 0.45
INVESTIGATE_COOLDOWN_S = 3.0
SCAN_FRAMES_PER_VIEW = 3
SCAN_FRAME_INTERVAL_S = 0.12
YAW_SETTLE_TOLERANCE_DEG = 8.0
YAW_SETTLE_TIMEOUT_S = 1.4

# Obstacle / safety
MAX_ATTITUDE_DEG = 5.0
MID_STEP_MAX_ATTITUDE_DEG = 6.5
CRITICAL_ATTITUDE_DEG = 10.0
RECOVERY_ATTITUDE_TOLERANCE_DEG = 15.0
RECOVERY_HARD_LIMIT_DEG = 35.0
MIN_FRONT_MOVE_CLEARANCE_M = 2.00
MIN_SIDE_MOVE_CLEARANCE_M = 1.65
MIN_LOWER_MOVE_CLEARANCE_M = 1.05
MID_STEP_ABORT_CLEARANCE_M = 1.80
YAW_MIN_FRONT_CLEARANCE_M = 1.55
YAW_MIN_SIDE_CLEARANCE_M = 1.30
NARROW_CORRIDOR_ENABLED = os.getenv("NARROW_CORRIDOR_ENABLED", "1") == "1"
NARROW_CORRIDOR_MIN_FRONT_M = 2.00
NARROW_CORRIDOR_MIN_SIDE_M = 1.30
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
    obstacle_distance_m=1.85,
    warning_distance_m=2.75,
    critical_lower_distance_m=MIN_LOWER_MOVE_CLEARANCE_M,
)

target_memory = TargetMemory(
    min_confidence=DETECTION_CONFIDENCE_THRESHOLD,
    min_depth_m=0.35,
    max_depth_m=9.0,
    min_confirm_count=3,
    min_confirm_age_s=0.5,
    min_yaw_span_deg=5.0,
    duplicate_distance_m=1.8,
    duplicate_bearing_deg=12.0,
    stale_candidate_s=MISSION_TIME_LIMIT_S,
)

exploration_memory = ExplorationMemory(
    cell_size_m=1.0,
    lookahead_m=5.0,
    novelty_weight=5.0,
    revisit_penalty=2.2,
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


def remember_safe_position():
    global last_safe_position

    if latest_position_ned is None:
        return

    c = monitor.get_directional_clearance()
    safe, _ = clearance_safe_for_motion(c, allow_corridor=NARROW_CORRIDOR_ENABLED)

    if not safe or not vehicle_stable(active_target_alt_d):
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
    detections, _, _, _ = detect_small_fuel_barrels(frame)

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
    box_colour = (0, 0, 255) if colour == "red" else (0, 255, 255)

    cv2.rectangle(draw, (x, y), (x + w, y + h), box_colour, 3)
    cv2.putText(
        draw,
        f"CONFIRMED {colour} score={get_score(summary)}",
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
    path = os.path.join(EVIDENCE_DIR, f"CONFIRMED_{colour}_{ts}.png")
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
                print(
                    f"🎯 CONFIRMED {confirmed['colour']} | "
                    f"red={summary['red']} yellow={summary['yellow']} "
                    f"score={get_score(summary)}"
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
                    trigger_yolo_detection_burst(
                        num_frames=CONFIRM_CAPTURE_BURST_FRAMES
                    )
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
    Move to a specific local N/E point.

    Used by both exploration and return-home.
    """
    await drone.offboard.set_position_ned(
        PositionNedYaw(
            target_n,
            target_e,
            target_down,
            normalize_angle_deg(yaw_deg),
        )
    )

    start_time = time.time()
    last_guidance_update = start_time
    base_yaw = normalize_angle_deg(yaw_deg)
    timeout_s = MOVE_TIMEOUT_S

    while time.time() - start_time < timeout_s:
        await asyncio.sleep(0.05)

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

        c = monitor.get_directional_clearance()
        safe, reason = clearance_safe_for_motion(c, allow_corridor=allow_corridor)

        if not safe or c["center"] < MID_STEP_ABORT_CLEARANCE_M:
            print(
                f"🚨 [{label}] mid-step abort clearance {reason} "
                f"L={c['left']:.2f} C={c['center']:.2f} R={c['right']:.2f} "
                f"LC={c['lower_center']:.2f}"
            )
            exploration_memory.mark_blocked_ray(
                latest_position_ned.north_m,
                latest_position_ned.east_m,
                latest_attitude["yaw"],
                min(c["center"], c["lower_center"]),
            )
            await recover_to_last_safe(
                drone,
                target_down,
                f"mid-step unsafe {reason}",
                prefer_latest=False,
            )
            return False

        if allow_corridor and narrow_corridor_active(c):
            timeout_s = min(timeout_s, CORRIDOR_MOVE_TIMEOUT_S)
            now = time.time()

            if now - last_guidance_update >= 0.15:
                corridor_steer_deg = corridor_steer_adjustment_deg(c)
                guided_yaw = normalize_angle_deg(base_yaw + corridor_steer_deg)
                guided_rad = math.radians(guided_yaw)
                target_n = latest_position_ned.north_m + (
                    NARROW_CORRIDOR_STEP_M * math.cos(guided_rad)
                )
                target_e = latest_position_ned.east_m + (
                    NARROW_CORRIDOR_STEP_M * math.sin(guided_rad)
                )

                await drone.offboard.set_position_ned(
                    PositionNedYaw(
                        target_n,
                        target_e,
                        target_down,
                        guided_yaw,
                    )
                )
                last_guidance_update = now
        elif allow_corridor:
            reset_corridor_pid()

        dist_to_target = math.hypot(
            latest_position_ned.north_m - target_n,
            latest_position_ned.east_m - target_e,
        )

        if dist_to_target < MOVE_REACHED_RADIUS_M:
            mark_current_cell_visited()
            remember_safe_position()
            if active_investigation_enabled:
                trigger_photo_burst(STOP_CAPTURE_BURST_FRAMES)
            return True

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

    await set_yaw(drone, home_yaw, target_down, duration_s=0.35)

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

        await set_yaw(drone, home_yaw, target_down, duration_s=0.35)

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

    if latest_position_ned is None:
        return False

    if critical_vehicle_state(target_down):
        raise RuntimeError("critical_state")

    if not vehicle_stable(target_down):
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
    tight_turn_side_limit = (
        BLIND_TIGHT_TURN_SIDE_CLEARANCE_M
        if pre_move_safe
        else YAW_MIN_SIDE_CLEARANCE_M
    )

    if (
        not active_investigation_enabled
        and turn_delta > 50.0
        and side_clearance < tight_turn_side_limit
    ):
        exploration_memory.mark_blocked_ray(
            latest_position_ned.north_m,
            latest_position_ned.east_m,
            selected_heading,
            min(c["center"], side_clearance),
        )
        print(
            f"🧭 [{label}] blind coverage skipping tight turn: "
            f"turn={turn_delta:.1f} side={side_clearance:.2f}"
        )
        log_mission_action(
            action_type="BLIND_TIGHT_TURN_SKIP",
            label=label,
            target_down=target_down,
            preferred_heading=preferred_heading_deg,
            selected_heading=selected_heading,
            heading_score=heading_score,
            extra={
                "turn_delta_deg": turn_delta,
                "side_clearance_m": side_clearance,
            },
        )
        return False

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

    step_m = NARROW_CORRIDOR_STEP_M if corridor_mode else MOVE_STEP_M
    yaw_rad = math.radians(command_heading)

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

    return await move_to_position_step(
        drone,
        target_n,
        target_e,
        target_down,
        command_heading,
        label=label,
        allow_corridor=NARROW_CORRIDOR_ENABLED,
    )


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

        if distance_from_start() > RESUME_RANGE_M:
            score -= 0.05 * angle_diff_deg(heading, heading_to_start_deg())
        elif distance_from_start() < 4.0:
            away_heading = normalize_angle_deg(
                math.degrees(
                    math.atan2(
                        latest_position_ned.east_m - start_e,
                        latest_position_ned.north_m - start_n,
                    )
                )
            )
            score -= 0.015 * angle_diff_deg(heading, away_heading)

        if score > best_score:
            best_score = score
            best_heading = heading

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
    moved_steps = 0

    while (
        search_time_remaining()
        and distance_from_start() > FRONTIER_SECTOR_RESUME_RANGE_M
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
        model_path="Codes/yolov10n.pt",
        burst_size=STOP_CAPTURE_BURST_FRAMES,
        threshold=DETECTION_CONFIDENCE_THRESHOLD,
    )
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

    print("\n===== GNSS-FREE X500_VISION QUALIFIER MISSION STARTED =====")
    print("Mode: fast open-frontier coverage first, candidate revisit second")
    print("A colour blob is remembered during coverage, then actively investigated later.")
    print("Exploration: open headings and local visited-path memory prefer new ground.\n")

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
        perception.cancel()
        camera_photo_saver.running = False
        camera_task.cancel()

        with contextlib.suppress(asyncio.CancelledError):
            await perception

        with contextlib.suppress(asyncio.CancelledError):
            await camera_task

        mission_logger.save()

        await stop_and_land(drone, safety_reason)



if __name__ == "__main__":
    asyncio.run(main())

