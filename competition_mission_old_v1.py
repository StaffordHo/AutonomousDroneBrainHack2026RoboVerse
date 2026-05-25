import asyncio
import math
import os
import subprocess
import time
from datetime import datetime

import cv2
import numpy as np
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

from small_fuel_detector import detect_small_fuel_barrels
from bearing_detection_logger import BearingDetectionLogger, normalize_angle_deg
from obstacle_monitor import ObstacleMonitor
from depth_debugger import save_depth_debug


# =========================
# Mission configuration
# =========================

EVIDENCE_DIR = "competition_evidence"
MISSION_TIME_LIMIT_S = 9 * 60

YELLOW_SCORE = 50
RED_SCORE = 100

# NED convention: negative down = up.
LOW_SCAN_ALT_D = -1.8      # better for yellow ground-level canisters
HIGH_SCAN_ALT_D = -3.2     # better for elevated red canisters
DEFAULT_ALT_D = LOW_SCAN_ALT_D

SEARCH_LIMIT_M = 10.0
SEARCH_STEP_M = 5.0

IMAGE_TOPIC_FALLBACK = "/world/roboverse/model/x500_depth_0/link/camera_link/sensor/IMX214/image"
DEPTH_TOPIC = "/depth_camera"

IMX214_HFOV_DEG = 69.0


# =========================
# Shared state
# =========================

latest_frame = None
latest_position_ned = None
latest_attitude = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}

monitor = ObstacleMonitor(obstacle_distance_m=1.6)
mission_start_time = None
start_n = 0.0
start_e = 0.0
score = 0


# =========================
# Callbacks / helpers
# =========================

def image_callback(msg: Image):
    global latest_frame
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    latest_frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


def depth_callback(msg: Image):
    depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    monitor.update_depth(depth)


def elapsed_s():
    if mission_start_time is None:
        return 0.0
    return time.time() - mission_start_time


def timed_out():
    return elapsed_s() > MISSION_TIME_LIMIT_S


def get_score(summary):
    return summary["red"] * RED_SCORE + summary["yellow"] * YELLOW_SCORE


def find_image_topic():
    try:
        topics = subprocess.check_output(["gz", "topic", "-l"], timeout=3).decode().split()
        for t in topics:
            if "IMX214/image" in t:
                return t
    except Exception:
        pass
    return IMAGE_TOPIC_FALLBACK


def estimate_bearing_deg(det, frame_shape):
    cx, _ = det["center"]
    _, w = frame_shape[:2]
    norm_x = (cx - w / 2.0) / max(w, 1)
    camera_offset = norm_x * IMX214_HFOV_DEG
    return normalize_angle_deg(latest_attitude["yaw"] + camera_offset)


def localize_detection(det, frame_shape):
    """
    Rough target localisation from bearing + depth.
    Good enough for duplicate suppression, not precision navigation.
    """
    depth_m = monitor.sample_depth_for_rgb_bbox(det["bbox"], frame_shape)
    bearing_deg = estimate_bearing_deg(det, frame_shape)

    if latest_position_ned is None or depth_m is None:
        return bearing_deg, None, None, depth_m

    yaw_rad = math.radians(bearing_deg)

    # Use depth as approximate horizontal range. This is imperfect but sufficient for dedup.
    tn = latest_position_ned.north_m + depth_m * math.cos(yaw_rad)
    te = latest_position_ned.east_m + depth_m * math.sin(yaw_rad)

    return bearing_deg, tn, te, depth_m


def refine_bbox_for_evidence(det, frame_shape):
    x, y, w, h = det["bbox"]
    img_h, img_w = frame_shape[:2]

    # Evidence-only padding; detection logic still uses original bbox.
    pad_x = max(3, int(0.15 * w))
    pad_y = max(3, int(0.10 * h))

    if det["colour"] == "red":
        # Red canister core is often partially occluded by compartment edge.
        x = max(0, x - int(0.20 * w) - pad_x)
    else:
        x = max(0, x - pad_x)

    y = max(0, y - pad_y)
    w = min(img_w - x, w + 2 * pad_x)
    h = min(img_h - y, h + 2 * pad_y)

    new_det = det.copy()
    new_det["bbox"] = (x, y, w, h)
    new_det["center"] = (x + w // 2, y + h // 2)
    return new_det


def save_evidence(frame, det, confirmed, summary):
    global score

    os.makedirs(EVIDENCE_DIR, exist_ok=True)

    colour = det["colour"]
    val = RED_SCORE if colour == "red" else YELLOW_SCORE
    score = get_score(summary)

    draw = frame.copy()
    det_draw = refine_bbox_for_evidence(det, frame.shape)
    x, y, w, h = det_draw["bbox"]

    box_colour = (0, 0, 255) if colour == "red" else (0, 255, 255)
    cv2.rectangle(draw, (x, y), (x + w, y + h), box_colour, 3)

    text = f"CONFIRMED {colour} +{val} Score:{score}"
    cv2.putText(draw, text, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.4, box_colour, 3)

    if det.get("depth_m") is not None:
        cv2.putText(
            draw,
            f"depth={det['depth_m']:.2f}m bearing={det.get('bearing_deg', 0):.1f}",
            (20, 105),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(EVIDENCE_DIR, f"CONFIRMED_{colour}_{ts}.png")
    cv2.imwrite(path, draw)
    print(f"🎯 SAVED EVIDENCE: {path}")
    return path


async def wait_for_telemetry():
    print("Waiting for telemetry...")
    while latest_position_ned is None:
        await asyncio.sleep(0.1)
    print("Telemetry ready.")


async def telemetry_task(drone):
    async def read_pos():
        global latest_position_ned
        async for p in drone.telemetry.position_velocity_ned():
            latest_position_ned = p.position

    async def read_att():
        global latest_attitude
        async for a in drone.telemetry.attitude_euler():
            latest_attitude = {
                "pitch": a.pitch_deg,
                "roll": a.roll_deg,
                "yaw": a.yaw_deg,
            }

    await asyncio.gather(read_pos(), read_att())


async def arm_with_retry(drone, attempts=10):
    for i in range(1, attempts + 1):
        try:
            print(f"Arming attempt {i}/{attempts}...")
            await drone.action.arm()
            print("Armed.")
            return True
        except ActionError as e:
            print(f"Arming denied: {e}. Retrying...")
            await asyncio.sleep(2)
    return False


# =========================
# Perception
# =========================

async def scan_and_capture(drone, logger, label="scan", scan_alt_d=None):
    """
    Rotate in place and detect/capture fuel barrels.
    Returns summary.
    """
    if scan_alt_d is None:
        scan_alt_d = DEFAULT_ALT_D

    print(f"🔭 [{label}] scanning at alt_d={scan_alt_d:.1f}")

    if latest_position_ned is None:
        return logger.summary()

    base_yaw = latest_attitude["yaw"]

    # 360 scan in coarse increments. This is simple and robust.
    for delta in [0, 45, 90, 135, 180, -135, -90, -45]:
        if timed_out():
            break

        if latest_position_ned is None:
            await asyncio.sleep(0.1)
            continue

        yaw = normalize_angle_deg(base_yaw + delta)
        await drone.offboard.set_position_ned(
            PositionNedYaw(
                latest_position_ned.north_m,
                latest_position_ned.east_m,
                scan_alt_d,
                yaw,
            )
        )
        await asyncio.sleep(1.2)

        if latest_frame is None:
            continue

        frame = latest_frame.copy()
        detections, _, _, raw = detect_small_fuel_barrels(frame)

        for det in detections:
            bearing, tn, te, depth_m = localize_detection(det, frame.shape)
            enriched = det.copy()
            enriched["bearing_deg"] = bearing
            enriched["target_n"] = tn
            enriched["target_e"] = te
            enriched["depth_m"] = depth_m

            confirmed = logger.add_detection(enriched)
            if confirmed is not None:
                summary = logger.summary()
                save_evidence(frame, enriched, confirmed, summary)

        if len(detections) > 0:
            print(f"   [{label}] candidates={len(detections)} confirmed={logger.summary()['total']}")

    return logger.summary()


# =========================
# Navigation
# =========================

async def hold_position(drone, duration_s=0.5, alt_d=None):
    if latest_position_ned is None:
        await asyncio.sleep(duration_s)
        return

    if alt_d is None:
        alt_d = latest_position_ned.down_m

    await drone.offboard.set_position_ned(
        PositionNedYaw(
            latest_position_ned.north_m,
            latest_position_ned.east_m,
            alt_d,
            latest_attitude["yaw"],
        )
    )
    await asyncio.sleep(duration_s)


async def escape_from_obstacle(drone, alt_d, label):
    c = monitor.get_directional_clearance()
    left = c["left"]
    right = c["right"]
    front = c["center"]

    turn = 45 if right >= left else -45
    new_yaw = normalize_angle_deg(latest_attitude["yaw"] + turn)

    print(f"⚠️ [{label}] blocked front={front:.2f}m left={left:.2f}m right={right:.2f}m; turn {turn:+d}°")
    save_depth_debug(monitor.latest_depth, f"blocked_{label}")

    # Turn first.
    await drone.offboard.set_position_ned(
        PositionNedYaw(
            latest_position_ned.north_m,
            latest_position_ned.east_m,
            alt_d,
            new_yaw,
        )
    )
    await asyncio.sleep(1.0)

    # Move a short escape step in the new heading only if the new front is not critical.
    c2 = monitor.get_directional_clearance()
    if c2["center"] < 1.0:
        print(f"   [{label}] escape direction still too close ({c2['center']:.2f}m).")
        return False

    step = 0.7
    yaw_rad = math.radians(new_yaw)
    en = latest_position_ned.north_m + step * math.cos(yaw_rad)
    ee = latest_position_ned.east_m + step * math.sin(yaw_rad)

    await drone.offboard.set_position_ned(PositionNedYaw(en, ee, alt_d, new_yaw))
    await asyncio.sleep(1.5)
    return True


async def navigate_safely(drone, target_n, target_e, target_alt_d, label="wp"):
    """
    Goal-directed step navigation with depth-based local avoidance.

    Deliberately simple: it skips hard waypoints instead of getting stuck.
    """
    start = time.time()
    blocked_count = 0

    while not timed_out():
        if latest_position_ned is None:
            await asyncio.sleep(0.1)
            continue

        curr_n = latest_position_ned.north_m
        curr_e = latest_position_ned.east_m
        dist = math.hypot(target_n - curr_n, target_e - curr_e)

        if dist < 0.9:
            print(f"✅ [{label}] reached waypoint.")
            return True

        if time.time() - start > 28:
            print(f"⏭️ [{label}] waypoint timeout. Skipping.")
            return False

        # Basic geofence around start.
        if abs(curr_n - start_n) > 19.0 or abs(curr_e - start_e) > 19.0:
            print(f"🚨 [{label}] geofence reached. Skipping outward move.")
            return False

        # Wait for level-ish vehicle before trusting depth.
        if abs(latest_attitude["pitch"]) > 5.0 or abs(latest_attitude["roll"]) > 5.0:
            await hold_position(drone, 0.2, target_alt_d)
            continue

        clearances = monitor.get_directional_clearance()
        front = clearances["center"]

        if front == 0.0:
            blocked_count += 1
            print(f"⚠️ [{label}] blind/blocked depth.")
            await escape_from_obstacle(drone, target_alt_d, label)
            if blocked_count >= 3:
                print(f"⏭️ [{label}] too many blocks. Skipping.")
                return False
            continue

        if front < monitor.obstacle_distance_m:
            blocked_count += 1
            await escape_from_obstacle(drone, target_alt_d, label)
            if blocked_count >= 3:
                print(f"⏭️ [{label}] too many blocks. Skipping.")
                return False
            continue

        # Face the waypoint, then take a short step.
        yaw_rad = math.atan2(target_e - curr_e, target_n - curr_n)
        yaw_deg = normalize_angle_deg(math.degrees(yaw_rad))

        step = min(0.9, dist)
        step_n = curr_n + step * math.cos(yaw_rad)
        step_e = curr_e + step * math.sin(yaw_rad)

        await drone.offboard.set_position_ned(PositionNedYaw(step_n, step_e, target_alt_d, yaw_deg))

        # Mid-step safety.
        for _ in range(12):
            await asyncio.sleep(0.1)
            too_close, d = monitor.obstacle_too_close()
            if too_close and d < 1.2:
                print(f"🚨 [{label}] mid-step abort, obstacle {d:.2f}m")
                await hold_position(drone, 0.2, target_alt_d)
                blocked_count += 1
                break

        # Do not immediately mark as stuck; let loop reevaluate.


def serpentine_waypoints(limit=SEARCH_LIMIT_M, step=SEARCH_STEP_M):
    rows = list(np.arange(-limit, limit + 0.001, step))
    cols = list(np.arange(-limit, limit + 0.001, step))

    points = []
    for i, dn in enumerate(rows):
        row_cols = cols if i % 2 == 0 else list(reversed(cols))
        for de in row_cols:
            points.append((float(dn), float(de)))
    return points


# =========================
# Main
# =========================

async def main():
    global mission_start_time, start_n, start_e, score

    os.makedirs(EVIDENCE_DIR, exist_ok=True)

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    # Subscribers.
    image_topic = find_image_topic()
    print(f"Using image topic: {image_topic}")

    node = Node()
    node.subscribe(Image, image_topic, image_callback)
    node.subscribe(Image, DEPTH_TOPIC, depth_callback)

    # Telemetry.
    asyncio.create_task(telemetry_task(drone))
    await wait_for_telemetry()

    print("Arming & takeoff...")
    armed = await arm_with_retry(drone)
    if not armed:
        print("Mission aborted: arming failed.")
        return

    await drone.action.set_takeoff_altitude(abs(DEFAULT_ALT_D))
    await drone.action.takeoff()
    await asyncio.sleep(6)

    await wait_for_telemetry()
    start_n = latest_position_ned.north_m
    start_e = latest_position_ned.east_m
    mission_start_time = time.time()

    # Offboard requires an initial setpoint.
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, latest_attitude["yaw"]))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard start failed: {e}. Landing.")
        await drone.action.land()
        return

    logger = BearingDetectionLogger(
        confirmation_frames=3,
        dist_threshold_m=1.2,
        bearing_threshold_deg=7.0,
        min_confidence=0.20,
    )

    try:
        waypoints = serpentine_waypoints()

        # Two passes: lower pass for yellow ground targets, higher pass for red elevated targets.
        for alt_d, pass_name in [(LOW_SCAN_ALT_D, "LOW_YELLOW_PASS"), (HIGH_SCAN_ALT_D, "HIGH_RED_PASS")]:
            print(f"\n===== {pass_name} alt_d={alt_d:.1f} =====")

            for idx, (dn, de) in enumerate(waypoints):
                if timed_out():
                    break

                summary = logger.summary()
                score = get_score(summary)

                # Keep searching even after eligibility is met, but this gives visibility.
                if summary["red"] >= 1 and summary["yellow"] >= 1:
                    print(f"Eligibility achieved. Current score={score}; continuing for more points until timeout.")

                label = f"{pass_name}_{idx:02d}_{dn:+.1f}_{de:+.1f}"

                # Scan before moving because current viewpoint may already see a target.
                await scan_and_capture(drone, logger, label=f"pre_{label}", scan_alt_d=alt_d)

                await navigate_safely(
                    drone,
                    start_n + dn,
                    start_e + de,
                    alt_d,
                    label=label,
                )

                # Always scan after move/blocked/skip.
                await scan_and_capture(drone, logger, label=f"post_{label}", scan_alt_d=alt_d)

        final = logger.summary()
        score = get_score(final)
        print("\n==============================")
        print("MISSION COMPLETE")
        print(f"Time: {elapsed_s():.1f}s")
        print(f"Red: {final['red']}  Yellow: {final['yellow']}  Total: {final['total']}")
        print(f"Score: {score}")
        print("==============================")

    finally:
        print("Landing...")
        try:
            await drone.offboard.stop()
        except Exception:
            pass
        await drone.action.land()


if __name__ == "__main__":
    asyncio.run(main())
