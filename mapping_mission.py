import asyncio
import math
import time
import grpc
import numpy as np

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

from obstacle_monitor import ObstacleMonitor
from trajectory_logger import TrajectoryLogger
from local_mapper import LocalMapper, normalize_angle_deg


# ============================================================
# Mission configuration
# ============================================================

MISSION_TIME_LIMIT_S = 6 * 60

# NED down: negative is up.
SCAN_ALT_D = -1.5

# Normal altitude tolerance during controlled flight.
ALT_TOLERANCE_M = 0.9

# Hard safety threshold. If exceeded, land immediately.
CRITICAL_ALTITUDE_DEVIATION_M = 3.0

# Slightly larger movement step now that the stability behaviour is safer.
MOVE_STEP_M = 0.45
MOVE_TIMEOUT_S = 1.8

# Widened mapping area.
# Previous stable version used soft=7, hard=10.
SOFT_RANGE_LIMIT_M = 12.0
HARD_RANGE_LIMIT_M = 16.0

DEPTH_TOPIC = "/depth_camera"

# Full 360 scans are useful but slow. Increase this to explore more.
SCAN_EVERY_N_MOVES = 8

# Scan less aggressively while still mapping headings.
SCAN_DELTAS_DEG = [60, 120, 180, -120, -60, 0]

# If pitch/roll exceeds this, pause/hold.
MAX_ATTITUDE_DEG = 8.0

# If pitch/roll exceeds this, land immediately.
CRITICAL_ATTITUDE_DEG = 25.0

# If repeatedly blocked, do not keep forcing movement.
MAX_BLOCKED_STREAK = 8

# If return-home repeatedly fails, land safely.
MAX_RETURN_ATTEMPTS = 6

# Exploration bias headings. This prevents the drone from only following local
# obstacle turns and helps it cover a wider sector of the map.
PATROL_RELATIVE_HEADINGS_DEG = [
    0,
    45,
    -45,
    90,
    -90,
    135,
    -135,
    180,
]


# ============================================================
# Shared state
# ============================================================

latest_position_ned = None
latest_attitude = {
    "pitch": 0.0,
    "roll": 0.0,
    "yaw": 0.0,
}

monitor = ObstacleMonitor(
    obstacle_distance_m=1.65,
    warning_distance_m=2.35,
)

mapper = LocalMapper(size_m=60.0, resolution=0.5)
logger = TrajectoryLogger(log_dir="logs")

mission_start_time = None
start_n = 0.0
start_e = 0.0
start_yaw = 0.0

blocked_streak = 0
return_attempt_count = 0
patrol_heading_index = 0


# ============================================================
# Callback and helper functions
# ============================================================

def depth_callback(msg: Image):
    depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    monitor.update_depth(depth)


def elapsed_s():
    if mission_start_time is None:
        return 0.0

    return time.time() - mission_start_time


def timed_out():
    return elapsed_s() > MISSION_TIME_LIMIT_S


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


def altitude_error_m():
    if latest_position_ned is None:
        return 0.0

    return abs(latest_position_ned.down_m - SCAN_ALT_D)


def current_patrol_heading():
    rel = PATROL_RELATIVE_HEADINGS_DEG[patrol_heading_index % len(PATROL_RELATIVE_HEADINGS_DEG)]
    return normalize_angle_deg(start_yaw + rel)


def advance_patrol_heading():
    global patrol_heading_index
    patrol_heading_index += 1
    return current_patrol_heading()


def log_current(action):
    if latest_position_ned is None:
        return

    logger.log_state(
        latest_position_ned.north_m,
        latest_position_ned.east_m,
        latest_position_ned.down_m,
        latest_attitude["yaw"],
        monitor.get_directional_clearance(),
        action,
        pitch_deg=latest_attitude["pitch"],
        roll_deg=latest_attitude["roll"],
        distance_from_start_m=distance_from_start(),
    )


def critical_vehicle_state():
    """
    Returns True if the vehicle is too unstable to recover safely in Offboard.
    """
    if latest_position_ned is None:
        return False

    pitch = abs(latest_attitude["pitch"])
    roll = abs(latest_attitude["roll"])
    alt_err = altitude_error_m()

    if pitch > CRITICAL_ATTITUDE_DEG or roll > CRITICAL_ATTITUDE_DEG:
        print(
            f"🚨 Critical attitude: pitch={latest_attitude['pitch']:.1f}, "
            f"roll={latest_attitude['roll']:.1f}. Landing."
        )
        return True

    if alt_err > CRITICAL_ALTITUDE_DEVIATION_M:
        print(
            f"🚨 Critical altitude deviation: down={latest_position_ned.down_m:.2f}, "
            f"target={SCAN_ALT_D:.2f}, error={alt_err:.2f}. Landing."
        )
        return True

    if distance_from_start() > HARD_RANGE_LIMIT_M:
        print(f"🚨 Hard range limit exceeded: {distance_from_start():.1f} m. Landing.")
        return True

    return False


def vehicle_attitude_safe():
    return (
        abs(latest_attitude["pitch"]) <= MAX_ATTITUDE_DEG
        and abs(latest_attitude["roll"]) <= MAX_ATTITUDE_DEG
    )


def altitude_safe():
    return altitude_error_m() <= ALT_TOLERANCE_M


# ============================================================
# Telemetry and PX4 helpers
# ============================================================

async def wait_for_telemetry():
    print("Waiting for telemetry...")

    while latest_position_ned is None:
        await asyncio.sleep(0.1)

    print("Telemetry ready.")


async def wait_for_depth(timeout_s=8.0):
    print("Waiting for depth frames...")

    start = time.time()

    while time.time() - start < timeout_s:
        if monitor.latest_depth is not None:
            print("Depth camera ready.")
            return True

        await asyncio.sleep(0.1)

    print("Warning: depth camera not ready.")
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


async def prime_and_start_offboard(drone):
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
                SCAN_ALT_D,
                yaw,
            )
        )
        await asyncio.sleep(0.5)


async def stop_and_land(drone, reason):
    print(f"🛑 Safety landing: {reason}")
    log_current(f"Safety landing: {reason}")

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
# Low-level motion
# ============================================================

async def hold_position(drone, duration_s=0.5, action="Hold"):
    if latest_position_ned is None:
        await asyncio.sleep(duration_s)
        return

    if critical_vehicle_state():
        raise RuntimeError("critical_state")

    await drone.offboard.set_position_ned(
        PositionNedYaw(
            latest_position_ned.north_m,
            latest_position_ned.east_m,
            SCAN_ALT_D,
            latest_attitude["yaw"],
        )
    )

    log_current(action)
    await asyncio.sleep(duration_s)


async def set_yaw(drone, yaw_deg, duration_s=0.6, action="Set yaw"):
    if latest_position_ned is None:
        await asyncio.sleep(duration_s)
        return

    if critical_vehicle_state():
        raise RuntimeError("critical_state")

    yaw_deg = normalize_angle_deg(yaw_deg)

    await drone.offboard.set_position_ned(
        PositionNedYaw(
            latest_position_ned.north_m,
            latest_position_ned.east_m,
            SCAN_ALT_D,
            yaw_deg,
        )
    )

    log_current(f"{action} {yaw_deg:.1f} deg")
    await asyncio.sleep(duration_s)


async def controlled_pause_if_needed(drone):
    if critical_vehicle_state():
        raise RuntimeError("critical_state")

    if not altitude_safe():
        print(
            f"⚠️ Altitude deviation down={latest_position_ned.down_m:.2f}, "
            f"target={SCAN_ALT_D:.2f}; holding."
        )
        await hold_position(drone, 0.8, action="Altitude correction")
        return False

    if not vehicle_attitude_safe():
        print(
            f"⚠️ Attitude not level p={latest_attitude['pitch']:.1f}, "
            f"r={latest_attitude['roll']:.1f}; holding."
        )
        await hold_position(drone, 0.8, action="Attitude recovery hold")
        return False

    return True


async def move_in_heading(
    drone,
    heading_deg,
    bypass_soft_range=False,
    label="move",
):
    """
    Move a short step in a specified heading.

    bypass_soft_range=True is used only by return-home logic.
    """
    global blocked_streak

    if latest_position_ned is None:
        return False

    if critical_vehicle_state():
        raise RuntimeError("critical_state")

    if not await controlled_pause_if_needed(drone):
        return False

    current_range = distance_from_start()

    if (not bypass_soft_range) and current_range > SOFT_RANGE_LIMIT_M:
        print(f"↩️ Soft range limit reached ({current_range:.1f} m). Return-home required.")
        return False

    await set_yaw(
        drone,
        heading_deg,
        duration_s=0.35,
        action=f"{label} yaw",
    )

    clearances = monitor.get_directional_clearance()
    front = clearances["center"]

    current_n = latest_position_ned.north_m
    current_e = latest_position_ned.east_m
    current_yaw = latest_attitude["yaw"]

    mapper.update_visited(current_n, current_e)

    if front > 0.05:
        mapper.update_ray(
            current_n,
            current_e,
            current_yaw,
            min(front, 6.0),
            mark_obstacle=(front < monitor.warning_distance_m),
        )

    if front < monitor.obstacle_distance_m:
        blocked_streak += 1

        mapper.mark_obstacle(
            current_n,
            current_e,
            current_yaw,
            max(front, monitor.obstacle_distance_m),
        )

        log_current(f"{label}: blocked front={front:.2f}")

        print(f"⚠️ {label}: blocked front={front:.2f} m, blocked_streak={blocked_streak}")

        return False

    blocked_streak = 0

    yaw_rad = math.radians(current_yaw)
    target_n = current_n + MOVE_STEP_M * math.cos(yaw_rad)
    target_e = current_e + MOVE_STEP_M * math.sin(yaw_rad)

    print(
        f"➡️ {label}: moving {MOVE_STEP_M:.2f} m at yaw={current_yaw:.1f} deg, "
        f"range={current_range:.1f} m, front={front:.2f} m"
    )

    await drone.offboard.set_position_ned(
        PositionNedYaw(
            target_n,
            target_e,
            SCAN_ALT_D,
            current_yaw,
        )
    )

    log_current(f"{label}: move {MOVE_STEP_M:.2f}m")

    start_move = time.time()

    while time.time() - start_move < MOVE_TIMEOUT_S:
        await asyncio.sleep(0.1)

        if critical_vehicle_state():
            raise RuntimeError("critical_state")

        too_close, distance = monitor.obstacle_too_close()

        if too_close and distance < 1.10:
            blocked_streak += 1
            print(f"🚨 Mid-step abort, obstacle at {distance:.2f} m, blocked_streak={blocked_streak}")

            mapper.mark_obstacle(
                latest_position_ned.north_m,
                latest_position_ned.east_m,
                latest_attitude["yaw"],
                max(distance, 0.8),
            )

            await hold_position(drone, 0.3, action=f"Mid-step abort obstacle {distance:.2f}")

            return False

        dist_to_target = math.hypot(
            latest_position_ned.north_m - target_n,
            latest_position_ned.east_m - target_e,
        )

        if dist_to_target < 0.18:
            mapper.update_visited(
                latest_position_ned.north_m,
                latest_position_ned.east_m,
            )
            return True

    mapper.update_visited(
        latest_position_ned.north_m,
        latest_position_ned.east_m,
    )

    return True


# ============================================================
# Higher-level exploration logic
# ============================================================

async def return_toward_start(drone):
    global return_attempt_count

    return_attempt_count += 1

    if return_attempt_count > MAX_RETURN_ATTEMPTS:
        raise RuntimeError("return_home_failed")

    start_distance = distance_from_start()
    home_yaw = heading_to_start_deg()

    print(
        f"🏠 Return-home attempt {return_attempt_count}/{MAX_RETURN_ATTEMPTS}, "
        f"range={start_distance:.1f} m"
    )

    moved = await move_in_heading(
        drone,
        home_yaw,
        bypass_soft_range=True,
        label="Return home",
    )

    end_distance = distance_from_start()

    if end_distance < SOFT_RANGE_LIMIT_M * 0.85:
        print("🏠 Back inside safe range. Resuming exploration.")
        return_attempt_count = 0
        return True

    if moved and end_distance < start_distance:
        print(f"🏠 Returning, range reduced {start_distance:.1f} -> {end_distance:.1f} m")
        return True

    for offset in [35, -35, 70, -70]:
        detour_yaw = normalize_angle_deg(home_yaw + offset)

        moved = await move_in_heading(
            drone,
            detour_yaw,
            bypass_soft_range=True,
            label=f"Return detour {offset:+d}",
        )

        new_distance = distance_from_start()

        if new_distance < end_distance:
            print(f"🏠 Detour reduced range {end_distance:.1f} -> {new_distance:.1f} m")
            return True

    print("⚠️ Return-home did not reduce range this cycle.")
    return False


async def scan_surroundings(drone):
    print("🔭 Scanning surroundings...")

    if latest_position_ned is None:
        return

    if critical_vehicle_state():
        raise RuntimeError("critical_state")

    current_n = latest_position_ned.north_m
    current_e = latest_position_ned.east_m
    base_yaw = latest_attitude["yaw"]

    log_current("Start scan")

    for delta in SCAN_DELTAS_DEG:
        yaw = normalize_angle_deg(base_yaw + delta)

        await set_yaw(
            drone,
            yaw,
            duration_s=0.45,
            action="Scan yaw",
        )

        clearances = monitor.get_directional_clearance()
        front = clearances["center"]

        if front > 0.05:
            mapper.update_ray(
                current_n,
                current_e,
                yaw,
                min(front, 6.0),
                mark_obstacle=(front < monitor.warning_distance_m),
            )

        logger.log_state(
            current_n,
            current_e,
            latest_position_ned.down_m,
            yaw,
            clearances,
            f"Scan sample yaw {yaw:.1f}",
            pitch_deg=latest_attitude["pitch"],
            roll_deg=latest_attitude["roll"],
            distance_from_start_m=distance_from_start(),
        )

    print("🔭 Scan complete.")


async def explore_step(drone):
    global return_attempt_count
    global blocked_streak

    if latest_position_ned is None:
        return False

    if critical_vehicle_state():
        raise RuntimeError("critical_state")

    current_n = latest_position_ned.north_m
    current_e = latest_position_ned.east_m
    current_yaw = latest_attitude["yaw"]

    mapper.update_visited(current_n, current_e)

    if distance_from_start() > SOFT_RANGE_LIMIT_M:
        await return_toward_start(drone)
        return False

    return_attempt_count = 0

    clearances = monitor.get_directional_clearance()
    front = clearances["center"]

    if front < monitor.obstacle_distance_m:
        blocked_streak += 1

        mapper.mark_obstacle(
            current_n,
            current_e,
            current_yaw,
            max(front, monitor.obstacle_distance_m),
        )

        new_yaw = mapper.suggest_heading(
            current_n,
            current_e,
            current_yaw,
            clearances=clearances,
        )

        print(
            f"⚠️ Blocked (front={front:.2f}m). "
            f"Turning to {new_yaw:.1f} deg. blocked_streak={blocked_streak}"
        )

        logger.log_state(
            current_n,
            current_e,
            latest_position_ned.down_m,
            current_yaw,
            clearances,
            f"Blocked, turn to {new_yaw:.1f} deg",
            pitch_deg=latest_attitude["pitch"],
            roll_deg=latest_attitude["roll"],
            distance_from_start_m=distance_from_start(),
        )

        await set_yaw(
            drone,
            new_yaw,
            duration_s=0.7,
            action="Avoid obstacle yaw",
        )

        if blocked_streak >= MAX_BLOCKED_STREAK:
            print("🧭 Repeated blocked states. Switching patrol heading instead of landing.")
            blocked_streak = 0
            next_heading = advance_patrol_heading()
            await set_yaw(
                drone,
                next_heading,
                duration_s=0.8,
                action="Advance patrol heading",
            )

        return False

    mapper.update_ray(
        current_n,
        current_e,
        current_yaw,
        min(front, 6.0),
        mark_obstacle=(front < monitor.warning_distance_m),
    )

    # Bias toward patrol headings when things are clear. This helps widen coverage.
    target_heading = current_yaw

    if distance_from_start() < SOFT_RANGE_LIMIT_M * 0.75:
        target_heading = current_patrol_heading()

    return await move_in_heading(
        drone,
        target_heading,
        bypass_soft_range=False,
        label="Explore",
    )


# ============================================================
# Main
# ============================================================

async def main():
    global mission_start_time
    global start_n
    global start_e
    global start_yaw

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    node = Node()
    node.subscribe(Image, DEPTH_TOPIC, depth_callback)

    asyncio.create_task(telemetry_task(drone))

    await wait_for_telemetry()
    await wait_for_depth(timeout_s=8.0)

    print("Arming & takeoff...")

    armed = await arm_with_retry(drone)

    if not armed:
        print("Mission aborted: arming failed.")
        return

    await drone.action.set_takeoff_altitude(abs(SCAN_ALT_D))
    await drone.action.takeoff()
    await asyncio.sleep(8)

    await wait_for_telemetry()

    start_n = latest_position_ned.north_m
    start_e = latest_position_ned.east_m
    start_yaw = latest_attitude["yaw"]
    mission_start_time = time.time()

    mapper.initialize_start(start_n, start_e)

    try:
        await prime_and_start_offboard(drone)

    except OffboardError as error:
        print(f"Offboard start failed: {error}. Landing.")
        await drone.action.land()
        return

    print("\n===== WIDER SAFE MAPPING EXPLORATION STARTED =====")

    steps_since_scan = SCAN_EVERY_N_MOVES

    safety_reason = "timeout"

    try:
        while not timed_out():
            if critical_vehicle_state():
                safety_reason = "critical vehicle state"
                break

            if steps_since_scan >= SCAN_EVERY_N_MOVES:
                await scan_surroundings(drone)
                steps_since_scan = 0

            moved = await explore_step(drone)

            if moved:
                steps_since_scan += 1

            log_current("Loop heartbeat")

    except RuntimeError as error:
        safety_reason = str(error)
        print(f"Safety stop: {safety_reason}")

    finally:
        print("\n==============================")
        print("MISSION COMPLETE OR SAFETY STOP")

        await stop_and_land(drone, safety_reason)

        logger.save()
        mapper.save(
            log_dir=logger.log_dir,
            start_time_str=logger.start_time_str,
        )

        print("==============================")


if __name__ == "__main__":
    asyncio.run(main())
