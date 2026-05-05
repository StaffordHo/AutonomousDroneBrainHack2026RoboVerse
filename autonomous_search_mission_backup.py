import asyncio
import os
import time
from datetime import datetime

import cv2
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

import integrated_stationary_mission as ism
from detection_logger import DetectionLogger
from small_fuel_detector import detect_small_fuel_barrels
from obstacle_monitor import create_obstacle_monitor

EVIDENCE_DIR = "search_evidence"


async def wait_connected(drone):
    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected.")
            return


async def wait_local_position(drone):
    print("Waiting for local position estimate...")
    async for health in drone.telemetry.health():
        print(
            f"global={health.is_global_position_ok}, "
            f"home={health.is_home_position_ok}, "
            f"local={health.is_local_position_ok}"
        )
        if health.is_local_position_ok:
            print("Local position OK.")
            return


async def arm_with_retry(drone, max_attempts=10, retry_delay=2.0):
    from mavsdk.action import ActionError

    for attempt in range(1, max_attempts + 1):
        try:
            print(f"Arming attempt {attempt}/{max_attempts}...")
            await drone.action.arm()
            print("Armed successfully.")
            return True
        except ActionError as error:
            print(f"Arming failed: {error}. Retrying in {retry_delay} seconds...")
            await asyncio.sleep(retry_delay)

    return False


def save_search_evidence(frame, detections, summary, label):
    os.makedirs(EVIDENCE_DIR, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = ism.draw_detections(frame, detections, summary)

    image_path = os.path.join(EVIDENCE_DIR, f"{timestamp}_{label}.png")
    cv2.imwrite(image_path, output)

    print(f"Saved search evidence: {image_path}")


async def perception_scan(logger, duration_s=5, label="scan"):
    """
    Runs perception for a short duration while the drone is hovering or moving slowly.
    """
    print(f"Perception scan: {label} for {duration_s}s")

    start_time = time.time()
    last_print_time = 0
    latest_detections = []

    while time.time() - start_time < duration_s:
        if ism.latest_frame is None:
            await asyncio.sleep(0.05)
            continue

        frame = ism.latest_frame.copy()

        detections, _, _, raw_detections = detect_small_fuel_barrels(frame)
        latest_detections = detections

        new_confirmations = logger.update(detections)
        summary = logger.summary()

        for confirmed in new_confirmations:
            print(
                f"NEW confirmed fuel: {confirmed['colour']} "
                f"center={confirmed['center']} bbox={confirmed['bbox']}"
            )
            save_search_evidence(frame, detections, summary, f"{label}_{confirmed['colour']}")

        now = time.time()
        if now - last_print_time > 1.0:
            print(
                f"[{label}] raw={len(raw_detections)}, merged={len(detections)} | "
                f"confirmed red={summary['red']}, yellow={summary['yellow']}, total={summary['total']}"
            )
            last_print_time = now

        output = ism.draw_detections(frame, detections, summary)
        cv2.imshow("Autonomous Search Mission", output)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        await asyncio.sleep(0.05)

    summary = logger.summary()

    if ism.latest_frame is not None:
        save_search_evidence(
            ism.latest_frame.copy(),
            latest_detections,
            summary,
            f"{label}_summary",
        )

    return summary


async def set_velocity_for(drone, vx, vy, vz, yaw_rate, duration_s, label, obstacle_monitor=None):
    """
    Body-frame velocity command:
    vx: forward m/s
    vy: right m/s
    vz: down m/s, so 0 keeps altitude
    yaw_rate: deg/s
    """
    print(f"Movement: {label}")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(vx, vy, vz, yaw_rate)
    )
    await asyncio.sleep(duration_s)

    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(1.0)

async def change_altitude_for(drone, vz, duration_s, label):
    """
    Change altitude using body-frame vertical velocity.

    PX4/MAVSDK body velocity convention:
    vz < 0 means move up
    vz > 0 means move down
    """
    print(f"Altitude movement: {label}")

    step_s = 0.5
    elapsed = 0.0

    while elapsed < duration_s:
                if obstacle_monitor is not None:
            too_close, distance = obstacle_monitor.obstacle_too_close()

            if distance is not None:
                print(f"[{label}] front distance = {distance:.2f} m")

            if too_close:
                print(f"[{label}] Obstacle too close at {distance:.2f} m. Stopping movement.")

                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
                )

                # Simple avoidance: yaw right slowly to look for a safer direction.
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(0.0, 0.0, 0.0, 20.0)
                )
                await asyncio.sleep(2.0)

                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
                )
                await asyncio.sleep(1.0)

                return

async def yaw_scan(drone, logger, label):
    """
    Small yaw scan at current position.
    """
    print(f"Yaw scan: {label}")

    perception_task = asyncio.create_task(
        perception_scan(logger, duration_s=10, label=label)
    )

    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 12.0)
    )
    await asyncio.sleep(4)

    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, -12.0)
    )
    await asyncio.sleep(8)

    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 12.0)
    )
    await asyncio.sleep(4)

    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )

    return await perception_task


async def main():
    print("Starting Gazebo camera subscriber...")
    node = Node()
    node.subscribe(Image, ism.IMAGE_TOPIC, ism.image_callback)
    print("Starting depth obstacle monitor...")
    depth_node, obstacle_monitor = create_obstacle_monitor()

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    await wait_connected(drone)
    await wait_local_position(drone)

    print("Setting takeoff altitude...")
    await drone.action.set_takeoff_altitude(2.5)

    print("Waiting for PX4 health checks to settle...")
    await asyncio.sleep(5)

    print("Arming...")
    armed = await arm_with_retry(drone)
    if not armed:
        print("Mission aborted: arming failed.")
        return

    print("Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(8)

    print("Setting initial offboard setpoint...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )

    print("Starting offboard mode...")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard failed: {error._result.result}")
        print("Landing...")
        await drone.action.land()
        return

    logger = DetectionLogger(center_distance_threshold=120, confirmation_frames=3)

    try:
        # Search pattern: conservative movement + scan stops
        # Search pattern: scan at multiple heights and positions

                # ============================================================
        # Multi-altitude lawnmower exploration pattern
        # ============================================================

        print("Starting autonomous map exploration...")

        # ---- Mid-altitude initial scan ----
        await yaw_scan(drone, logger, "scan_start_mid_altitude")

        # ---- Lane 1: move forward ----
        await set_velocity_for(
            drone,
            vx=0.4,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.0,
            duration_s=6,
            label="lane1_forward",
            obstacle_monitor=obstacle_monitor
        )
        await yaw_scan(drone, logger, "scan_lane1_end")

        # ---- Shift right to lane 2 ----
        await set_velocity_for(
            drone,
            vx=0.0,
            vy=0.4,
            vz=0.0,
            yaw_rate=0.0,
            duration_s=4,
            label="shift_right_to_lane2",
            obstacle_monitor=obstacle_monitor
        )
        await yaw_scan(drone, logger, "scan_lane2_start")

        # ---- Lane 2: move backward ----
        await set_velocity_for(
            drone,
            vx=-0.4,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.0,
            duration_s=6,
            label="lane2_backward",
            obstacle_monitor=obstacle_monitor
        )
        await yaw_scan(drone, logger, "scan_lane2_end")

        # ---- Shift right to lane 3 ----
        await set_velocity_for(
            drone,
            vx=0.0,
            vy=0.4,
            vz=0.0,
            yaw_rate=0.0,
            duration_s=4,
            label="shift_right_to_lane3",
            obstacle_monitor=obstacle_monitor
        )
        await yaw_scan(drone, logger, "scan_lane3_start")

        # ---- Lane 3: move forward ----
        await set_velocity_for(
            drone,
            vx=0.4,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.0,
            duration_s=6,
            label="lane3_forward",
            obstacle_monitor=obstacle_monitor
        )
        await yaw_scan(drone, logger, "scan_lane3_end")

        # ============================================================
        # Altitude sweep for hidden/elevated compartments
        # ============================================================

        # Move up for elevated red fuel barrels
        await change_altitude_for(
            drone,
            vz=-0.20,
            duration_s=4,
            label="move_up_for_elevated_red_targets",
            obstacle_monitor=obstacle_monitor
        )
        await yaw_scan(drone, logger, "scan_high_altitude")

        # Move left/back across another lane at high altitude
        await set_velocity_for(
            drone,
            vx=-0.3,
            vy=-0.3,
            vz=0.0,
            yaw_rate=0.0,
            duration_s=5,
            label="diagonal_high_altitude_reposition",
            obstacle_monitor=obstacle_monitor
        )
        await yaw_scan(drone, logger, "scan_high_altitude_repositioned")

        # Move down for ground-level yellow fuel barrels
        await change_altitude_for(
            drone,
            vz=0.20,
            duration_s=5,
            label="move_down_for_ground_yellow_targets",
            obstacle_monitor=obstacle_monitor
        )
        await yaw_scan(drone, logger, "scan_low_altitude")

        # Final slow forward scan at lower altitude
        await set_velocity_for(
            drone,
            vx=0.3,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.0,
            duration_s=5,
            label="final_low_altitude_forward",
            obstacle_monitor=obstacle_monitor
        )
        await yaw_scan(drone, logger, "scan_final")

        final_summary = logger.summary()
        print("Final search summary:")
        print(final_summary)

    finally:
        print("Stopping offboard mode...")
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            print(f"Stopping offboard failed: {error._result.result}")

        print("Landing...")
        await drone.action.land()

        cv2.destroyAllWindows()

    print("Autonomous search mission complete.")


if __name__ == "__main__":
    asyncio.run(main())
