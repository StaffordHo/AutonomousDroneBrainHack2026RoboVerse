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

YELLOW_SCORE = 50
RED_SCORE = 100


def calculate_score(summary):
    return summary["yellow"] * YELLOW_SCORE + summary["red"] * RED_SCORE


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

    score = calculate_score(summary)

    cv2.putText(
        output,
        f"Score: {score}",
        (30, 100),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (255, 255, 255),
        3,
    )

    image_path = os.path.join(EVIDENCE_DIR, f"{timestamp}_{label}.png")
    cv2.imwrite(image_path, output)

    print(f"Saved search evidence: {image_path}")
    print(f"Current score: {score}")


async def perception_scan(logger, duration_s=5, label="scan", stop_on_detection=True):
    """
    Runs perception for a short duration while the drone is hovering or moving slowly.

    Returns:
        summary, found_new_detection
    """
    print(f"Perception scan: {label} for {duration_s}s")

    start_time = time.time()
    last_print_time = 0
    latest_detections = []
    found_new_detection = False

    while time.time() - start_time < duration_s:
        if ism.latest_frame is None:
            await asyncio.sleep(0.05)
            continue

        frame = ism.latest_frame.copy()

        detections, _, _, raw_detections = detect_small_fuel_barrels(frame)
        latest_detections = detections

        new_confirmations = logger.update(detections)
        summary = logger.summary()

        if new_confirmations:
            found_new_detection = True

            for confirmed in new_confirmations:
                print(
                    f"NEW confirmed fuel: {confirmed['colour']} "
                    f"center={confirmed['center']} bbox={confirmed['bbox']}"
                )

            score = calculate_score(summary)

            print(
                f"Score updated: red={summary['red']}, "
                f"yellow={summary['yellow']}, total={summary['total']}, "
                f"score={score}"
            )

            save_search_evidence(
                frame,
                detections,
                summary,
                f"{label}_new_fuel",
            )

            if stop_on_detection:
                print(f"[{label}] Fuel found. Ending scan and moving to next search area.")
                return summary, True

        now = time.time()
        if now - last_print_time > 1.0:
            score = calculate_score(summary)
            print(
                f"[{label}] raw={len(raw_detections)}, merged={len(detections)} | "
                f"confirmed red={summary['red']}, "
                f"yellow={summary['yellow']}, total={summary['total']}, "
                f"score={score}"
            )
            last_print_time = now

        output = ism.draw_detections(frame, detections, summary)

        cv2.putText(
            output,
            f"Score: {calculate_score(summary)}",
            (30, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 255, 255),
            3,
        )

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

    return summary, found_new_detection


async def set_velocity_for(
    drone,
    vx,
    vy,
    vz,
    yaw_rate,
    duration_s,
    label,
    obstacle_monitor=None,
):
    """
    Body-frame velocity command with simple front obstacle checking.

    vx: forward m/s
    vy: right m/s
    vz: down m/s, so 0 keeps altitude
    yaw_rate: deg/s

    Note:
    The obstacle check uses the forward-facing depth camera. It is most useful
    for forward movement. For sideways/backward movement, it is still a rough
    safety check but not a full 360-degree obstacle avoidance system.
    """
    print(f"Movement: {label}")

    step_s = 0.5
    elapsed = 0.0

    while elapsed < duration_s:
        if obstacle_monitor is not None:
            too_close, distance = obstacle_monitor.obstacle_too_close()

            if distance is not None:
                print(f"[{label}] front distance = {distance:.2f} m")

            if too_close:
                print(
                    f"[{label}] Obstacle too close at {distance:.2f} m. "
                    "Stopping movement and yawing away."
                )

                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
                )
                await asyncio.sleep(0.5)

                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(0.0, 0.0, 0.0, 20.0)
                )
                await asyncio.sleep(2.0)

                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
                )
                await asyncio.sleep(1.0)

                return False

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(vx, vy, vz, yaw_rate)
        )

        await asyncio.sleep(step_s)
        elapsed += step_s

    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(1.0)

    return True


async def change_altitude_for(drone, vz, duration_s, label):
    """
    Change altitude using body-frame vertical velocity.

    PX4/MAVSDK body velocity convention:
    vz < 0 means move up
    vz > 0 means move down

    Note:
    The current depth camera is forward-facing, so this function does not use
    front obstacle checking for vertical movement.
    """
    print(f"Altitude movement: {label}")

    step_s = 0.5
    elapsed = 0.0

    while elapsed < duration_s:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, vz, 0.0)
        )
        await asyncio.sleep(step_s)
        elapsed += step_s

    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(1.0)


async def yaw_for_or_until_detection(drone, perception_task, yaw_rate, duration_s):
    """
    Yaw for duration_s, but stop early if perception_task finishes.

    Returns:
        True only if the perception task actually confirmed a new fuel detection.
    """
    step_s = 0.25
    elapsed = 0.0

    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, yaw_rate)
    )

    while elapsed < duration_s:
        if perception_task.done():
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )

            _, found_new_detection = perception_task.result()
            return found_new_detection

        await asyncio.sleep(step_s)
        elapsed += step_s

    return False


async def yaw_scan(drone, logger, label):
    """
    Small yaw scan at current position.

    If a barrel is confirmed, the image is saved, score is updated,
    and the scan stops early so the drone can search elsewhere.
    """
    print(f"Yaw scan: {label}")

    perception_task = asyncio.create_task(
        perception_scan(
            logger,
            duration_s=10,
            label=label,
            stop_on_detection=True,
        )
    )

    found = await yaw_for_or_until_detection(
        drone,
        perception_task,
        yaw_rate=12.0,
        duration_s=4,
    )

    if not found:
        found = await yaw_for_or_until_detection(
            drone,
            perception_task,
            yaw_rate=-12.0,
            duration_s=8,
        )

    if not found:
        found = await yaw_for_or_until_detection(
            drone,
            perception_task,
            yaw_rate=12.0,
            duration_s=4,
        )

    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )

    if not perception_task.done():
        summary, found_from_task = await perception_task
        found = found or found_from_task
    else:
        summary, found_from_task = perception_task.result()
        found = found or found_from_task

    if found:
        print(f"[{label}] New fuel detected. Proceeding to next search area.")
    else:
        print(f"[{label}] No fuel detected. Continuing exploration.")

    return summary


async def main():
    print("Starting Gazebo camera subscriber...")
    camera_node = Node()
    camera_node.subscribe(Image, ism.IMAGE_TOPIC, ism.image_callback)

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
        print("Starting autonomous map exploration...")

        await yaw_scan(drone, logger, "scan_start_mid_altitude")

        await set_velocity_for(
            drone,
            vx=0.4,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.0,
            duration_s=6,
            label="lane1_forward",
            obstacle_monitor=obstacle_monitor,
        )
        await yaw_scan(drone, logger, "scan_lane1_end")

        await set_velocity_for(
            drone,
            vx=0.0,
            vy=0.4,
            vz=0.0,
            yaw_rate=0.0,
            duration_s=4,
            label="shift_right_to_lane2",
            obstacle_monitor=obstacle_monitor,
        )
        await yaw_scan(drone, logger, "scan_lane2_start")

        await set_velocity_for(
            drone,
            vx=-0.4,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.0,
            duration_s=6,
            label="lane2_backward",
            obstacle_monitor=obstacle_monitor,
        )
        await yaw_scan(drone, logger, "scan_lane2_end")

        await set_velocity_for(
            drone,
            vx=0.0,
            vy=0.4,
            vz=0.0,
            yaw_rate=0.0,
            duration_s=4,
            label="shift_right_to_lane3",
            obstacle_monitor=obstacle_monitor,
        )
        await yaw_scan(drone, logger, "scan_lane3_start")

        await set_velocity_for(
            drone,
            vx=0.4,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.0,
            duration_s=6,
            label="lane3_forward",
            obstacle_monitor=obstacle_monitor,
        )
        await yaw_scan(drone, logger, "scan_lane3_end")

        await change_altitude_for(
            drone,
            vz=-0.20,
            duration_s=4,
            label="move_up_for_elevated_red_targets",
        )
        await yaw_scan(drone, logger, "scan_high_altitude")

        await set_velocity_for(
            drone,
            vx=-0.3,
            vy=-0.3,
            vz=0.0,
            yaw_rate=0.0,
            duration_s=5,
            label="diagonal_high_altitude_reposition",
            obstacle_monitor=obstacle_monitor,
        )
        await yaw_scan(drone, logger, "scan_high_altitude_repositioned")

        await change_altitude_for(
            drone,
            vz=0.20,
            duration_s=5,
            label="move_down_for_ground_yellow_targets",
        )
        await yaw_scan(drone, logger, "scan_low_altitude")

        await set_velocity_for(
            drone,
            vx=0.3,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.0,
            duration_s=5,
            label="final_low_altitude_forward",
            obstacle_monitor=obstacle_monitor,
        )
        await yaw_scan(drone, logger, "scan_final")

        final_summary = logger.summary()
        final_score = calculate_score(final_summary)

        print("Final search summary:")
        print(final_summary)
        print(f"Final score estimate: {final_score}")

    finally:
        print("Stopping offboard mode...")
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            print(f"Stopping offboard failed: {error._result.result}")

        print("Landing...")
        await drone.action.land()

        cv2.destroyAllWindows()

    _ = camera_node
    _ = depth_node

    print("Autonomous search mission complete.")


if __name__ == "__main__":
    asyncio.run(main())
