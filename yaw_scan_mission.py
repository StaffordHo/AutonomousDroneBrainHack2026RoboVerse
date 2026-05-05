import asyncio
import math
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

import cv2

import integrated_stationary_mission as ism
from bearing_detection_logger import BearingDetectionLogger, normalize_angle_deg


IMAGE_WIDTH = 1920
IMX214_HORIZONTAL_FOV_DEG = math.degrees(1.204)

latest_yaw_deg = None


def estimate_bearing_deg(detection, drone_yaw_deg):
    cx, _ = detection["center"]

    # Normalized horizontal position:
    # left edge ≈ -0.5, image centre = 0, right edge ≈ +0.5
    normalized_x = (cx - (IMAGE_WIDTH / 2.0)) / IMAGE_WIDTH

    # Convert image offset into camera horizontal angle
    camera_offset_deg = normalized_x * IMX214_HORIZONTAL_FOV_DEG

    # Approximate world bearing
    return normalize_angle_deg(drone_yaw_deg + camera_offset_deg)


async def yaw_reader(drone):
    global latest_yaw_deg

    async for attitude in drone.telemetry.attitude_euler():
        latest_yaw_deg = attitude.yaw_deg


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


async def bearing_perception_loop(duration_s=32):
    logger = BearingDetectionLogger(
        bearing_threshold_deg=8.0,
        confirmation_frames=3,
    )

    print(f"Running bearing-aware perception loop for {duration_s} seconds...")

    start_time = time.time()
    last_print_time = 0
    saved_once = False
    latest_detections = []

    while time.time() - start_time < duration_s:
        if ism.latest_frame is None or latest_yaw_deg is None:
            await asyncio.sleep(0.05)
            continue

        frame = ism.latest_frame.copy()
        detections = ism.detect_barrels(frame)

        bearing_detections = []
        for det in detections:
            det_with_bearing = det.copy()
            det_with_bearing["bearing_deg"] = estimate_bearing_deg(det, latest_yaw_deg)
            bearing_detections.append(det_with_bearing)

        latest_detections = bearing_detections

        new_confirmations = logger.update(bearing_detections)

        for confirmed in new_confirmations:
            print(
                f"NEW confirmed detection: {confirmed['colour']} barrel "
                f"bearing={confirmed['bearing_deg']:.1f} deg "
                f"center={confirmed['center']}"
            )

        summary = logger.summary()

        if new_confirmations and not saved_once:
            ism.save_evidence(frame, bearing_detections, summary, "yaw_bearing_first_confirmations")
            saved_once = True

        now = time.time()
        if now - last_print_time > 1.0:
            print(
                f"Yaw={latest_yaw_deg:.1f} deg | "
                f"Confirmed barrels: red={summary['red']}, "
                f"yellow={summary['yellow']}, total={summary['total']}"
            )
            last_print_time = now

        output = ism.draw_detections(frame, bearing_detections, summary)

        cv2.putText(
            output,
            f"Yaw: {latest_yaw_deg:.1f} deg",
            (30, 95),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 255, 255),
            3,
        )

        cv2.imshow("Bearing-aware Yaw Scan Mission", output)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        await asyncio.sleep(0.03)

    final_summary = logger.summary()

    if ism.latest_frame is not None:
        ism.save_evidence(
            ism.latest_frame.copy(),
            latest_detections,
            final_summary,
            "yaw_bearing_final_summary",
        )

    cv2.destroyAllWindows()

    return final_summary


async def main():
    print("Starting Gazebo camera subscriber...")
    node = Node()
    node.subscribe(Image, ism.IMAGE_TOPIC, ism.image_callback)

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    await wait_connected(drone)
    await wait_local_position(drone)

    print("Starting yaw telemetry reader...")
    yaw_task = asyncio.create_task(yaw_reader(drone))

    print("Setting takeoff altitude...")
    await drone.action.set_takeoff_altitude(2.5)

    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()

    print("Waiting for takeoff to stabilise...")
    await asyncio.sleep(8)

    print("Setting initial offboard velocity setpoint...")
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
        yaw_task.cancel()
        return

    print("Starting bearing-aware perception task...")
    perception_task = asyncio.create_task(bearing_perception_loop(duration_s=32))

    print("Yaw scan right for 8 seconds...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 15.0)
    )
    await asyncio.sleep(8)

    print("Yaw scan left for 16 seconds...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, -15.0)
    )
    await asyncio.sleep(16)

    print("Yaw scan right/center for 8 seconds...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 15.0)
    )
    await asyncio.sleep(8)

    print("Hovering...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )

    summary = await perception_task

    print("Stopping offboard mode...")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard failed: {error._result.result}")

    print("Perception summary:")
    print(summary)

    print("Landing...")
    await drone.action.land()

    yaw_task.cancel()

    print("Bearing-aware yaw scan mission complete.")


if __name__ == "__main__":
    asyncio.run(main())
