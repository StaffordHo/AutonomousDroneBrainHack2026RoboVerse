import asyncio
import os
import time
import math
import argparse
from datetime import datetime

import cv2
import numpy as np

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from mavsdk.action import ActionError

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

from small_fuel_detector import detect_small_fuel_barrels
from bearing_detection_logger import BearingDetectionLogger, normalize_angle_deg
from obstacle_monitor import create_obstacle_monitor

IMAGE_TOPIC = "/world/roboverse/model/x500_depth_0/link/camera_link/sensor/IMX214/image"
EVIDENCE_DIR = "competition_evidence"

YELLOW_SCORE = 50
RED_SCORE = 100
MAX_MISSION_TIME_S = 9 * 60  # 9 minutes (leaves 1 min buffer for landing and scoring)

IMAGE_WIDTH = 1920
IMX214_HORIZONTAL_FOV_DEG = math.degrees(1.204)

# Global State
latest_frame = None
latest_yaw_deg = None
mission_start_time = None


def get_elapsed_time():
    if mission_start_time is None:
        return 0
    return time.time() - mission_start_time


def check_timeout():
    if get_elapsed_time() > MAX_MISSION_TIME_S:
        print("WARNING: Mission hard timeout reached! Aborting current action to land.")
        return True
    return False


def calculate_score(summary):
    return summary["yellow"] * YELLOW_SCORE + summary["red"] * RED_SCORE


def estimate_bearing_deg(detection, drone_yaw_deg):
    cx, _ = detection["center"]
    
    # Normalized horizontal position: left edge ≈ -0.5, image centre = 0, right edge ≈ +0.5
    normalized_x = (cx - (IMAGE_WIDTH / 2.0)) / IMAGE_WIDTH
    
    # Convert image offset into camera horizontal angle
    camera_offset_deg = normalized_x * IMX214_HORIZONTAL_FOV_DEG
    
    # Approximate world bearing
    return normalize_angle_deg(drone_yaw_deg + camera_offset_deg)


def image_callback(msg: Image):
    global latest_frame
    width = msg.width
    height = msg.height
    img = np.frombuffer(msg.data, dtype=np.uint8)
    img = img.reshape((height, width, 3))
    latest_frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


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
        if health.is_local_position_ok:
            print("Local position OK.")
            return


async def arm_with_retry(drone, max_attempts=10, retry_delay=2.0):
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


def draw_detections(frame, detections, summary):
    output = frame.copy()
    
    for det in detections:
        x, y, w, h = det["bbox"]
        cx, cy = det["center"]
        colour = det["colour"]
        
        box_colour = (0, 0, 255) if colour == "red" else (0, 255, 255)
        
        cv2.rectangle(output, (x, y), (x + w, y + h), box_colour, 3)
        cv2.circle(output, (cx, cy), 5, box_colour, -1)
        
        bearing = det.get("bearing_deg", 0.0)
        label = f"{colour} b:{bearing:.0f}"
        
        cv2.putText(
            output,
            label,
            (x, max(y - 10, 30)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            box_colour,
            2,
        )

    score = calculate_score(summary)
    
    # Draw summary panel
    cv2.putText(
        output,
        f"Score: {score} | Confirmed: red={summary['red']} yellow={summary['yellow']}",
        (30, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (255, 255, 255),
        3,
    )
    
    elapsed = get_elapsed_time()
    cv2.putText(
        output,
        f"Time: {elapsed//60:.0f}m {elapsed%60:.0f}s / 10m",
        (30, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (200, 200, 200),
        2,
    )

    return output


def save_evidence(frame, detections, summary, label):
    os.makedirs(EVIDENCE_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = draw_detections(frame, detections, summary)
    image_path = os.path.join(EVIDENCE_DIR, f"{timestamp}_{label}.png")
    cv2.imwrite(image_path, output)
    print(f"Saved evidence image: {image_path} (Score: {calculate_score(summary)})")


async def perception_scan(logger, args, duration_s=5, label="scan", stop_on_detection=True):
    print(f"Perception scan: {label} for {duration_s}s")
    
    start_time = time.time()
    last_print_time = 0
    found_new_detection = False
    latest_detections = []

    while time.time() - start_time < duration_s:
        if check_timeout():
            break

        if latest_frame is None or latest_yaw_deg is None:
            await asyncio.sleep(0.05)
            continue

        frame = latest_frame.copy()
        
        raw_detections, _, _, _ = detect_small_fuel_barrels(frame)
        
        bearing_detections = []
        for det in raw_detections:
            det_with_bearing = det.copy()
            det_with_bearing["bearing_deg"] = estimate_bearing_deg(det, latest_yaw_deg)
            bearing_detections.append(det_with_bearing)

        latest_detections = bearing_detections
        new_confirmations = logger.update(bearing_detections)
        summary = logger.summary()

        if new_confirmations:
            found_new_detection = True
            for confirmed in new_confirmations:
                print(
                    f"NEW confirmed fuel: {confirmed['colour']} barrel "
                    f"at world bearing {confirmed['bearing_deg']:.1f} deg"
                )
            
            save_evidence(frame, bearing_detections, summary, f"{label}_new_fuel")
            
            if stop_on_detection:
                print(f"[{label}] Fuel found. Ending scan early to continue search.")
                return summary, True

        now = time.time()
        if now - last_print_time > 1.0:
            print(
                f"[{label}] Elapsed: {get_elapsed_time():.0f}s | "
                f"red={summary['red']} yellow={summary['yellow']} "
                f"score={calculate_score(summary)}"
            )
            last_print_time = now

        if not args.headless:
            output = draw_detections(frame, bearing_detections, summary)
            cv2.imshow("RoboVerse Competition Mission", output)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        await asyncio.sleep(0.05)

    summary = logger.summary()
    
    # Save a final state image for this scan if we didn't save a new detection
    if latest_frame is not None and not found_new_detection:
        save_evidence(latest_frame.copy(), latest_detections, summary, f"{label}_summary")

    return summary, found_new_detection


async def set_velocity_for(drone, vx, vy, vz, yaw_rate, duration_s, label, obstacle_monitor=None):
    print(f"Movement: {label}")
    step_s = 0.5
    elapsed = 0.0

    while elapsed < duration_s:
        if check_timeout():
            return False

        if obstacle_monitor is not None:
            too_close, distance = obstacle_monitor.obstacle_too_close()
            if too_close:
                print(f"[{label}] Obstacle too close at {distance:.2f} m. Stopping and avoiding.")
                await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                await asyncio.sleep(0.5)
                # Yaw to find a clear path
                await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 30.0))
                await asyncio.sleep(2.0)
                await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                await asyncio.sleep(1.0)
                return False

        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx, vy, vz, yaw_rate))
        await asyncio.sleep(step_s)
        elapsed += step_s

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(1.0)
    return True


async def yaw_for_or_until_detection(drone, perception_task, yaw_rate, duration_s):
    step_s = 0.25
    elapsed = 0.0
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, yaw_rate))
    
    while elapsed < duration_s:
        if check_timeout():
            break
            
        if perception_task.done():
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            _, found_new_detection = perception_task.result()
            return found_new_detection
            
        await asyncio.sleep(step_s)
        elapsed += step_s
        
    return False


async def yaw_scan(drone, logger, args, label):
    print(f"Yaw scan: {label}")
    
    perception_task = asyncio.create_task(
        perception_scan(logger, args, duration_s=12, label=label, stop_on_detection=True)
    )

    # Sweep right, then left, then right to center
    found = await yaw_for_or_until_detection(drone, perception_task, yaw_rate=15.0, duration_s=4)
    if not found:
        found = await yaw_for_or_until_detection(drone, perception_task, yaw_rate=-15.0, duration_s=8)
    if not found:
        found = await yaw_for_or_until_detection(drone, perception_task, yaw_rate=15.0, duration_s=4)

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    if not perception_task.done():
        summary, found_from_task = await perception_task
        found = found or found_from_task
    else:
        summary, found_from_task = perception_task.result()
        found = found or found_from_task

    return summary


async def main():
    global mission_start_time

    parser = argparse.ArgumentParser(description="RoboVerse 2026 Qualifier Competition Script")
    parser.add_argument("--headless", action="store_true", help="Run without OpenCV GUI window")
    args = parser.parse_args()

    print("Starting Gazebo camera subscriber...")
    camera_node = Node()
    camera_node.subscribe(Image, IMAGE_TOPIC, image_callback)

    print("Starting depth obstacle monitor...")
    depth_node, obstacle_monitor = create_obstacle_monitor()

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    await wait_connected(drone)
    await wait_local_position(drone)

    print("Starting yaw telemetry reader...")
    yaw_task = asyncio.create_task(yaw_reader(drone))

    print("Setting takeoff altitude...")
    await drone.action.set_takeoff_altitude(3.5)

    print("Arming...")
    armed = await arm_with_retry(drone)
    if not armed:
        print("Mission aborted: arming failed.")
        return

    mission_start_time = time.time()

    print("Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(8)

    print("Setting initial offboard setpoint...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    print("Starting offboard mode...")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard failed: {error}")
        await drone.action.land()
        yaw_task.cancel()
        return

    logger = BearingDetectionLogger(bearing_threshold_deg=8.0, confirmation_frames=3)

    try:
        # ---------------------------------------------------------
        # PASS 1: Mid-Altitude Systematic Search (Ground focus)
        # ---------------------------------------------------------
        print("--- BEGIN PASS 1 (Mid-Altitude) ---")
        await yaw_scan(drone, logger, args, "pass1_start")

        # Fly 3 parallel lanes, faster than before
        for lane in range(3):
            if check_timeout(): break
            
            # Forward leg
            await set_velocity_for(
                drone, vx=1.2, vy=0.0, vz=0.0, yaw_rate=0.0, duration_s=12, 
                label=f"pass1_lane_{lane}_fwd", obstacle_monitor=obstacle_monitor
            )
            await yaw_scan(drone, logger, args, f"pass1_scan_{lane}_fwd")
            
            # Shift right
            if lane < 2 and not check_timeout():
                await set_velocity_for(
                    drone, vx=0.0, vy=1.0, vz=0.0, yaw_rate=0.0, duration_s=5, 
                    label=f"pass1_shift_right_{lane}", obstacle_monitor=obstacle_monitor
                )
                await yaw_scan(drone, logger, args, f"pass1_scan_shift_{lane}")
                
            # Fly backwards for the next lane to save turning time
            if lane < 2 and not check_timeout():
                await set_velocity_for(
                    drone, vx=-1.2, vy=0.0, vz=0.0, yaw_rate=0.0, duration_s=12, 
                    label=f"pass1_lane_{lane}_back", obstacle_monitor=obstacle_monitor
                )
                await yaw_scan(drone, logger, args, f"pass1_scan_{lane}_back")
                
                # Shift right again for next lane
                await set_velocity_for(
                    drone, vx=0.0, vy=1.0, vz=0.0, yaw_rate=0.0, duration_s=5, 
                    label=f"pass1_shift_right_back_{lane}", obstacle_monitor=obstacle_monitor
                )
                await yaw_scan(drone, logger, args, f"pass1_scan_shift_back_{lane}")

        # ---------------------------------------------------------
        # PASS 2: High-Altitude Return (Elevated targets focus)
        # ---------------------------------------------------------
        if not check_timeout():
            print("--- BEGIN PASS 2 (High-Altitude) ---")
            
            # Climb up by 2 meters
            await set_velocity_for(
                drone, vx=0.0, vy=0.0, vz=-2.0, yaw_rate=0.0, duration_s=4, 
                label="climb_pass2"
            )
            await yaw_scan(drone, logger, args, "pass2_start")

            # Diagonal return across the map
            await set_velocity_for(
                drone, vx=-1.0, vy=-1.0, vz=0.0, yaw_rate=0.0, duration_s=12, 
                label="pass2_return_1", obstacle_monitor=obstacle_monitor
            )
            await yaw_scan(drone, logger, args, "pass2_return_mid")
            
            await set_velocity_for(
                drone, vx=-1.0, vy=-1.0, vz=0.0, yaw_rate=0.0, duration_s=12, 
                label="pass2_return_2", obstacle_monitor=obstacle_monitor
            )
            await yaw_scan(drone, logger, args, "pass2_end")

        # Final Summary Print
        final_summary = logger.summary()
        final_score = calculate_score(final_summary)
        print("\n=================================")
        print("MISSION COMPLETE - FINAL SUMMARY")
        print("=================================")
        print(f"Time elapsed: {get_elapsed_time():.0f} seconds")
        print(f"Red Barrels (100pt): {final_summary['red']}")
        print(f"Yellow Barrels (50pt): {final_summary['yellow']}")
        print(f"TOTAL SCORE: {final_score}")
        print("=================================\n")

    finally:
        print("Stopping offboard mode...")
        try:
            await drone.offboard.stop()
        except OffboardError:
            pass

        print("Landing...")
        await drone.action.land()
        
        if not args.headless:
            cv2.destroyAllWindows()
            
        yaw_task.cancel()


if __name__ == "__main__":
    asyncio.run(main())
