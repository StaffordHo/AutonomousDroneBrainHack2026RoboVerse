import asyncio
import os
import time
import math
import argparse
from datetime import datetime

import cv2
import numpy as np

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed, PositionNedYaw, VelocityNedYaw
from mavsdk.action import ActionError

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

from small_fuel_detector import detect_small_fuel_barrels
from bearing_detection_logger import BearingDetectionLogger, normalize_angle_deg
from obstacle_monitor import create_obstacle_monitor

from AvoidancePlanner import AvoidancePlanner
from GlobalMapper import GlobalMapper

IMAGE_TOPIC = "/world/roboverse/model/x500_depth_0/link/camera_link/sensor/IMX214/image"
EVIDENCE_DIR = "competition_evidence"

YELLOW_SCORE = 50
RED_SCORE = 100
MAX_MISSION_TIME_S = 9 * 60  # 9 minutes

IMAGE_WIDTH = 1920
IMX214_HORIZONTAL_FOV_DEG = math.degrees(1.204)

# Global State
latest_frame = None
latest_yaw_deg = None
latest_position_ned = None
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
    normalized_x = (cx - (IMAGE_WIDTH / 2.0)) / IMAGE_WIDTH
    camera_offset_deg = normalized_x * IMX214_HORIZONTAL_FOV_DEG
    return normalize_angle_deg(drone_yaw_deg + camera_offset_deg)

def image_callback(msg: Image):
    global latest_frame
    width = msg.width
    height = msg.height
    img = np.frombuffer(msg.data, dtype=np.uint8)
    img = img.reshape((height, width, 3))
    latest_frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

async def telemetry_reader(drone):
    global latest_position_ned
    async for pos_vel in drone.telemetry.position_velocity_ned():
        latest_position_ned = pos_vel.position
        
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
        cv2.putText(output, label, (x, max(y - 10, 30)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, box_colour, 2)

    score = calculate_score(summary)
    cv2.putText(output, f"Score: {score} | Confirmed: red={summary['red']} yellow={summary['yellow']}",
                (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3)
    elapsed = get_elapsed_time()
    cv2.putText(output, f"Time: {elapsed//60:.0f}m {elapsed%60:.0f}s / 10m",
                (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
    return output

def save_evidence(frame, detections, summary, label):
    os.makedirs(EVIDENCE_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = draw_detections(frame, detections, summary)
    image_path = os.path.join(EVIDENCE_DIR, f"{timestamp}_{label}.png")
    cv2.imwrite(image_path, output)
    print(f"Saved evidence image: {image_path} (Score: {calculate_score(summary)})")

async def perception_scan(logger, args, duration_s=5, label="scan", stop_on_detection=True, obstacle_monitor=None, mapper=None):
    print(f"Perception scan: {label} for {duration_s}s")
    start_time = time.time()
    last_print_time = 0
    found_new_detection = False
    latest_detections = []

    while time.time() - start_time < duration_s:
        if check_timeout():
            break

        if latest_frame is None or latest_yaw_deg is None or latest_position_ned is None:
            await asyncio.sleep(0.05)
            continue

        # Update mapper if provided
        if mapper is not None and obstacle_monitor is not None and obstacle_monitor.latest_depth is not None:
            pose = {
                'north': latest_position_ned.north_m,
                'east': latest_position_ned.east_m,
                'yaw': math.radians(latest_yaw_deg),
                'down': latest_position_ned.down_m
            }
            mapper.update_frame(obstacle_monitor.latest_depth, pose)

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
            save_evidence(frame, bearing_detections, summary, f"{label}_new_fuel")
            if stop_on_detection:
                return summary, True

        now = time.time()
        if now - last_print_time > 1.0:
            print(f"[{label}] Elapsed: {get_elapsed_time():.0f}s | red={summary['red']} yellow={summary['yellow']}")
            last_print_time = now

        if not args.headless:
            output = draw_detections(frame, bearing_detections, summary)
            cv2.imshow("RoboVerse", output)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        await asyncio.sleep(0.05)

    summary = logger.summary()
    if latest_frame is not None and not found_new_detection:
        save_evidence(latest_frame.copy(), latest_detections, summary, f"{label}_summary")
    return summary, found_new_detection

async def navigate_to_waypoint(drone, target_n, target_e, target_d, obstacle_monitor, avoid_planner, global_mapper, label="nav"):
    print(f"Navigating to {target_n:.1f}N, {target_e:.1f}E (Label: {label})")
    
    # Send a small velocity initially to engage offboard correctly
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    
    while not check_timeout():
        if latest_position_ned is None or latest_yaw_deg is None:
            await asyncio.sleep(0.1)
            continue
            
        current_n = latest_position_ned.north_m
        current_e = latest_position_ned.east_m
        current_d = latest_position_ned.down_m
        
        distance_to_target = math.sqrt((target_n - current_n)**2 + (target_e - current_e)**2)
        
        if distance_to_target < 1.5:  # Arrival threshold
            print(f"[{label}] Arrived at waypoint. (Distance {distance_to_target:.1f}m < 1.5m)")
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
            return True
            
        depth_img = obstacle_monitor.latest_depth
        if depth_img is not None:
            pose = {
                'north': current_n,
                'east': current_e,
                'yaw': math.radians(latest_yaw_deg),
                'down': current_d
            }
            
            # 1. Update Global Mapper
            global_mapper.update_frame(depth_img, pose)
            
            # 2. Compute Avoidance Velocity Setpoint
            vx, vy, info = avoid_planner.compute_velocity(
                depth_map=depth_img,
                pose=pose,
                target_n=target_n,
                target_e=target_e
            )
            
            # Calculate altitude correction (vz) in strict NED frame
            # Down is positive. If target_d is -3.5 and current_d is -3.0, we want to go UP (negative vz).
            vz_error = target_d - current_d
            vz = max(min(vz_error * 1.0, 1.0), -1.0) # Clamp vertical speed to 1 m/s
            
            # 3. Yaw Control (Face the target or the safe avoidance path)
            # Rationale: If the path is blocked or we need to steer heavily, 
            # we turn the camera to face the safest direction instead of blindly strafing.
            target_yaw_rad = math.atan2(target_e - current_e, target_n - current_n)
            blocked = info.get('blocked', False)
            
            # If blocked or steering more than 20 degrees off-goal, face the safe path
            if blocked or abs(info['best_angle']) > math.radians(20):
                target_yaw_rad = pose["yaw"] + info['best_angle']
                
            yaw_error_rad = target_yaw_rad - pose["yaw"]
            
            # Normalize yaw error to [-pi, pi]
            while yaw_error_rad > math.pi: yaw_error_rad -= 2 * math.pi
            while yaw_error_rad < -math.pi: yaw_error_rad += 2 * math.pi
                
            # If we are very far off in yaw (e.g. > 30 deg), don't move forward until we face it
            if abs(yaw_error_rad) > math.radians(30):
                vx = 0.0
                vy = 0.0

            # Convert Body velocities (vx, vy) to NED velocities (vn, ve)
            yaw = pose["yaw"]
            vn = vx * math.cos(yaw) - vy * math.sin(yaw)
            ve = vx * math.sin(yaw) + vy * math.cos(yaw)

            # Map boundary geofence (prevent flying out of 40x40 map)
            if current_n > 18.0 and vn > 0: vn = 0.0
            if current_n < -18.0 and vn < 0: vn = 0.0
            if current_e > 18.0 and ve > 0: ve = 0.0
            if current_e < -18.0 and ve < 0: ve = 0.0
            
            # Send the strict world-frame velocity setpoint
            target_yaw_deg = math.degrees(target_yaw_rad)
            await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vz, target_yaw_deg))
            
        await asyncio.sleep(0.1) # 10Hz update rate
        
    return False

async def yaw_for_or_until_detection(drone, perception_task, yaw_rate, duration_s, obstacle_monitor, mapper):
    step_s = 0.25
    elapsed = 0.0
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, yaw_rate))
    
    while elapsed < duration_s:
        if check_timeout():
            break
            
        # Update map during yaw
        if latest_position_ned is not None and latest_yaw_deg is not None and obstacle_monitor.latest_depth is not None:
            pose = {
                'north': latest_position_ned.north_m,
                'east': latest_position_ned.east_m,
                'yaw': math.radians(latest_yaw_deg),
                'down': latest_position_ned.down_m
            }
            mapper.update_frame(obstacle_monitor.latest_depth, pose)
            
        if perception_task.done():
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            _, found_new_detection = perception_task.result()
            return found_new_detection
            
        await asyncio.sleep(step_s)
        elapsed += step_s
        
    return False

async def yaw_scan(drone, logger, args, label, obstacle_monitor, mapper):
    print(f"Yaw scan: {label}")
    
    perception_task = asyncio.create_task(
        perception_scan(logger, args, duration_s=12, label=label, stop_on_detection=True, obstacle_monitor=obstacle_monitor, mapper=mapper)
    )

    found = await yaw_for_or_until_detection(drone, perception_task, yaw_rate=15.0, duration_s=4, obstacle_monitor=obstacle_monitor, mapper=mapper)
    if not found:
        found = await yaw_for_or_until_detection(drone, perception_task, yaw_rate=-15.0, duration_s=8, obstacle_monitor=obstacle_monitor, mapper=mapper)
    if not found:
        found = await yaw_for_or_until_detection(drone, perception_task, yaw_rate=15.0, duration_s=4, obstacle_monitor=obstacle_monitor, mapper=mapper)

    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

    if not perception_task.done():
        summary, found_from_task = await perception_task
    else:
        summary, found_from_task = perception_task.result()

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

    # --- Setup Planners ---
    # Gazebo depth_camera for x500_depth is 640x480.
    K = np.array([[433.0, 0.0, 320.0],
                  [0.0, 433.0, 240.0],
                  [0.0, 0.0, 1.0]])
                  
    avoid_planner = AvoidancePlanner(K=K, width=640, height=480, max_speed=1.5, safe_distance=4.0, critical_distance=1.5)
    # Using yaw_in_degrees=False because we pass math.radians() to the mapper
    global_mapper = GlobalMapper(K=K, cam_height=1.0, obs_h_min=0.1, obs_h_max=1.5, yaw_in_degrees=False, yaw_smoothing=0.8, z_min=0.3, z_max=8.0)

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    await wait_connected(drone)
    await wait_local_position(drone)

    print("Starting telemetry readers...")
    pos_task = asyncio.create_task(telemetry_reader(drone))
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
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

    print("Starting offboard mode...")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard failed: {error}")
        await drone.action.land()
        yaw_task.cancel()
        pos_task.cancel()
        return

    logger = BearingDetectionLogger(bearing_threshold_deg=8.0, confirmation_frames=3)

    try:
        # Get start position
        while latest_position_ned is None:
            await asyncio.sleep(0.1)
        start_n = latest_position_ned.north_m
        start_e = latest_position_ned.east_m

        # ---------------------------------------------------------
        # PASS 1: Mid-Altitude Systematic Search (Ground focus)
        # We define a strict NED waypoint pattern relative to start
        # ---------------------------------------------------------
        print("--- BEGIN PASS 1 (Mid-Altitude) ---")
        alt_1 = -3.5
        await yaw_scan(drone, logger, args, "pass1_start", obstacle_monitor, global_mapper)

        # Lane waypoints (Local NED relative to start)
        # 40x40 arena: We fly 30 meters forward, 4 lanes wide (18m total shift)
        L = 30.0
        W = 6.0
        waypoints_pass1 = [
            (start_n + L, start_e),               # Fwd lane 0
            (start_n + L, start_e + W),           # Shift right
            (start_n,     start_e + W),           # Back lane 1
            (start_n,     start_e + W*2),         # Shift right
            (start_n + L, start_e + W*2),         # Fwd lane 2
            (start_n + L, start_e + W*3),         # Shift right
            (start_n,     start_e + W*3),         # Back lane 3
        ]

        for i, (wp_n, wp_e) in enumerate(waypoints_pass1):
            if check_timeout(): break
            await navigate_to_waypoint(drone, wp_n, wp_e, alt_1, obstacle_monitor, avoid_planner, global_mapper, label=f"pass1_wp_{i}")
            await yaw_scan(drone, logger, args, f"pass1_scan_wp_{i}", obstacle_monitor, global_mapper)

        # ---------------------------------------------------------
        # PASS 2: High-Altitude Return (Elevated targets focus)
        # ---------------------------------------------------------
        if not check_timeout():
            print("--- BEGIN PASS 2 (High-Altitude) ---")
            alt_2 = -5.5
            
            # Ascend in place (at the end of pass 1)
            await navigate_to_waypoint(drone, latest_position_ned.north_m, latest_position_ned.east_m, alt_2, obstacle_monitor, avoid_planner, global_mapper, label="climb_pass2")
            await yaw_scan(drone, logger, args, "pass2_start", obstacle_monitor, global_mapper)

            # Diagonal return across the covered area
            await navigate_to_waypoint(drone, start_n + (L/2), start_e + (W*1.5), alt_2, obstacle_monitor, avoid_planner, global_mapper, label="pass2_return_mid")
            await yaw_scan(drone, logger, args, "pass2_scan_mid", obstacle_monitor, global_mapper)

            await navigate_to_waypoint(drone, start_n, start_e, alt_2, obstacle_monitor, avoid_planner, global_mapper, label="pass2_return_end")
            await yaw_scan(drone, logger, args, "pass2_end", obstacle_monitor, global_mapper)

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
        
        # Save Global Map
        global_mapper.save_points("global_obstacles.npy")

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
        pos_task.cancel()


if __name__ == "__main__":
    asyncio.run(main())
