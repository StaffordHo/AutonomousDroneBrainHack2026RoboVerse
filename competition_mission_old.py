import asyncio
import os
import time
import math
import numpy as np
import cv2
from datetime import datetime

from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw, OffboardError

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

from small_fuel_detector import detect_small_fuel_barrels
from bearing_detection_logger import BearingDetectionLogger, normalize_angle_deg
from obstacle_monitor import ObstacleMonitor
from depth_debugger import save_depth_debug

# --- CONFIG ---
EVIDENCE_DIR = "competition_evidence"
ALTITUDE = -1.5 
IMAGE_WIDTH = 1920
FOV_DEG = 69.0

# --- GLOBAL STATE ---
latest_frame = None
latest_position_ned = None
latest_attitude = {"p": 0.0, "r": 0.0, "y": 0.0}
START_N, START_E = 0.0, 0.0
score = 0

def image_callback(msg: Image):
    global latest_frame
    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    latest_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

def depth_callback(msg: Image):
    data = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    monitor.update_depth(data)

def estimate_bearing_deg(det):
    cx, _ = det["center"]
    norm_x = (cx - IMAGE_WIDTH / 2.0) / IMAGE_WIDTH
    cam_offset = norm_x * FOV_DEG
    return normalize_angle_deg(latest_attitude["y"] + cam_offset)

def refine_bbox(det, img_shape):
    x, y, w, h = det["bbox"]
    img_h, img_w = img_shape[:2]
    
    # Heuristic: Canisters are usually slightly wider than the detected red core
    if det["colour"] == "red":
        x = max(0, x - int(0.20 * w))
        w = int(1.40 * w)
        h = int(1.10 * h)
    elif det["colour"] == "yellow":
        x = max(0, x - int(0.10 * w))
        w = int(1.20 * w)
        h = int(1.10 * h)
        
    w = min(w, img_w - x)
    h = min(h, img_h - y)
    det["bbox"] = (x, y, w, h)
    return det

async def scan_and_capture(drone, logger, label=""):
    global score
    print(f"🔭 [{label}] Scanning for barrels...")
    # Rotate 360 in 6 steps
    for angle in range(0, 361, 60):
        target_yaw = normalize_angle_deg(latest_attitude["y"] + angle)
        await drone.offboard.set_position_ned(PositionNedYaw(latest_position_ned.north_m, latest_position_ned.east_m, ALTITUDE, target_yaw))
        await asyncio.sleep(2.0) # Wait for stability
        
        if latest_frame is not None:
            detections, _, _, _ = detect_small_fuel_barrels(latest_frame)
            for det in detections:
                det["bearing_deg"] = estimate_bearing_deg(det)
                det["n"] = latest_position_ned.north_m
                det["e"] = latest_position_ned.east_m
                
                if logger.add_detection(det):
                    det = refine_bbox(det, latest_frame.shape)
                    val = 100 if det["colour"] == "red" else 50
                    score += val
                    
                    # Annotate evidence
                    x, y, w, h = det["bbox"]
                    annotated = latest_frame.copy()
                    col = (0,0,255) if det["colour"] == "red" else (0,255,255)
                    cv2.rectangle(annotated, (x,y), (x+w, y+h), col, 3)
                    cv2.putText(annotated, f"CONFIRMED {det['colour']} +{val} Score:{score}", (20, 60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1.5, col, 3)
                    
                    img_path = f"{EVIDENCE_DIR}/CONFIRMED_{det['colour']}_{datetime.now().strftime('%H%M%S')}.png"
                    cv2.imwrite(img_path, annotated)
                    print(f"🎯 SAVED EVIDENCE: {img_path}")

async def navigate_safely(drone, target_n, target_e, monitor, label):
    start_time = time.time()
    blocked_count = 0
    while True:
        if time.time() - start_time > 30: return False
        curr_n, curr_e = latest_position_ned.north_m, latest_position_ned.east_m
        dist_to_goal = math.sqrt((target_n - curr_n)**2 + (target_e - curr_e)**2)
        if dist_to_goal < 0.8: return True

        while abs(latest_attitude["p"]) > 2.0 or abs(latest_attitude["r"]) > 2.0:
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0,0,0, latest_attitude["y"]))
            await asyncio.sleep(0.1)

        clearance = monitor.get_directional_clearance()
        front = clearance["center"]
        
        if front < 2.0:
            blocked_count += 1
            if front == 0.0: 
                await drone.action.land()
                return False
            
            # ESCAPE MOVE
            turn_dir = 45 if clearance["right"] > clearance["left"] else -45
            print(f"⚠️ BLOCKED. ESCAPING {turn_dir} deg...")
            esc_yaw = normalize_angle_deg(latest_attitude["y"] + turn_dir)
            await drone.offboard.set_position_ned(PositionNedYaw(curr_n, curr_e, ALTITUDE, esc_yaw))
            await asyncio.sleep(1.5)
            
            rad = math.radians(esc_yaw)
            esc_n = curr_n + 1.2 * math.cos(rad)
            esc_e = curr_e + 1.2 * math.sin(rad)
            await drone.offboard.set_position_ned(PositionNedYaw(esc_n, esc_e, ALTITUDE, esc_yaw))
            await asyncio.sleep(2.5)
            
            if blocked_count >= 3: return False
            continue

        # Step
        target_yaw_rad = math.atan2(target_e - curr_e, target_n - curr_n)
        target_yaw_deg = math.degrees(target_yaw_rad)
        step_n = curr_n + 1.0 * math.cos(target_yaw_rad)
        step_e = curr_e + 1.0 * math.sin(target_yaw_rad)
        await drone.offboard.set_position_ned(PositionNedYaw(step_n, step_e, ALTITUDE, target_yaw_deg))
        
        for _ in range(25):
            await asyncio.sleep(0.1)
            if monitor.get_directional_clearance()["center"] < 1.8: break
            if math.sqrt((latest_position_ned.north_m - step_n)**2 + (latest_position_ned.east_m - step_e)**2) < 0.2: break

async def main():
    global latest_position_ned, START_N, START_E, latest_attitude, monitor
    os.makedirs(EVIDENCE_DIR, exist_ok=True)
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    monitor = ObstacleMonitor()
    logger = BearingDetectionLogger(confirmation_frames=5)
    
    import subprocess
    all_topics = subprocess.check_output(['gz', 'topic', '-l']).decode().split()
    IMG_TOPIC = next((t for t in all_topics if "IMX214/image" in t), "/world/roboverse/model/x500_depth_0/link/camera_link/sensor/IMX214/image")
    
    node = Node()
    node.subscribe(Image, IMG_TOPIC, image_callback)
    node.subscribe(Image, "/depth_camera", depth_callback)

    async def read_telemetry():
        global latest_position_ned, latest_attitude
        async def read_pos():
            global latest_position_ned
            async for p in drone.telemetry.position_velocity_ned(): latest_position_ned = p.position
        async def read_att():
            global latest_attitude
            async for a in drone.telemetry.attitude_euler(): 
                latest_attitude = {"p": a.pitch_deg, "r": a.roll_deg, "y": a.yaw_deg}
        await asyncio.gather(read_pos(), read_att())
    asyncio.create_task(read_telemetry())

    print("Arming & Takeoff...")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(abs(ALTITUDE))
    await drone.action.takeoff()
    await asyncio.sleep(5)
    
    START_N, START_E = latest_position_ned.north_m, latest_position_ned.east_m
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0,0,0,0))
    await drone.offboard.start()

    # --- SMALLER, MORE FREQUENT GRID ---
    LIMIT = 10.0
    STEP = 5.0
    for dn in np.arange(-LIMIT, LIMIT + 1, STEP):
        for de in np.arange(-LIMIT, LIMIT + 1, STEP):
            # PRE-SCAN BEFORE MOVE
            await scan_and_capture(drone, logger, label=f"PRE:{dn},{de}")
            
            # NAVIGATE
            await navigate_safely(drone, START_N + dn, START_E + de, monitor, f"G:{dn},{de}")
            
            # POST-SCAN
            await scan_and_capture(drone, logger, label=f"POST:{dn},{de}")

    print(f"Mission Finished. Score: {score}")
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(main())
