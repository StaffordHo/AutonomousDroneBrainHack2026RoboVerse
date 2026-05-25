#!/usr/bin/env python3
import asyncio
import numpy as np
import time
import threading
import cv2
from queue import Queue
import copy
import math

from depth_receiver import DepthReceiver
from drone_control import Drone
from AvoidancePlanner import AvoidancePlanner
from get_position_with_task_v2 import Telemetry, position_monitor_task
from exploration_memory import ExplorationMemory
from gzphotodetectorsaver import GZPhotoDetectorSaver

def normalize_angle_deg(angle):
    while angle > 180:
        angle -= 360
    while angle <= -180:
        angle += 360
    return angle


class SharedState:
    def __init__(self):
        self.bridgeque = Queue()
        self.lock = threading.Lock()
    def push(self, item):
        with self.lock:
            self.bridgeque.put(item)
    def pop(self):
        with self.lock:
            if not self.bridgeque.empty():
                return self.bridgeque.get()
            else:
                return None

class DroneNavigation:
    def __init__(self, bridge: SharedState,
                 depth_topic="/depth_camera",
                 loop_hz=20.0,):

        self.loop_hz = loop_hz
        self.running = True
        self.scale = 2.0
        self.bridge = bridge

        # =========================
        # GRID HEADING SYSTEM
        # =========================
        self.grid_headings = [0, 90, 180, -90]  # N, E, S, W
        self.current_heading_idx = 0
        self.target_yaw_deg = self.grid_headings[self.current_heading_idx]
        self.yaw_tolerance = 5.0

        # =========================
        #  NED POSE TRACKING
        # =========================
        self.pose = {
            "north": 0.0,
            "east": 0.0,
            "down": -2.0,
            "yaw": 0.0,
            "yaw_deg": 0.0
        }

        # Camera intrinsics
        K = np.array([[433.0, 0.0, 320.0],
                      [0.0, 433.0, 240.0],
                      [0.0, 0.0, 1.0]])

        self.receiver = DepthReceiver(depth_topic)

        self.planner = AvoidancePlanner(
            K=K, width=640, height=480,
            max_speed=2.9,
            safe_distance=4.5,
            critical_distance=3.0,
            num_bins=48,
            smoothing_alpha=0.7
        )

        self.exploration_memory = ExplorationMemory(
            cell_size_m=1.0,
            lookahead_m=7.0,
            novelty_weight=8.0,
            revisit_penalty=50.0,
            blocked_penalty=90.0,
            turn_penalty=0.02,
        )

        self.memory_initialized = False

        self.drone = Drone()
        self.telemetry = Telemetry()

        self.cameratopic = "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image"
        self.cameradetector = GZPhotoDetectorSaver(topic=self.cameratopic, model_path="8n_2.0.pt", threshold=0.6)

        self.detect_at_stop = True   # was False
        self.capture_at_stop = True
        self._burst_mode = "capture"  # alternates each interval

        # FIX: track when we last triggered a burst so we don't hammer it at 20 Hz
        self._last_burst_time = 0.0
        self._burst_interval_s = 5.0  # trigger a capture burst at most every 5 seconds

    # =========================
    #  YAW UTILS
    # =========================
    def _yaw_error(self, target, current):
        error = target - current
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        return error

    async def align_to_grid(self):
        current_yaw = await self.drone.get_yaw()
        error = self._yaw_error(self.target_yaw_deg, current_yaw)

        if abs(error) > self.yaw_tolerance:
            print(f"Aligning to {self.target_yaw_deg}° (err={error:.2f})")
            await self.drone.rotate_to_yaw(self.target_yaw_deg)

    async def update_pose(self):
        self.pose["north"]   = self.telemetry.north
        self.pose["east"]    = self.telemetry.east
        self.pose["down"]    = self.telemetry.down
        self.pose["yaw_deg"] = self.telemetry.yaw_deg
        self.pose["yaw"]     = self.telemetry.yaw_rad

    # =========================
    # GRID TURNING
    # =========================
    async def rotate_next_direction(self, info=None):
        base_yaw = self.pose["yaw_deg"]
        step_deg = 30.0
        max_steps = int(360 / step_deg)

        # Decide initial turn direction based on current clearance
        if info is not None:
            left_clear = info["clearance"]["left"]
            right_clear = info["clearance"]["right"]
            directions = [1, -1] if right_clear >= left_clear else [-1, 1]
            print(f"Turning {'right' if directions[0] == 1 else 'left'} first (L:{left_clear:.1f}m R:{right_clear:.1f}m)")
        else:
            directions = [1, -1]

        # tmp_yaw = base_yaw % 90
        # if tmp_yaw >= 45:
        #     print("turning right")
        #     directions = [1, -1]
        # else:
        #     print("turning left")
        #     directions = [-1, 1]

        for direction in directions:
            for i in range(1, max_steps + 1):
                offset = step_deg * i * direction
                candidate = normalize_angle_deg(base_yaw + offset)

                print(f"Trying heading: {candidate:.1f}°")
                await self.drone.rotate_to_yaw(candidate)
                await asyncio.sleep(0.2)

                depth_frame = self.receiver.get_frame()
                if depth_frame is None:
                    continue

                _, _, _, info = self.planner.compute_position_ned(
                    depth_frame,
                    self.pose,
                    step_size=1.5
                )
                print(info["blocked"])
                print(info["clearance"])
                # if not info["blocked"] and info["clearance"]["center"] > self.planner.safe_distance:
                if not info["blocked"]:
                    self.target_yaw_deg = candidate
                    print(f"Clear heading found: {candidate:.1f}°")
                    return

        # Full sweep found nothing — reverse
        self.target_yaw_deg = normalize_angle_deg(base_yaw + 180)
        print(f"No clear heading found, reversing to {self.target_yaw_deg:.1f}°")
        await self.drone.rotate_to_yaw(self.target_yaw_deg)

    def choose_heading(self, depth_frame):
        """
        Use the full polar histogram from AvoidancePlanner for fine-grained
        heading selection, biased toward unvisited cells via ExplorationMemory.
        The chosen heading is always within forward arc (±90°) to prevent sideways flight.
        """
        histogram, angles, distances = self.planner.compute_histogram(depth_frame)
        left, center, right = self.planner.compute_clearance(depth_frame)

        current_yaw = self.pose["yaw_deg"]

        clearances = {
            "center": float(center),
            "left":   float(left),
            "right":  float(right),
        }

        best_heading = self.target_yaw_deg
        best_score = -1e9

        for angle_rad, distance, cost in zip(angles, distances, histogram):
            if distance < self.planner.safe_distance:
                obs_n, obs_e = self.exploration_memory._point_from(
                    self.pose["north"],
                    self.pose["east"],
                    normalize_angle_deg(self.pose["yaw_deg"] + math.degrees(angle_rad)),
                    distance
                )
                self.exploration_memory.mark_obstacle_at(obs_n, obs_e, distance)
            # Convert camera-relative angle to absolute NED heading
            angle_deg = math.degrees(angle_rad)
            candidate_heading = normalize_angle_deg(current_yaw + angle_deg)

            # Hard constraint: only consider forward-facing headings (±90° from current yaw)
            # This ensures the drone always flies where the camera is pointing
            if abs(normalize_angle_deg(candidate_heading - current_yaw)) > 90.0:
                continue

            # Skip bins that are too close
            if distance < self.planner.safe_distance:
                continue

            score = self.exploration_memory.heading_score(
                current_n=self.pose["north"],
                current_e=self.pose["east"],
                candidate_heading_deg=candidate_heading,
                current_yaw_deg=current_yaw,
                preferred_heading_deg=self.target_yaw_deg,
                clearances=clearances,
            )

            # Boost score by how far the obstacle-free distance is
            score += min(distance, 8.0) * 0.5

            # Penalise bins with high obstacle cost
            score -= cost * 20.0

            if score > best_score:
                best_score = score
                best_heading = candidate_heading

        print(f"Memory heading: {best_heading:.1f}° score={best_score:.1f}")
        return best_heading

    # =========================
    # MAIN LOOP
    # =========================
    async def run(self):
        print("\nPOSITION-BASED AUTONOMOUS AVOIDANCE NAVIGATION\n")

        await self.drone.connect()
        print("Starting position monitor.")
        self.monitor_task = asyncio.create_task(
            position_monitor_task(self.drone, self.telemetry, asyncio.Event())
        )
        await self.drone.arm_and_takeoff()

        self.cameratask = asyncio.create_task(self.cameradetector.run())

        self.exploration_memory.initialize(self.telemetry.north, self.telemetry.east)
        self.exploration_memory.mark_visited(self.telemetry.north, self.telemetry.east)
        self.memory_initialized = True

        n = self.telemetry.north
        e = self.telemetry.east
        print(f"Drone Origin Position: N: {n} E: {e} Yaw: {self.telemetry.yaw_deg}°")


        try:

            await self.drone.go_to_altitude(3.8)
            await self.drone.rotate_to_yaw(-80)

            while self.running:
                t_start = time.monotonic()

                depth_frame = self.receiver.get_frame()
                if depth_frame is not None:

                    # FIX: throttle burst triggers — only fire every _burst_interval_s seconds
                    now = time.monotonic()
                    if now - self._last_burst_time >= self._burst_interval_s:
                        if self._burst_mode == "capture" and self.capture_at_stop:
                            self.cameradetector.trigger_capture_burst()
                            self._burst_mode = "detect"
                        elif self._burst_mode == "detect" and self.detect_at_stop:
                            self.cameradetector.trigger_detection_burst()
                            self._burst_mode = "capture"
                        self._last_burst_time = now

                    await self.update_pose()

                    north, east, down, info = self.planner.compute_position_ned(
                        depth_frame,
                        self.pose,
                        step_size=1.25
                    )

                    c = info['clearance']
                    self.exploration_memory.refresh_blocked_cells_for_height(self.pose["down"])
                    # ===================================
                    #  BLOCK HANDLING
                    # ===================================
                    if info['blocked']:
                        self.exploration_memory.mark_blocked_ray_at_height(
                            self.pose["north"],
                            self.pose["east"],
                            self.pose["yaw_deg"],
                            info["clearance"]["center"],
                            self.pose["down"],
                        )
                        await self.drone.send_velocity(0, 0, 0, self.target_yaw_deg)
                        await self.rotate_next_direction(info)
                    else:
                        # await self.align_to_grid()

                        best_heading = self.choose_heading(depth_frame)

                        if abs(normalize_angle_deg(best_heading - self.target_yaw_deg)) > 15.0:
                            self.target_yaw_deg = best_heading
                            print(f"Heading updated to {self.target_yaw_deg:.1f}°")

                        # Recompute NED setpoint using the chosen heading as the new facing direction
                        yaw_rad = math.radians(self.target_yaw_deg)
                        north = self.pose["north"] + 1.5 * math.cos(yaw_rad)
                        east  = self.pose["east"]  + 1.5 * math.sin(yaw_rad)
                        down  = self.pose["down"]

                        await self.drone.send_position_setpoint(
                            north=north,
                            east=east,
                            down=down,
                            yaw_deg=self.target_yaw_deg   # drone faces the heading it moves toward
                        )

                        if self.memory_initialized:
                            self.exploration_memory.mark_visited(
                                self.pose["north"],
                                self.pose["east"],
                            )

                    # Maintain loop timing
                    elapsed = time.monotonic() - t_start
                    sleep_time = (1.0 / self.loop_hz) - elapsed
                    if sleep_time > 0:
                        await asyncio.sleep(sleep_time)

        except Exception as e:
            print(f"Error in navigation loop: {type(e).__name__}: {e}")
        except asyncio.CancelledError:
            print("Navigation cancelled")
        finally:
            await self.drone.send_velocity(0, 0, 0, self.target_yaw_deg)
            print("Drone hovering safely")
            self.exploration_memory.print_map(title="Final Exploration Map")

    def stop(self):
        self.running = False


# =========================
#  ENTRY POINT
# =========================
async def main():
    bridge_state = SharedState()
    nav = DroneNavigation(bridge=bridge_state)
    task = asyncio.create_task(nav.run())
    try:
        while True:
            await asyncio.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopping...")
        nav.stop()
        await asyncio.gather(task, return_exceptions=True)


if __name__ == "__main__":
    asyncio.run(main())