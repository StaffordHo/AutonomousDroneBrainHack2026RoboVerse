#!/usr/bin/env python3
"""
Pure exploration survey for the RoboVerse qualifier world.

This script flies the same local-NED, depth-gated, RRT/NBV-assisted stack as
competition_mission.py, but it optimizes for mapping and waypoint discovery
instead of stopping once scoring eligibility is reached.

Outputs:
  survey_outputs/waypoints_<timestamp>.json
  survey_outputs/waypoints_<timestamp>.csv
  survey_outputs/coverage_route_<timestamp>.csv
  survey_outputs/visited_cells_<timestamp>.csv
"""

import asyncio
import contextlib
import csv
import json
import math
import os
from datetime import datetime
from pathlib import Path

# Survey defaults. Operator-provided env vars still win.
os.environ.setdefault("MISSION_TIME_LIMIT_S", os.getenv("SURVEY_TIME_LIMIT_S", "720"))
os.environ.setdefault("LANDING_BUFFER_S", os.getenv("SURVEY_LANDING_BUFFER_S", "24"))
os.environ.setdefault("NBV_FRONTIER_ENABLED", "1")
os.environ.setdefault("NBV_FRONTIER_GOALS_PER_PASS", "10")
os.environ.setdefault("NBV_FRONTIER_RINGS_M", "6,10,14,18,21")
os.environ.setdefault("NBV_FRONTIER_MAX_STEPS", "18")
os.environ.setdefault("NBV_FRONTIER_SCAN_FRAMES", "2")
os.environ.setdefault("UNCHARTED_FRONTIER_ENABLED", "1")
os.environ.setdefault("UNCHARTED_NARROW_BONUS", "24")
os.environ.setdefault("RRT_ASSIST_MAX_RANGE_M", "9.0")
os.environ.setdefault("RRT_ASSIST_MAX_ITERATIONS", "220")
os.environ.setdefault("OPEN_CRUISE_STEP_M", "0.58")
os.environ.setdefault("NARROW_CORRIDOR_ENABLED", "0")
os.environ.setdefault("MIN_FRONT_MOVE_CLEARANCE_M", "1.20")
os.environ.setdefault("MIN_SIDE_MOVE_CLEARANCE_M", "0.65")
os.environ.setdefault("MIN_LOWER_MOVE_CLEARANCE_M", "0.65")
os.environ.setdefault("MID_STEP_ABORT_CLEARANCE_M", "1.05")
os.environ.setdefault("YAW_MIN_FRONT_CLEARANCE_M", "1.05")
os.environ.setdefault("YAW_MIN_SIDE_CLEARANCE_M", "0.65")
os.environ.setdefault("SOFT_RANGE_LIMIT_M", "21.0")
os.environ.setdefault("RESUME_RANGE_M", "16.2")
os.environ.setdefault("HARD_RANGE_LIMIT_M", "24.0")

import competition_mission as mission
from mavsdk.offboard import OffboardError


SURVEY_OUTPUT_DIR = Path(os.getenv("SURVEY_OUTPUT_DIR", "survey_outputs"))
SURVEY_NBV_CYCLES = int(os.getenv("SURVEY_NBV_CYCLES", "4"))
SURVEY_FRONTIER_STRIDES = int(os.getenv("SURVEY_FRONTIER_STRIDES", "10"))
SURVEY_FRONTIER_STEPS = int(os.getenv("SURVEY_FRONTIER_STEPS", "8"))
SURVEY_BOOTSTRAP_FRONTIER = os.getenv("SURVEY_BOOTSTRAP_FRONTIER", "1") == "1"
SURVEY_REVISIT_CANDIDATES = os.getenv("SURVEY_REVISIT_CANDIDATES", "0") == "1"
SURVEY_INVESTIGATE_CANDIDATES = os.getenv("SURVEY_INVESTIGATE_CANDIDATES", "0") == "1"
SURVEY_WAYPOINT_STANDOFF_M = float(os.getenv("SURVEY_WAYPOINT_STANDOFF_M", "2.4"))
SURVEY_ROUTE_SAMPLE_M = float(os.getenv("SURVEY_ROUTE_SAMPLE_M", "2.5"))


SURVEY_MACRO_HEADINGS_DEG = [
    0.0,
    22.5,
    45.0,
    67.5,
    90.0,
    112.5,
    135.0,
    157.5,
    180.0,
    -157.5,
    -135.0,
    -112.5,
    -90.0,
    -67.5,
    -45.0,
    -22.5,
]


def mean(values):
    return sum(values) / max(len(values), 1)


def entry_target_xy(entry):
    if entry.get("mean_target_n") is not None and entry.get("mean_target_e") is not None:
        return float(entry["mean_target_n"]), float(entry["mean_target_e"])

    ns = entry.get("target_n_list", [])
    es = entry.get("target_e_list", [])

    if ns and es:
        return mean(ns), mean(es)

    return None


def entry_yaw_span(entry):
    yaws = entry.get("observer_yaws", [])
    if len(yaws) < 2:
        return 0.0

    sin_sum = sum(math.sin(math.radians(yaw)) for yaw in yaws)
    cos_sum = sum(math.cos(math.radians(yaw)) for yaw in yaws)
    centre = mission.normalize_angle_deg(math.degrees(math.atan2(sin_sum, cos_sum)))
    return max(abs(mission.normalize_angle_deg(yaw - centre)) for yaw in yaws) * 2.0


def waypoint_for_entry(entry, status, index):
    target_xy = entry_target_xy(entry)
    if target_xy is None:
        return None

    target_n, target_e = target_xy
    bearing_from_start = math.atan2(target_e - mission.start_e, target_n - mission.start_n)
    visit_n = target_n - SURVEY_WAYPOINT_STANDOFF_M * math.cos(bearing_from_start)
    visit_e = target_e - SURVEY_WAYPOINT_STANDOFF_M * math.sin(bearing_from_start)
    yaw_to_target = mission.normalize_angle_deg(
        math.degrees(math.atan2(target_e - visit_e, target_n - visit_n))
    )
    colour = entry.get("colour", "unknown")
    confidence = float(entry.get("confidence", 0.0))
    count = int(entry.get("count", 0))
    yaw_span = entry_yaw_span(entry)
    depths = entry.get("depths", [])
    mean_depth = mean(depths) if depths else None
    score = count * 2.0 + confidence * 8.0 + min(yaw_span / 8.0, 5.0)

    if status == "confirmed":
        score += 25.0

    return {
        "id": f"{status}_{index:02d}_{colour}",
        "status": status,
        "colour": colour,
        "label": entry.get("label"),
        "source": entry.get("source"),
        "priority": round(score, 3),
        "target_n": round(target_n, 3),
        "target_e": round(target_e, 3),
        "visit_n": round(visit_n, 3),
        "visit_e": round(visit_e, 3),
        "yaw_deg": round(yaw_to_target, 2),
        "alt_d": mission.HIGH_SCAN_ALT_D if colour == "red" else mission.LOW_SCAN_ALT_D,
        "count": count,
        "confidence": round(confidence, 3),
        "yaw_span_deg": round(yaw_span, 2),
        "mean_depth_m": round(mean_depth, 3) if mean_depth is not None else None,
    }


def collect_survey_waypoints():
    summary = mission.target_memory.summary()
    waypoints = []

    for index, entry in enumerate(summary["confirmed"]):
        waypoint = waypoint_for_entry(entry, "confirmed", index)
        if waypoint is not None:
            waypoints.append(waypoint)

    for index, entry in enumerate(summary["candidates"]):
        waypoint = waypoint_for_entry(entry, "candidate", index)
        if waypoint is not None:
            waypoints.append(waypoint)

    waypoints.sort(key=lambda item: item["priority"], reverse=True)
    return waypoints


def collect_coverage_route():
    route = []
    last_n = None
    last_e = None

    sampled_points = []
    for point_n, point_e in mission.exploration_memory.path_history:
        if last_n is None:
            sampled_points.append((point_n, point_e))
            last_n = point_n
            last_e = point_e
            continue

        if math.hypot(point_n - last_n, point_e - last_e) >= SURVEY_ROUTE_SAMPLE_M:
            sampled_points.append((point_n, point_e))
            last_n = point_n
            last_e = point_e

    for index, (point_n, point_e) in enumerate(sampled_points):
        if index + 1 < len(sampled_points):
            next_n, next_e = sampled_points[index + 1]
            yaw = mission.normalize_angle_deg(
                math.degrees(math.atan2(next_e - point_e, next_n - point_n))
            )
        else:
            yaw = mission.start_yaw

        route.append(
            {
                "id": f"route_{index:03d}",
                "north_m": round(point_n, 3),
                "east_m": round(point_e, 3),
                "yaw_deg": round(yaw, 2),
                "alt_d": mission.LOW_SCAN_ALT_D if index % 2 == 0 else mission.HIGH_SCAN_ALT_D,
            }
        )

    return route


def export_survey_outputs(reason):
    SURVEY_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    memory_debug = mission.exploration_memory.debug_summary()
    summary = mission.target_memory.summary()
    waypoints = collect_survey_waypoints()
    coverage_route = collect_coverage_route()

    payload = {
        "created_at": timestamp,
        "reason": reason,
        "start": {
            "north_m": mission.start_n,
            "east_m": mission.start_e,
            "yaw_deg": mission.start_yaw,
        },
        "counts": {
            "red": summary["red"],
            "yellow": summary["yellow"],
            "total_confirmed": summary["total"],
            "candidate_count": len(summary["candidates"]),
            "visited_cells": memory_debug["visited_cells"],
            "blocked_cells": memory_debug["blocked_cells"],
            "path_points": memory_debug["path_points"],
        },
        "waypoints": waypoints,
        "coverage_route": coverage_route,
    }

    json_path = SURVEY_OUTPUT_DIR / f"waypoints_{timestamp}.json"
    csv_path = SURVEY_OUTPUT_DIR / f"waypoints_{timestamp}.csv"
    route_path = SURVEY_OUTPUT_DIR / f"coverage_route_{timestamp}.csv"
    visited_path = SURVEY_OUTPUT_DIR / f"visited_cells_{timestamp}.csv"

    json_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    fieldnames = [
        "id",
        "status",
        "colour",
        "label",
        "source",
        "priority",
        "target_n",
        "target_e",
        "visit_n",
        "visit_e",
        "yaw_deg",
        "alt_d",
        "count",
        "confidence",
        "yaw_span_deg",
        "mean_depth_m",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(waypoints)

    with route_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["id", "north_m", "east_m", "yaw_deg", "alt_d"],
        )
        writer.writeheader()
        writer.writerows(coverage_route)

    with visited_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["cell_n", "cell_e", "visit_count"])
        for (cell_n, cell_e), visit_count in sorted(mission.exploration_memory.visited_cells.items()):
            writer.writerow([cell_n, cell_e, visit_count])

    print("\n==============================")
    print("SURVEY SUMMARY")
    print(f"Red confirmed: {summary['red']}")
    print(f"Yellow confirmed: {summary['yellow']}")
    print(f"Unconfirmed candidates: {len(summary['candidates'])}")
    print(f"Exported waypoints: {len(waypoints)}")
    print(f"Coverage route points: {len(coverage_route)}")
    print(f"Visited cells: {memory_debug['visited_cells']}")
    print(f"Blocked cells: {memory_debug['blocked_cells']}")
    print(f"Waypoint JSON: {json_path}")
    print(f"Waypoint CSV: {csv_path}")
    print(f"Coverage route CSV: {route_path}")
    print(f"Visited cells CSV: {visited_path}")
    print("==============================")

    return json_path


async def boot_vehicle():
    drone = mission.System()
    ros2_sensor_bridge = None
    gz_node = None
    camera_task = None
    telemetry_task = None

    try:
        await drone.connect(system_address="udpin://0.0.0.0:14540")
        await mission.wait_for_connection(drone)

        image_topic = mission.find_image_topic()
        print(f"Using image topic: {image_topic}")
        print(f"Using depth topic: {mission.DEPTH_TOPIC}")

        if mission.USE_ROS2_SENSOR_BRIDGE:
            if mission.ROS2_AVAILABLE:
                ros2_sensor_bridge = mission.Ros2SensorBridge(
                    mission.ROS2_IMAGE_TOPIC,
                    mission.ROS2_DEPTH_TOPIC,
                    mission.update_latest_frame_bgr,
                    mission.update_latest_depth,
                )
                if ros2_sensor_bridge.start():
                    print(
                        "Using ROS2 sensor bridge: "
                        f"image={mission.ROS2_IMAGE_TOPIC}, depth={mission.ROS2_DEPTH_TOPIC}"
                    )
                else:
                    ros2_sensor_bridge = None
                    print("ROS2 sensor bridge unavailable. Falling back to Gazebo transport.")
            else:
                print("rclpy/sensor_msgs unavailable. Falling back to Gazebo transport.")

        if ros2_sensor_bridge is None:
            gz_node = mission.Node()
            gz_node.subscribe(mission.Image, image_topic, mission.image_callback)
            gz_node.subscribe(mission.Image, mission.DEPTH_TOPIC, mission.depth_callback)
            print("Using Gazebo transport for RGB/depth sensor topics.")

        mission.camera_photo_saver = mission.GZPhotoDetectorSaver(
            topic=image_topic,
            save_dir=mission.PHOTO_BURST_DIR,
            model_path=mission.YOLO_MODEL_PATH,
            burst_size=mission.STOP_CAPTURE_BURST_FRAMES,
            threshold=mission.DETECTION_CONFIDENCE_THRESHOLD,
            enable_yolo=mission.YOLO_BURST_ENABLED,
            imgsz=mission.YOLO_IMGSZ,
            device=mission.YOLO_DEVICE,
        )
        mission.initialize_mission_yolo_model()
        camera_task = asyncio.create_task(mission.camera_photo_saver.run())
        telemetry_task = asyncio.create_task(mission.telemetry_task(drone))

        await mission.wait_for_local_position(drone)
        await mission.wait_for_telemetry()
        await mission.wait_for_camera_depth(timeout_s=8.0)

        print("Arming & takeoff...")
        armed = await mission.arm_with_retry(drone)

        if not armed:
            raise RuntimeError("arming failed")

        await drone.action.set_takeoff_altitude(abs(mission.DEFAULT_ALT_D))
        await drone.action.takeoff()
        await asyncio.sleep(8)
        await mission.wait_for_telemetry()

        mission.start_n = mission.latest_position_ned.north_m
        mission.start_e = mission.latest_position_ned.east_m
        mission.start_yaw = mission.latest_attitude["yaw"]
        mission.exploration_memory.initialize(mission.start_n, mission.start_e)
        mission.exploration_memory.mark_visited(mission.start_n, mission.start_e)
        mission.remember_safe_position()
        mission.mission_start_time = mission.time.time()

        await mission.prime_and_start_offboard(drone, mission.DEFAULT_ALT_D)

        return {
            "drone": drone,
            "ros2_sensor_bridge": ros2_sensor_bridge,
            "gz_node": gz_node,
            "camera_task": camera_task,
            "telemetry_task": telemetry_task,
        }
    except Exception:
        if ros2_sensor_bridge is not None:
            ros2_sensor_bridge.stop()

        if mission.camera_photo_saver is not None:
            mission.camera_photo_saver.running = False

        for task in (camera_task, telemetry_task):
            if task is not None:
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await task

        with contextlib.suppress(Exception):
            await drone.action.land()

        raise


async def survey_world(drone):
    mission.active_allowed_colours = ("yellow", "red")
    mission.active_target_alt_d = mission.LOW_SCAN_ALT_D
    mission.active_investigation_enabled = SURVEY_INVESTIGATE_CANDIDATES

    print("\n===== ROBOVERSE FULL-WORLD SURVEY STARTED =====")
    print(
        "Survey profile: "
        f"time={mission.MISSION_TIME_LIMIT_S:.0f}s "
        f"nbv_cycles={SURVEY_NBV_CYCLES} "
        f"frontier_strides={SURVEY_FRONTIER_STRIDES} "
        f"frontier_steps={SURVEY_FRONTIER_STEPS}"
    )
    print(
        "Survey outputs will include confirmed and candidate barrel waypoints "
        "for a second scoring run."
    )
    print(
        "RRT/NBV: "
        f"rrt_range={mission.RRT_ASSIST_MAX_RANGE_M:.1f}m "
        f"rrt_iters={mission.RRT_ASSIST_MAX_ITERATIONS} "
        f"nbv_rings={mission.NBV_FRONTIER_RINGS_M} "
        f"nbv_goals={mission.NBV_FRONTIER_GOALS_PER_PASS}"
    )

    perception = asyncio.create_task(mission.perception_task())

    try:
        await mission.hold_position(drone, mission.LOW_SCAN_ALT_D, duration_s=0.5)
        await mission.scan_current_view(label="SURVEY_START_VIEW", frames=2)

        if SURVEY_BOOTSTRAP_FRONTIER and mission.search_time_remaining():
            await mission.frontier_coverage_pass(
                drone,
                target_down=mission.LOW_SCAN_ALT_D,
                pass_name="SURVEY_BOOTSTRAP_LOW",
                stride_count=min(SURVEY_FRONTIER_STRIDES, len(SURVEY_MACRO_HEADINGS_DEG)),
                steps_per_stride=SURVEY_FRONTIER_STEPS,
                allowed_colours=("yellow", "red"),
                investigate=SURVEY_INVESTIGATE_CANDIDATES,
                macro_headings=SURVEY_MACRO_HEADINGS_DEG,
                stop_on_eligibility=False,
            )

        cycle = 0
        while mission.search_time_remaining() and cycle < SURVEY_NBV_CYCLES:
            rotation = cycle * 11.25
            target_down = mission.LOW_SCAN_ALT_D if cycle % 2 == 0 else mission.HIGH_SCAN_ALT_D
            macro_headings = [
                mission.normalize_angle_deg(heading + rotation)
                for heading in SURVEY_MACRO_HEADINGS_DEG
            ]

            completed = await mission.nbv_frontier_explore_pass(
                drone,
                target_down=target_down,
                pass_name=f"SURVEY_NBV_{cycle:02d}",
                max_goals=mission.NBV_FRONTIER_GOALS_PER_PASS,
            )

            if mission.search_time_remaining():
                await mission.frontier_coverage_pass(
                    drone,
                    target_down=target_down,
                    pass_name=f"SURVEY_FRONTIER_{cycle:02d}",
                    stride_count=SURVEY_FRONTIER_STRIDES,
                    steps_per_stride=SURVEY_FRONTIER_STEPS,
                    allowed_colours=("yellow", "red"),
                    investigate=SURVEY_INVESTIGATE_CANDIDATES,
                    macro_headings=macro_headings,
                    stop_on_eligibility=False,
                )

            if completed == 0 and not mission.search_time_remaining():
                break

            cycle += 1

        if SURVEY_REVISIT_CANDIDATES and mission.search_time_remaining():
            print("\n===== SURVEY_CANDIDATE_REVISIT =====")
            await mission.revisit_candidate_waypoints(drone)

    finally:
        mission.mission_should_stop = True
        perception.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await perception


async def main():
    resources = None
    safety_reason = "survey complete"

    try:
        resources = await boot_vehicle()
        await survey_world(resources["drone"])

        if not mission.search_time_remaining():
            safety_reason = "time budget reached"

    except OffboardError as error:
        safety_reason = f"offboard_error:{error}"
        print(f"Offboard start failed: {error}")
    except RuntimeError as error:
        safety_reason = str(error)
        print(f"Survey safety stop: {safety_reason}")
    finally:
        json_path = export_survey_outputs(safety_reason)
        print(f"Latest survey waypoint file: {json_path}")

        if resources is not None:
            if resources["ros2_sensor_bridge"] is not None:
                resources["ros2_sensor_bridge"].stop()

            if mission.camera_photo_saver is not None:
                mission.camera_photo_saver.running = False

            for task_name in ("camera_task", "telemetry_task"):
                task = resources.get(task_name)
                if task is not None:
                    task.cancel()
                    with contextlib.suppress(asyncio.CancelledError):
                        await task

            mission.mission_logger.save()
            await mission.stop_and_land(resources["drone"], safety_reason)


if __name__ == "__main__":
    asyncio.run(main())
