#!/usr/bin/env python3
"""
Second-run scorer that visits survey-discovered barrel waypoints first.

Run a survey with survey_world_mission.py, then run this script. It loads the
latest survey_outputs/waypoints_*.json unless SURVEY_WAYPOINT_FILE is set.
"""

import asyncio
import contextlib
import json
import os
from pathlib import Path

os.environ.setdefault("MISSION_TIME_LIMIT_S", "540")
os.environ.setdefault("LANDING_BUFFER_S", "18")
os.environ.setdefault("SURVEY_TIME_LIMIT_S", os.environ["MISSION_TIME_LIMIT_S"])

from survey_world_mission import boot_vehicle, export_survey_outputs
import competition_mission as mission


WAYPOINT_DIR = Path(os.getenv("SURVEY_OUTPUT_DIR", "survey_outputs"))
WAYPOINT_FILE = os.getenv("SURVEY_WAYPOINT_FILE")
MAX_WAYPOINTS = int(os.getenv("SURVEY_SCORE_MAX_WAYPOINTS", "12"))
INCLUDE_CANDIDATES = os.getenv("SURVEY_SCORE_INCLUDE_CANDIDATES", "1") == "1"
FOLLOW_ROUTE = os.getenv("SURVEY_SCORE_FOLLOW_ROUTE", "0") == "1"
MAX_ROUTE_POINTS = int(os.getenv("SURVEY_SCORE_MAX_ROUTE_POINTS", "30"))
FOLLOW_WITH_FRONTIER = os.getenv("SURVEY_SCORE_FOLLOW_WITH_FRONTIER", "0") == "1"


def latest_waypoint_file():
    if WAYPOINT_FILE:
        return Path(WAYPOINT_FILE)

    files = sorted(WAYPOINT_DIR.glob("waypoints_*.json"))
    if not files:
        raise FileNotFoundError(
            f"No survey waypoint files found in {WAYPOINT_DIR}. "
            "Run survey_world_mission.py first or set SURVEY_WAYPOINT_FILE."
        )

    return files[-1]


def load_waypoints():
    path = latest_waypoint_file()
    payload = json.loads(path.read_text(encoding="utf-8"))
    waypoints = payload.get("waypoints", [])
    coverage_route = payload.get("coverage_route", [])

    if not INCLUDE_CANDIDATES:
        waypoints = [wp for wp in waypoints if wp.get("status") == "confirmed"]

    waypoints = sorted(
        waypoints,
        key=lambda wp: float(wp.get("priority", 0.0)),
        reverse=True,
    )

    print(
        f"Loaded {len(waypoints)} survey target waypoints and "
        f"{len(coverage_route)} route points from {path}"
    )
    return path, waypoints[:MAX_WAYPOINTS], coverage_route[:MAX_ROUTE_POINTS]


async def visit_survey_waypoint(drone, waypoint, index):
    colour = waypoint.get("colour", "yellow")
    target_down = float(
        waypoint.get(
            "alt_d",
            mission.HIGH_SCAN_ALT_D if colour == "red" else mission.LOW_SCAN_ALT_D,
        )
    )
    mission.active_allowed_colours = (colour,) if colour in ("red", "yellow") else ("yellow", "red")
    mission.active_target_alt_d = target_down
    mission.active_investigation_enabled = True
    mission.new_candidate_event.clear()

    label = f"SURVEY_SCORE_{index:02d}_{waypoint.get('status', 'wp')}_{colour}"
    print(
        f"\n===== {label} priority={waypoint.get('priority')} "
        f"visit=({waypoint['visit_n']:.1f}, {waypoint['visit_e']:.1f}) "
        f"target=({waypoint['target_n']:.1f}, {waypoint['target_e']:.1f}) ====="
    )

    await mission.hold_position(drone, target_down, duration_s=0.4)

    await mission.navigate_to_coverage_goal(
        drone,
        float(waypoint["visit_n"]),
        float(waypoint["visit_e"]),
        target_down,
        label=label,
        max_steps=max(mission.MAX_REVISIT_GOAL_STEPS, 10),
    )

    if not mission.search_time_remaining():
        return

    if waypoint.get("yaw_deg") is not None:
        yaw = float(waypoint["yaw_deg"])
    else:
        yaw = mission.heading_to_point_deg(
            float(waypoint["target_n"]),
            float(waypoint["target_e"]),
        )
    await drone.offboard.set_position_ned(
        mission.PositionNedYaw(
            mission.latest_position_ned.north_m,
            mission.latest_position_ned.east_m,
            target_down,
            yaw,
        )
    )
    await asyncio.sleep(mission.INVESTIGATION_SETTLE_S)
    await mission.scan_current_view(
        label=f"{label}_face",
        frames=mission.SCAN_FRAMES_PER_VIEW,
    )
    await mission.investigate_candidate(drone, target_down)


async def follow_route_point(drone, route_point, index):
    target_down = float(route_point.get("alt_d", mission.LOW_SCAN_ALT_D))
    mission.active_allowed_colours = ("yellow", "red")
    mission.active_target_alt_d = target_down
    mission.active_investigation_enabled = True

    label = f"SURVEY_ROUTE_{index:03d}"
    print(
        f"\n===== {label} n={route_point['north_m']:.1f} "
        f"e={route_point['east_m']:.1f} ====="
    )
    await mission.navigate_to_coverage_goal(
        drone,
        float(route_point["north_m"]),
        float(route_point["east_m"]),
        target_down,
        label=label,
        max_steps=8,
    )

    if mission.search_time_remaining():
        await mission.scan_current_view(label=f"{label}_view", frames=2)


async def score_from_waypoints(drone, waypoints, coverage_route):
    mission.active_allowed_colours = ("yellow", "red")
    mission.active_target_alt_d = mission.LOW_SCAN_ALT_D
    mission.active_investigation_enabled = True

    perception = asyncio.create_task(mission.perception_task())

    try:
        for index, waypoint in enumerate(waypoints):
            if not mission.search_time_remaining():
                break
            await visit_survey_waypoint(drone, waypoint, index)

        if mission.search_time_remaining():
            await mission.revisit_candidate_waypoints(drone)

        if FOLLOW_ROUTE and mission.search_time_remaining():
            print("\n===== SURVEY_SCORE_COVERAGE_ROUTE =====")
            for index, route_point in enumerate(coverage_route):
                if not mission.search_time_remaining():
                    break
                await follow_route_point(drone, route_point, index)

        if FOLLOW_WITH_FRONTIER and mission.search_time_remaining():
            print("\n===== SURVEY_SCORE_FRONTIER_FALLBACK =====")
            await mission.global_coverage_sweep(drone, defer_investigation=True)

            if mission.search_time_remaining():
                await mission.revisit_candidate_waypoints(drone)

    finally:
        mission.mission_should_stop = True
        perception.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await perception


async def main():
    resources = None
    safety_reason = "survey waypoint scoring complete"

    try:
        path, waypoints, coverage_route = load_waypoints()
        if not waypoints and not coverage_route:
            raise RuntimeError(f"No usable waypoints or route points in {path}")

        resources = await boot_vehicle()
        await score_from_waypoints(resources["drone"], waypoints, coverage_route)

        if not mission.search_time_remaining():
            safety_reason = "time budget reached"

        summary = mission.target_memory.summary()
        memory_debug = mission.exploration_memory.debug_summary()

        print("\n==============================")
        print("SURVEY WAYPOINT SCORE SUMMARY")
        print(f"Red: {summary['red']}")
        print(f"Yellow: {summary['yellow']}")
        print(f"Total confirmed: {summary['total']}")
        print(f"Score: {mission.get_score(summary)}")
        print(f"Eligibility met: {mission.eligibility_met(summary)}")
        print(f"Unconfirmed candidates left: {len(summary['candidates'])}")
        print(f"Visited cells: {memory_debug['visited_cells']}")
        print(f"Blocked cells: {memory_debug['blocked_cells']}")
        print("==============================")

    except RuntimeError as error:
        safety_reason = str(error)
        print(f"Waypoint scoring safety stop: {safety_reason}")
    finally:
        export_survey_outputs(safety_reason)

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
