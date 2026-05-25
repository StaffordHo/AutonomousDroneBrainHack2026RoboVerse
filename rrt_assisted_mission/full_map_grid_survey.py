#!/usr/bin/env python3
"""
Deterministic 40m x 40m RoboVerse survey.

This is intentionally less adventurous than the frontier/NBV survey. It flies a
high-altitude boustrophedon grid, pauses at each view point, and yaws through a
small panorama so the camera sees all sides of each map cell. The goal is full
world coverage with fewer narrow-zone attitude failures.
"""

import asyncio
import contextlib
import math
import os

# Conservative survey profile for the 10-minute qualifier envelope.
os.environ.setdefault("SURVEY_TIME_LIMIT_S", os.getenv("FULL_MAP_TIME_LIMIT_S", "585"))
os.environ.setdefault("MISSION_TIME_LIMIT_S", os.getenv("FULL_MAP_TIME_LIMIT_S", "585"))
os.environ.setdefault("LANDING_BUFFER_S", "25")
os.environ.setdefault("NARROW_CORRIDOR_ENABLED", "0")
os.environ.setdefault("MIN_FRONT_MOVE_CLEARANCE_M", "1.35")
os.environ.setdefault("MIN_SIDE_MOVE_CLEARANCE_M", "0.80")
os.environ.setdefault("MIN_LOWER_MOVE_CLEARANCE_M", "0.75")
os.environ.setdefault("MID_STEP_ABORT_CLEARANCE_M", "1.15")
os.environ.setdefault("YAW_MIN_FRONT_CLEARANCE_M", "1.15")
os.environ.setdefault("YAW_MIN_SIDE_CLEARANCE_M", "0.75")
os.environ.setdefault("MOVE_STEP_M", "0.24")
os.environ.setdefault("FAST_OPEN_STEP_M", "0.34")
os.environ.setdefault("OPEN_CRUISE_STEP_M", "0.50")
os.environ.setdefault("RRT_ASSIST_MAX_RANGE_M", "9.0")
os.environ.setdefault("RRT_ASSIST_MAX_ITERATIONS", "240")
os.environ.setdefault("RRT_ASSIST_MIN_FAILED_STEPS", "1")
os.environ.setdefault("MAX_FAILED_STEPS_PER_GOAL", "2")
os.environ.setdefault("SOFT_RANGE_LIMIT_M", "21.5")
os.environ.setdefault("RESUME_RANGE_M", "18.5")
os.environ.setdefault("HARD_RANGE_LIMIT_M", "24.0")

from survey_world_mission import export_survey_outputs, boot_vehicle
import competition_mission as mission


FULL_MAP_HALF_EXTENT_M = float(os.getenv("FULL_MAP_HALF_EXTENT_M", "16.0"))
FULL_MAP_GRID_SPACING_M = float(os.getenv("FULL_MAP_GRID_SPACING_M", "8.0"))
FULL_MAP_SURVEY_ALT_D = float(os.getenv("FULL_MAP_SURVEY_ALT_D", "-2.8"))
FULL_MAP_SCAN_FRAMES = int(os.getenv("FULL_MAP_SCAN_FRAMES", "2"))
FULL_MAP_SCAN_SETTLE_S = float(os.getenv("FULL_MAP_SCAN_SETTLE_S", "0.25"))
FULL_MAP_SCAN_OFFSETS_DEG = [
    float(value)
    for value in os.getenv("FULL_MAP_SCAN_OFFSETS_DEG", "0,90,-90,180").split(",")
    if value.strip()
]
FULL_MAP_GOAL_STEPS = int(os.getenv("FULL_MAP_GOAL_STEPS", "14"))


def normalize_zero(value):
    return 0.0 if abs(value) < 1e-6 else value


def symmetric_offsets(half_extent_m, spacing_m):
    offsets = [0.0]
    step = spacing_m

    while step <= half_extent_m + 1e-6:
        offsets.extend([step, -step])
        step += spacing_m

    return [normalize_zero(value) for value in offsets]


def ordered_full_map_grid():
    row_offsets = symmetric_offsets(FULL_MAP_HALF_EXTENT_M, FULL_MAP_GRID_SPACING_M)
    row_offsets.sort(key=lambda value: (abs(value), value))

    col_values = sorted(symmetric_offsets(FULL_MAP_HALF_EXTENT_M, FULL_MAP_GRID_SPACING_M))
    waypoints = []
    last_col = 0.0

    for row_index, row in enumerate(row_offsets):
        if row_index == 0:
            cols = symmetric_offsets(FULL_MAP_HALF_EXTENT_M, FULL_MAP_GRID_SPACING_M)
        else:
            forward = col_values
            reverse = list(reversed(col_values))
            cols = forward if abs(forward[0] - last_col) <= abs(reverse[0] - last_col) else reverse

        for col in cols:
            waypoints.append(
                {
                    "row": row,
                    "col": col,
                    "north_m": mission.start_n + row,
                    "east_m": mission.start_e + col,
                }
            )

        last_col = cols[-1]

    return waypoints


async def scan_panorama(drone, label, target_down):
    if mission.latest_position_ned is None:
        return

    base_yaw = mission.latest_attitude["yaw"]

    for scan_index, offset in enumerate(FULL_MAP_SCAN_OFFSETS_DEG):
        if not mission.search_time_remaining():
            break

        if mission.critical_vehicle_state(target_down):
            raise RuntimeError("critical_state")

        yaw = mission.normalize_angle_deg(base_yaw + offset)
        await drone.offboard.set_position_ned(
            mission.PositionNedYaw(
                mission.latest_position_ned.north_m,
                mission.latest_position_ned.east_m,
                target_down,
                yaw,
            )
        )
        await asyncio.sleep(FULL_MAP_SCAN_SETTLE_S)
        await mission.scan_current_view(
            label=f"{label}_scan_{scan_index:02d}",
            frames=FULL_MAP_SCAN_FRAMES,
        )
        await mission.handle_candidate_event(drone, target_down)


async def run_full_map_grid(drone):
    mission.active_allowed_colours = ("yellow", "red")
    mission.active_target_alt_d = FULL_MAP_SURVEY_ALT_D
    mission.active_investigation_enabled = False

    waypoints = ordered_full_map_grid()

    print("\n===== FULL_MAP_GRID_SURVEY STARTED =====")
    print(
        f"Map target: 40m x 40m, half_extent={FULL_MAP_HALF_EXTENT_M:.1f}m, "
        f"spacing={FULL_MAP_GRID_SPACING_M:.1f}m, waypoints={len(waypoints)}"
    )
    print(
        f"Altitude down={FULL_MAP_SURVEY_ALT_D:.1f}, scan_offsets={FULL_MAP_SCAN_OFFSETS_DEG}, "
        f"time_limit={mission.MISSION_TIME_LIMIT_S:.0f}s"
    )

    perception = asyncio.create_task(mission.perception_task())

    try:
        await mission.hold_position(drone, FULL_MAP_SURVEY_ALT_D, duration_s=0.8)
        await scan_panorama(drone, "FULL_MAP_HOME", FULL_MAP_SURVEY_ALT_D)

        for index, waypoint in enumerate(waypoints):
            if not mission.search_time_remaining():
                break

            label = f"FULL_MAP_GRID_{index:02d}_r{waypoint['row']:+.0f}_c{waypoint['col']:+.0f}"
            distance = mission.distance_to_point_m(waypoint["north_m"], waypoint["east_m"])

            print(
                f"\n===== {label} target_N={waypoint['north_m']:.1f} "
                f"target_E={waypoint['east_m']:.1f} dist={distance:.1f} ====="
            )

            await mission.navigate_to_coverage_goal(
                drone,
                waypoint["north_m"],
                waypoint["east_m"],
                FULL_MAP_SURVEY_ALT_D,
                label=label,
                max_steps=FULL_MAP_GOAL_STEPS,
            )

            if not mission.search_time_remaining():
                break

            await scan_panorama(drone, label, FULL_MAP_SURVEY_ALT_D)

    finally:
        mission.mission_should_stop = True
        perception.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await perception


async def main():
    resources = None
    safety_reason = "full map grid survey complete"

    try:
        resources = await boot_vehicle()
        await run_full_map_grid(resources["drone"])

        if not mission.search_time_remaining():
            safety_reason = "time budget reached"
    except RuntimeError as error:
        safety_reason = str(error)
        print(f"Full-map survey safety stop: {safety_reason}")
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
