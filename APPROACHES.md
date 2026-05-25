# RoboVerse Mission Approaches

This file summarizes the main approaches in this repository and how they evolved.

## 1. Early Direct Scripts

Key files:

- `autonomous_search_mission.py`
- `integrated_stationary_mission.py`
- `yaw_scan_mission.py`
- `small_fuel_detector.py`
- `barrel_detector.py`

Purpose:

- Verify MAVSDK connection, arming, takeoff, image access, and simple fuel-barrel detection.
- Prove that HSV and YOLO-style detection can identify red/yellow targets from the x500 vision camera.
- Build confidence in local-position navigation without relying on GNSS/global position.

Limitations:

- Mostly local behavior.
- Weak memory of explored regions.
- Easy to get stuck in repeated scans or blocked corridors.

## 2. Monolithic Competition Mission

Key file:

- `competition_mission.py`

Purpose:

- Combine takeoff, camera/depth access, visual detection, obstacle avoidance, heading selection, target memory, and landing into one runnable script.

Strengths:

- Simple to run under competition pressure.
- Uses the MAVSDK command path that worked reliably with PX4 SITL.
- Keeps detection and movement tightly coupled for quick iteration.

Limitations:

- Hard to debug because navigation, perception, memory, and control are all in one process.
- Local heading choices can revisit known corridors.
- Full-world coverage is not guaranteed.

## 3. RRT-Assisted Mission

Key folder:

- `rrt_assisted_mission/`

Purpose:

- Improve the monolithic mission without losing its working MAVSDK control path.
- Add local RRT assist when repeated movement failures indicate the drone is trapped near obstacles.
- Add next-best-view and uncharted-frontier bias for deeper map coverage.

Important scripts:

- `competition_mission.py`: main scoring mission.
- `survey_world_mission.py`: first-run full-world survey exporter.
- `score_from_survey_waypoints.py`: second-run scorer using survey waypoints.
- `check_ros2_usage.py`: confirms ROS2/package usage when needed.

What worked:

- Stronger coverage than the original monolithic script.
- Better candidate memory and revisit behavior.
- Local RRT assist can escape blocked headings without rewriting the whole mission.

Known issues:

- Still reactive and heuristic-heavy.
- Tight passages can trigger attitude spikes.
- Survey outputs are useful, but the first survey can still miss areas if the vehicle gets unstable.

## 4. MX Hybrid Mission

Key folder:

- `mx_hybrid_mission/`

Purpose:

- Preserve useful ideas from the separate `mx code` reference while keeping the project in the working RoboVerse/MAVSDK shape.

Notable pieces:

- Faster open-space movement.
- Narrow-passage probing.
- Local planner and mapper components adapted into the competition stack.

Why it exists:

- The MX code showed better speed and confidence through tight areas, but needed integration with the existing qualifier detector, logging, and mission flow.

## 5. Full-World Survey and Waypoint Scoring

Key scripts:

- `rrt_assisted_mission/survey_world_mission.py`
- `rrt_assisted_mission/score_from_survey_waypoints.py`

Purpose:

- Use the fact that the world layout is fixed between attempts.
- Run a survey to export confirmed and likely barrel waypoints.
- Use the second run to visit high-value target waypoints immediately.

Best use:

- Run survey first if there is time before the best judged attempt.
- Use exported waypoints to reduce search time in the scoring run.

Limitations:

- Barrel positions can move between released qualifier maps, so survey waypoints are only useful for the same map/session.
- If the survey crashes or stops early, the waypoint list may be biased toward the explored half of the world.

## 6. ROS2 A* Baseline

Key folder:

- `ros2_astar_mission/`

Purpose:

- Rebuild the mission as proper ROS2 nodes.
- Use occupancy mapping and A* path planning instead of purely local heading decisions.

Nodes:

- `depth_mapper_node`: builds a live occupancy grid from depth.
- `frontier_goal_node`: generates full-map coverage goals.
- `mission_manager_node`: prioritizes fuel detections and falls back to exploration.
- `astar_planner_node`: computes A* routes through the grid.
- `fuel_detector_node`: runs YOLO with HSV fallback.
- `mavsdk_waypoint_follower_node`: follows planned waypoints through MAVSDK.
- `px4_offboard_node`: pure ROS2 PX4 path for later use when `px4_msgs` is available.
- `dataset_capture_node`: captures training frames for YOLO.

Why this matters:

- Each subsystem can be inspected separately with ROS2 topics.
- A* gives a clearer path-planning baseline.
- The system can grow toward a more maintainable architecture than the monolithic mission.

Current status:

- Builds under ROS2 Humble.
- Avoids `cv_bridge` because the local environment has a NumPy 2.x versus ROS Humble binary compatibility issue.
- Uses MAVSDK control by default because it is the working PX4 command path on this machine.
- Includes direct-goal fallback if A* waypoints go stale.

## 7. Detection and Dataset Strategy

Current target assumption:

- 4 red barrels.
- 5 yellow barrels.

Detection design:

- Yellow barrels are ground-level targets.
- Red barrels are elevated targets, including boxes and open-box placements.
- YOLO should be trained with varied altitude, distance, lighting, occlusion, and viewing angles.

Recommended dataset captures:

- Low-altitude oblique views of yellow barrels.
- Mid/high views of red barrels on boxes.
- Red barrels partially hidden in open boxes.
- Bright and shadowed map regions.
- Motion-blurred and partially occluded frames.

The ROS2 stack includes `dataset_capture_node` and `scripts/train_yolo.py` for this workflow.

## Practical Recommendation

For immediate competition-style testing, start with:

1. `rrt_assisted_mission/competition_mission.py`
2. `rrt_assisted_mission/survey_world_mission.py`
3. `rrt_assisted_mission/score_from_survey_waypoints.py`

For long-term maintainability and clearer debugging, continue developing:

1. `ros2_astar_mission/`
2. YOLO dataset capture/training
3. A* occupancy tuning
4. Robust PX4 offboard control through ROS2 once `px4_msgs` is available
