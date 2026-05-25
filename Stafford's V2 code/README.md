# RoboVerse Qualifier Team Handoff

This folder is a clean copy of the current working mission stack for sharing with teammates.

## Main Entry Point

Run from this folder after starting PX4/Gazebo with `x500_vision` in the `roboverse` world:

```bash
python3 competition_mission.py
```

## Active Mission Stack

- `competition_mission.py` - current GNSS-free qualifier mission.
- `obstacle_monitor.py` - depth-based front/side/lower clearance checks.
- `exploration_memory.py` - visited/blocked-cell novelty scoring.
- `target_memory.py` - candidate and confirmed target memory.
- `small_fuel_detector.py` - HSV/shape detector for yellow and red scoring barrels.
- `gzphotodetectorsaver.py` - background raw/YOLO photo capture worker.
- `mission_logger.py` - CSV/JSONL action logger.
- `ros2_sensor_bridge.py` - optional ROS2 sensor bridge fallback.
- `Codes/yolov10n.pt` - YOLO model used by capture bursts.

## Useful Supporting Scripts

- Sensor checks: `diagnostic_sensors.py`, `camera_test.py`, `depth_camera_test.py`, `obstacle_monitor_test.py`.
- PX4 checks: `takeoff_test.py`, `offboard_test.py`, `optimize_px4.py`.
- Mapping/analysis: `mapping_mission.py`, `local_mapper.py`, `trajectory_logger.py`, `plot_trajectory.py`.
- Planning references: `AStarPlanner.py`, `AvoidancePlanner.py`, `GlobalMapper.py`, `top_down.py`.
- Detector references: `barrel_detector.py`, `live_barrel_detector.py`.

## Teammate Split Ideas

- Navigation/safety: corridor PID, recovery behavior, obstacle thresholds.
- Exploration: macro frontier policy, entropy/novelty scoring, recentering.
- Perception: yellow/red detector tuning, YOLO burst usage, duplicate suppression.
- Infrastructure: logging, replay analysis, ROS2/Gazebo bridge reliability.

## Included Context

- `tasks/todo.md` and `tasks/lessons.md` summarize tuning history and lessons.
- `sample_logs/` includes one recent high-scoring run and one older good run.
- `sample_assets/` includes detector reference images/masks.
- `docs/` includes local setup/refactor notes and ROS2 bridge notes.

## Not Included

Large runtime output folders are intentionally omitted:

- `bc_logs/`
- `competition_photos/`
- `competition_evidence/`
- `logs/`
- `documents/` PDFs

Copy those separately only if a teammate needs raw evidence or official PDFs.
