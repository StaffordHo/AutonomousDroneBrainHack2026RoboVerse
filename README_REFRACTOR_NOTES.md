# RoboVerse Qualifier Refactor v1

This refactor prioritises a simple, robust qualifier strategy:

1. Use the currently validated `x500_depth`-style camera/depth topics.
2. Search using a serpentine grid relative to the drone's start position.
3. Scan before and after each waypoint because blocked positions can still be useful viewpoints.
4. Use depth for local obstacle avoidance; skip impossible waypoints instead of getting stuck.
5. Use a conservative HSV detector only as a stopgap.
6. Deduplicate detections using estimated target position when depth is available.
7. Save evidence only when a target first transitions into confirmed state.

Important limitation:
- The HSV detector remains the weakest part. For serious qualifier robustness, collect images and train a YOLO detector for:
  - `red_fuel_barrel`
  - `yellow_fuel_barrel`

Install:
```bash
cd ~/roboverse_qualifier
cp competition_mission.py competition_mission_old.py
cp small_fuel_detector.py small_fuel_detector_old.py
cp bearing_detection_logger.py bearing_detection_logger_old.py
cp obstacle_monitor.py obstacle_monitor_old.py
cp depth_debugger.py depth_debugger_old.py

# Copy these replacement files into ~/roboverse_qualifier
python3 -m py_compile competition_mission.py small_fuel_detector.py bearing_detection_logger.py obstacle_monitor.py depth_debugger.py
python3 competition_mission.py
```
