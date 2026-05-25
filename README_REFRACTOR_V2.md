# RoboVerse Qualifier Refactor v2

Changes from v1:
- Reduced movement aggressiveness.
- Uses local expanding waypoints instead of starting at far grid corners.
- Reduces search limit to 6 m and step to 3 m.
- Sends a valid PositionNedYaw setpoint before Offboard start.
- Lands once University eligibility is met: at least one red and one yellow.
- Caps false-positive bursts by returning after one new confirmation per scan.
- Increases detector confidence threshold.
- Keeps HSV detector conservative because YOLO training is still the recommended real fix.

Run:
```bash
cd ~/roboverse_qualifier
python3 -m py_compile competition_mission.py small_fuel_detector.py bearing_detection_logger.py obstacle_monitor.py depth_debugger.py
python3 competition_mission.py
```
