# Dependencies

Expected environment:

- PX4 SITL with Gazebo Harmonic/Gazebo Transport Python bindings available.
- MAVSDK Python.
- Python 3.10+.
- OpenCV and NumPy.
- Optional ROS2 Python packages if `USE_ROS2_SENSOR_BRIDGE=1`.
- Optional Ultralytics for YOLO capture bursts.

Common Python packages:

```bash
python3 -m pip install mavsdk opencv-python numpy grpcio ultralytics matplotlib
```

Gazebo Python bindings such as `gz.transport13` and `gz.msgs10` usually come from the PX4/Gazebo environment rather than pip. Verify them with:

```bash
python3 -c "from gz.transport13 import Node; from gz.msgs10.image_pb2 import Image; print('gz python bindings ok')"
```

Optional environment toggles:

```bash
NARROW_CORRIDOR_ENABLED=1
CONTINUE_FRONTIER_AFTER_ELIGIBILITY=1
EXTRA_SCORING_AFTER_ELIGIBILITY=0
USE_ROS2_SENSOR_BRIDGE=0
```
