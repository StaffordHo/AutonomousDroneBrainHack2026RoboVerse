import numpy as np
from AvoidancePlanner import AvoidancePlanner
import math

K = np.array([[433.0, 0.0, 320.0],
              [0.0, 433.0, 240.0],
              [0.0, 0.0, 1.0]])

planner = AvoidancePlanner(K=K, width=640, height=480, max_speed=1.5, safe_distance=2.5, critical_distance=0.8)

# Test 1: Wall at 1.0 meters in front
depth_map = np.ones((480, 640), dtype=np.float32) * 10.0
depth_map[:, 200:440] = 1.0 # Wall in the center

pose = {"north": 0.0, "east": 0.0, "yaw": 0.0, "down": -3.5}
target_n, target_e = 10.0, 0.0

vx, vy, info = planner.compute_velocity(depth_map, pose, target_n, target_e)
print(f"Test 1 (Wall at 1.0m): vx={vx:.2f}, vy={vy:.2f}, speed={info['forward_speed']:.2f}")

# Test 2: Wall at 0.5 meters (critical)
depth_map[:, 200:440] = 0.5
vx, vy, info = planner.compute_velocity(depth_map, pose, target_n, target_e)
print(f"Test 2 (Wall at 0.5m): vx={vx:.2f}, vy={vy:.2f}, speed={info['forward_speed']:.2f}, emergency={info.get('blocked')}")

# Test 3: Wall is nan (too close?)
depth_map[:, 200:440] = np.nan
vx, vy, info = planner.compute_velocity(depth_map, pose, target_n, target_e)
print(f"Test 3 (Wall is NaN): vx={vx:.2f}, vy={vy:.2f}, speed={info['forward_speed']:.2f}")

# Test 4: Wall is inf
depth_map[:, 200:440] = np.inf
vx, vy, info = planner.compute_velocity(depth_map, pose, target_n, target_e)
print(f"Test 4 (Wall is Inf): vx={vx:.2f}, vy={vy:.2f}, speed={info['forward_speed']:.2f}")
