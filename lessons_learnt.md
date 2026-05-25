# RoboVerse Qualifier: Lessons Learnt & Technical Log

## 1. Perception & Sensor Handling
### The "Invisible Wall" (Blindness) Bug
*   **Discovery:** When the drone is closer than 15cm to a wall, the OakD-Lite depth sensor returns `NaN` or `Inf`.
*   **The Error:** Previous code filtered out `NaN`, assuming the path was clear, causing the drone to accelerate into walls.
*   **The Fix:** Treat `NaN/Inf` as a **0.0m obstacle**. If the sensor is blind, assume a wall is touching the lens.

### Coordinate Scaling
*   **Lesson:** RGB (1920x1080) and Depth (640x480) have different aspect ratios and resolutions. 
*   **The Fix:** Always use `scale_rgb_to_depth()` before projecting a barrel detection into 3D space to avoid "ghost" obstacle offsets.

## 2. PX4 & EKF2 Configuration
### EKF2 Missing Data Error
*   **Discovery:** Modern PX4 (v1.14+) uses individual `_CTRL` bits rather than `AID_MASK`.
*   **The Fix:** Set `EKF2_EV_CTRL=7` (Horizontal + Vertical + Yaw), `EKF2_GPS_CTRL=0`, and `EKF2_BARO_CTRL=0` for pure vision-based stability.
*   **Optimization:** Set `EKF2_EV_DELAY` to 40ms to match the OakD-Lite processing latency.

## 3. Autonomous Navigation Logic
### The "Corner Trap"
*   **Lesson:** Reversing for 2 seconds is often not enough to clear a narrow dead-end.
*   **The Fix: Vertical Escape.** When stuck, climb to 5.5m (above the maze), reverse for 4 seconds, and re-plan. 

### Telemetry Blocking
*   **Lesson:** `async for` loops on MAVSDK telemetry streams are blocking. 
*   **The Fix:** Use `asyncio.gather()` to run Position and Yaw streams in parallel, or the script will hang on the first loop.

## 4. Competition Strategy
*   **Frontier Exploration:** On an unknown map, relative frontier generation (`start_n + dn`) is safer than absolute coordinates.
*   **GCS Reliability:** Always set `COM_ARM_WO_GPS=1` to allow the mission to start even if the Ground Control Station isn't connected.
