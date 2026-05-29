# RoboVerse Document Technique Notes

These notes summarize the useful control/navigation guidance found in
`../documents/`.

## Most Relevant Findings

- The qualifier spec confirms the mission is GNSS-free object detection plus
  obstacle avoidance in a 40m x 40m x 8m space port. Yellow barrels are ground
  level; red barrels are not ground level.
- Workshop 1's Offboard example uses `set_velocity_ned`, not long absolute
  position jumps. It also states PX4 needs setpoints at least every 0.5s.
- Workshop 2 says `x500_vision` provides a VIO-style local estimate through
  `LOCAL_POSITION_NED`, and that the estimate is relative to takeoff or vehicle
  initialization.
- Workshop 2 also notes the EKF origin setup command:
  `commander set_ekf_origin 47.397742 8.545594 488.0`.
- The supplementary VIO material says if the position estimate drifts
  indefinitely, the offboard controller may still be intact but the VIO/local
  estimate is not trustworthy.
- Workshop 2/3 emphasize depth histogram behavior: split depth into regions,
  classify left/center/right or polar bins, choose a safe direction, smooth the
  commanded velocity, and then command the drone.
- Workshop 3 recommends a virtual target approach: combine a goal vector with
  an avoidance vector, then project a short look-ahead setpoint. It warns that
  too large a look-ahead can cut corners through obstacles, while too small a
  look-ahead causes jitter.
- The exploration advice is lawnmower/BFS/DFS-style coverage with visited-grid
  memory, plus wall-following or backtracking when stuck.

## Practical Consequences For This Stack

- Treat impossible `position_velocity_ned()` jumps as a VIO/local-position
  health problem, not as a ROS2 planner bug.
- Keep a velocity-mode smoke test available. It proves MAVSDK/PX4 Offboard
  motion without trusting N/E position.
- For the scoring mission, prefer short running setpoints or velocity commands
  generated from depth histograms, then use global A*/coverage only as intent.
- The next practical architecture on this laptop is a single MAVSDK follower
  process that subscribes to depth and sends `VelocityNedYaw`; only add the
  mapper/A*/detector graph after that path survives a full smoke run.
- Before a serious run, set or verify EKF origin in PX4 and restart if local
  position drifts far beyond plausible motion.
