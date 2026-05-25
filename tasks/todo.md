# RoboVerse Qualifier Mission Improvement Plan

## Spec

- Target platform: `x500_vision` in the RoboVerse qualifier world.
- Primary goal: autonomously detect at least one yellow ground-level fuel barrel and at least one elevated red fuel barrel within the 10-minute attempt window.
- Navigation must remain responsive; camera image processing, detection, and photo saving must run in the background and must not block the main movement loop.
- Use local NED position only. Do not require GNSS/global position.
- Use `/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image` as the demo camera default while still allowing auto-discovery via `gz topic -l`.
- Preserve the current local exploration, obstacle avoidance, and target confirmation structure unless a smaller change cannot meet the spec.

## Checklist

- [x] Create a clean GitHub/team handoff folder with the current mission stack, helper scripts, docs, sample logs, and model.
- [x] Analyze corridor-improved run `mission_log_20260516_084415`.
- [x] Let corridor passability apply during the whole movement step, not only at move start.
- [x] Add PID-style corridor centering state for faster heading corrections.
- [x] Reduce unnecessary older-pose retreats from tiny side-clearance and pitch transients.
- [x] Re-run syntax/config verification.
- [x] Make YOLO burst inference opt-in/lazy-loaded to reduce CPU use.
- [x] Add explicit frontier deliberation/lookahead scoring instead of opaque planner reasoning.
- [x] Add UDP manual teleop override and Linux joystick bridge for controller testing.
- [x] Add keyboard UDP teleop fallback for controllers that do not expose Linux input devices.
- [x] Add strong persistent-candidate confirmation so high-confidence yellow sightings do not stay unconfirmed due depth/yaw wobble.
- [x] Re-run syntax/config verification for lightweight perception, deliberation, and teleop changes.
- [x] Analyze narrow-corridor behavior from `mission_log_20260516_083109`.
- [x] Split yaw clearance from forward-motion clearance so the drone can turn out of near-wall views.
- [x] Add narrow-corridor passability with centerline steering and shorter steps.
- [x] Add frontier fallback when all headings score as blocked instead of facing the same wall.
- [x] Re-run syntax/config verification.
- [x] Analyze early macro-frontier crash `mission_log_20260516_082451`.
- [x] Prevent local path-memory nudges from overriding authoritative frontier headings.
- [x] Keep legacy fixed-ring/local sweep behavior unchanged by making the nudge opt-out only for frontier strides.
- [x] Re-run syntax/config verification.
- [x] Analyze eligibility run `mission_log_20260516_081611`.
- [x] Add macro-sector frontier coverage instead of purely local greedy frontier.
- [x] Recenter between far macro sectors so coverage fans across the world.
- [x] Continue passive frontier exploration after eligibility by default.
- [x] Re-run syntax/config verification.
- [x] Analyze broad-but-local run `mission_log_20260516_075730`.
- [x] Replace default fixed ring coverage with fast open-frontier coverage.
- [x] Search missing colours with frontier strides instead of local full-world sweep.
- [x] Keep candidate revisit focused on colours still needed for eligibility.
- [x] Re-run syntax/config verification.
- [x] Analyze early blind-coverage critical-state run `mission_log_20260516_074939`.
- [x] Change instability and lower-clearance aborts from hold-in-place to safe-pose retreat.
- [x] Tighten lower-frame clearance before continuing blind coverage.
- [x] Re-run syntax/config verification.
- [x] Analyze ROS2 bridge run `mission_log_20260515_022313`.
- [x] Stop default post-eligibility full-world sweep that caused the late critical-state crash.
- [x] Add explicit opt-in for risky extra scoring after eligibility.
- [x] Re-run syntax/config verification.
- [x] Assess ROS2/PX4 bridge availability for `x500_vision`.
- [x] Add optional ROS2 sensor bridge support without breaking MAVSDK fallback.
- [x] Document required ROS2 bridge command and environment variables.
- [x] Re-run syntax verification.
- [x] Analyze blind coverage critical-state run `mission_log_20260515_021350`.
- [x] Change global coverage ordering from ring-major pivots to outward spoke-major travel.
- [x] Tighten per-step arrival radius so `0.30m` commands produce real distance.
- [x] Suppress routine photo bursts during blind movement.
- [x] Re-run syntax and configuration verification.
- [x] Analyze latest global-ring run for candidate-investigation time sinks.
- [x] Change mission order to blind global coverage first.
- [x] Keep passive candidate memory during blind coverage but defer active YOLO/investigation.
- [x] Add post-coverage candidate waypoint revisit phase.
- [x] Re-run syntax and configuration verification.
- [x] Analyze latest critical-state run `mission_log_20260515_014433` and undo risky efficiency tuning.
- [x] Add explicit global ring/sector coverage goals instead of relying only on local heading sweeps.
- [x] Keep obstacle avoidance local and conservative while steering toward far coverage goals.
- [x] Run global coverage before fallback and repeated local full-world sweeps.
- [x] Re-run syntax and configuration verification.
- [x] Analyze latest broad run `mission_log_20260514_234538` for time-efficiency bottlenecks.
- [x] Increase distance covered per movement step while preserving obstacle gates.
- [x] Reduce redundant stopped scans because continuous background perception is already active.
- [x] Reduce false unstable aborts without changing critical attitude safety.
- [x] Re-run syntax and configuration verification.
- [x] Analyze latest eligibility run `mission_log_20260514_233158` for remaining breadth limits.
- [x] Extend full-world sweep beyond a single low/high cycle while preserving landing reserve.
- [x] Rotate sweep headings between cycles so the drone does not repeat the same local loop.
- [x] Increase sweep advance distance modestly without changing crash safety thresholds.
- [x] Re-run syntax and configuration verification.
- [x] Analyze latest sticky-recovery run `mission_log_20260514_231528`.
- [x] Add broader recovery behavior so the drone retreats to older safe poses instead of the newest near-obstacle pose.
- [x] Skip blocked headings quickly so a pass keeps searching the world instead of grinding in one compartment.
- [x] Rebalance clearance thresholds using the crash states as hard unsafe cases.
- [x] Re-run syntax and log-state verification.
- [x] Analyze repeat crash log `mission_log_20260514_225701`.
- [x] Fix bug where unsafe pre-select clearance was logged but not enforced.
- [x] Remove in-place blocked yaw turns near obstacles.
- [x] Add last-known-safe pose recovery.
- [x] Re-run syntax and log-state verification.
- [x] Analyze crash log `mission_log_20260514_225113`.
- [x] Add side-clearance safety gates before movement.
- [x] Abort in-progress movement earlier on warning-distance obstacles, side clearance, or attitude drift.
- [x] Restore conservative range/sweep settings after out-of-world report.
- [x] Re-run syntax and log-based verification.
- [x] Change mission objective from early eligibility landing to timed full-world search.
- [x] Strengthen post-confirmation duplicate suppression for repeated sightings of the same nearby target.
- [x] Add a broad post-eligibility sweep that continues alternating yellow/red target modes.
- [x] Verify with latest successful run log and syntax checks.
- [x] Tune detector for actual barrel appearance from user screenshots.
- [x] Verify detector on saved run photos and synthetic/known target images.
- [x] Re-run syntax checks after detector tuning.
- [x] Analyze manual flight log/photos from `mission_log_20260514_222529`.
- [x] Install/verify `ultralytics` for optional YOLO burst detection.
- [x] Fix yaw-settle/movement mismatch so position steps use the intended heading.
- [x] Add deliberate stopped visual scan bursts to confirm candidates.
- [x] Reduce raw photo burst spam so navigation/perception keep priority.
- [x] Re-run syntax and smoke verification after fixes.
- [x] Review qualifier/supporting documents for mission constraints and topic guidance.
- [x] Review current `competition_mission.py`, `gzphotodetectorsaver.py`, detection, target memory, and obstacle-monitor code.
- [x] Fix `gzphotodetectorsaver.py` so burst capture/detection can actually run safely as a background task.
- [x] Move mission frame processing off the main asyncio loop using a background worker/executor.
- [x] Add non-blocking evidence photo capture after stopped motion/investigation/confirmation points.
- [x] Tighten x500_vision topic selection and startup diagnostics.
- [x] Run syntax/unit-style verification that does not require Gazebo.
- [x] Add review/results notes.

## GitHub Handoff Package

- Created `github_handoff_20260516/` as a clean package for teammate upload/review.
- Included active mission stack, imported helper modules, diagnostic/mapping/planning scripts, `Codes/yolov10n.pt`, sample detector assets, sample mission logs, and setup/refactor notes.
- Added package-local `README.md`, `DEPENDENCIES.md`, and `.gitignore`.
- Excluded bulk runtime outputs such as `competition_photos/`, `competition_evidence/`, full `bc_logs/`, `logs/`, and source PDFs.
- Verification:
  - `python3 -m py_compile github_handoff_20260516/*.py`

## Macro Frontier Crash Follow-Up

- Latest run crashed early in `FRONTIER_LOW_BOTH_00` after macro sector 02. The sector logic tried to peel from the long `97 deg` corridor toward `82 deg`/`62 deg`, but `move_in_heading()` then re-ran local path memory and selected `97 deg` again.
- Root cause: the new macro/frontier planner was choosing broader headings, but the lower movement layer could still nudge those headings by up to `35 deg`, which defeated whole-world coverage and kept the drone close to the same obstacle shelf.
- Fix: `move_in_heading()` now accepts `allow_memory_nudge=True`; `frontier_stride()` passes `allow_memory_nudge=False` so frontier-selected headings are authoritative while all pre-yaw, post-yaw, side, lower, and mid-step safety gates still run.
- Verification after follow-up:
  - `python3 -m py_compile competition_mission.py exploration_memory.py obstacle_monitor.py target_memory.py small_fuel_detector.py gzphotodetectorsaver.py`
  - Import/config check confirmed frontier mode remains the default and fixed-ring fallback remains opt-in.

## Narrow Corridor Follow-Up

- Latest run reached eligibility but still spent too much time near local compartments. Many aborts happened with one side near `1.60m` while the forward/lower lane was still open, so the planner treated passable aisles as blocked.
- Fixes:
  - Added yaw-specific clearance, so the drone can rotate out of a near-front obstacle without requiring full forward-motion clearance in the current view.
  - Added narrow-corridor mode: if front/lower clearance are healthy and side clearance is above an absolute `1.35m` floor, the move is allowed as a corridor traversal.
  - Corridor traversal blends yaw toward the wider side, logs `corridor_mode`, and uses a shorter `0.20m` step instead of the normal `0.30m`.
  - Frontier selection now gives passable corridors a bonus and falls back to a side-escape heading if all candidates score as blocked.
- Verification after follow-up:
  - `python3 -m py_compile competition_mission.py exploration_memory.py obstacle_monitor.py target_memory.py small_fuel_detector.py gzphotodetectorsaver.py`
  - Import/config check confirmed `NARROW_CORRIDOR_ENABLED=True`, side floor `1.35m`, and corridor step `0.20m`.

## Corridor Recovery/PID Follow-Up

- Latest run was much better: it reached eligibility, continued post-eligibility exploration, and visited `46` cells before a late critical-state stop. The remaining time sink was frequent `Recovering to older safe pose` after side readings around `1.64-1.65m` and pitch transients around `5.1-5.6 deg`.
- Fixes:
  - Corridor passability now applies throughout `move_to_position_step()`, so a move that starts in open space can keep going when side clearance briefly dips into the corridor band.
  - Mid-step attitude abort now allows up to `6.5 deg` before retreating, while the hard `10 deg` critical stop remains unchanged.
  - Added a persistent PID-style corridor steering controller with P/I/D terms and a clamped `12 deg` output.
  - Kept frontier heading scoring stateless, so candidate evaluation does not mutate the live corridor PID controller.
  - Corridor motion now uses a shorter `0.75s` rolling-setpoint timeout to keep narrow-passage decisions fast.
  - Corridor minimums are now `front >= 2.00m` and side `>= 1.30m`, matching the observed passable aisle cases while still rejecting genuinely tight shelves.
  - Exploration moves pass corridor capability into the mid-step monitor by default, but return-home keeps its separate conservative path.
- Verification after follow-up:
  - `python3 -m py_compile competition_mission.py exploration_memory.py obstacle_monitor.py target_memory.py small_fuel_detector.py gzphotodetectorsaver.py`
  - Import/config check confirmed mid-step attitude `6.5`, corridor front `2.0`, side `1.3`, and PID gains.

## Lightweight Perception, Deliberation, And Teleop Follow-Up

- `Codes/yolov10n.pt` is already a nano-class model, so the main CPU fix is to keep YOLO burst inference opt-in instead of loading/running it continuously.
- YOLO is now lazy-loaded only when `YOLO_BURST_ENABLED=1`; default mission import no longer loads `ultralytics` or `torch`.
- Continuous HSV perception can be slowed with `PERCEPTION_PERIOD_S` when CPU is tighter than visual latency.
- Frontier selection now includes explicit projected-cell lookahead scoring. Enable `FRONTIER_DELIBERATION_LOG=1` to print top candidate headings and their scores.
- Added `TELEOP_ENABLED=1` UDP manual override in `competition_mission.py` and `teleop_udp_bridge.py` for Linux joystick devices.
- Added `TELEOP.md` with setup, axis mapping, deadman-button, and DJI RC-N3 caveats.
- After `mission_log_20260522_020448`, added a strong persistent-candidate confirmation path for cases like yellow `count=11`, `conf=0.98` that stayed unconfirmed because strict yaw/depth checks were too brittle.
- Verification:
  - `python3 -m py_compile competition_mission.py exploration_memory.py gzphotodetectorsaver.py teleop_udp_bridge.py obstacle_monitor.py target_memory.py small_fuel_detector.py ros2_sensor_bridge.py`
  - Import/config check confirmed `YOLO_BURST_ENABLED=False`, `TELEOP_ENABLED=False`, `FRONTIER_DELIBERATION_ENABLED=True`, and no `ultralytics`/`torch` import by default.
  - `GZPhotoDetectorSaver(enable_yolo=False)` confirmed no YOLO/torch import.
  - Strong persistent-candidate smoke test confirmed a repeated high-confidence yellow candidate returns `confirmation_reason=strong_persistent`.

## Review

- Qualifier document constraints reviewed: 10-minute attempt, local/GNSS-free mission, yellow ground-level and red elevated fuel barrels, University eligibility requires at least one of each.
- Kept the existing local-NED exploration/target-memory mission architecture because it already matches the challenge better than a full rewrite.
- Repaired `gzphotodetectorsaver.py` so capture bursts run in the background, tolerate missing `ultralytics`, ignore idle frames, and save unique image files without blocking the navigation loop.
- Updated `competition_mission.py` to prefer `x500_vision_0` image topics, start the camera saver as a background task, process visual detection through an executor, and trigger raw photo bursts only after stopped motion/confirmation points.
- Verification run:
  - `python3 -m py_compile competition_mission.py gzphotodetectorsaver.py small_fuel_detector.py obstacle_monitor.py target_memory.py exploration_memory.py`
  - Fake Gazebo image smoke test for `GZPhotoDetectorSaver.trigger_capture_burst()`.
  - Import/config check for `competition_mission.py` confirmed the x500_vision fallback topic and `/depth_camera`.
- Gazebo/PX4 flight verification was not run in this shell; it requires the simulator/drone stack to be active.

## Manual Run Follow-Up

- Manual run on `x500_vision`/`roboverse` took off, navigated, returned, logged cleanly, and landed, but scored `0` with two unconfirmed candidates.
- Root-cause adjustments after the run:
  - Added yaw settling before movement and compute movement targets from the selected command heading instead of stale live yaw.
  - Added stopped multi-frame scans after holds/motion and during investigation yaw offsets.
  - Relaxed target confirmation from 4 to 3 sightings and widened stale candidate lifetime while retaining confidence, depth, and yaw consistency checks.
  - Reduced raw photo bursts from 3 frames to 1 and rate-limited bursts to keep camera I/O from crowding navigation/perception.
  - Installed `ultralytics` and configured optional YOLO bursts using `Codes/yolov10n.pt`.
- Verification after follow-up:
  - `python3 -m py_compile competition_mission.py gzphotodetectorsaver.py small_fuel_detector.py obstacle_monitor.py target_memory.py exploration_memory.py`
  - `cv2`, `numpy`, and `ultralytics` import successfully.
  - `GZPhotoDetectorSaver` loads `Codes/yolov10n.pt`.

## Actual Target Appearance Follow-Up

- User screenshots show the red-scoring barrel is visually a white/orange-banded elevated barrel, not a saturated red object.
- Detector tuning now treats red targets as elevated red/orange warm-colour barrel bands and allows squat/angled far-away red bboxes.
- Lowered mission confidence threshold to `0.52` because the old run had a real far red target that scored around `0.53` before tuning.
- Verification after tuning:
  - Saved run photos now produce 15 red detections, with top confidence around `0.83`.
  - `actual_targets_test.png` and `success/redfuelbarrel.jpg` both detect red targets.
  - `python3 -m py_compile competition_mission.py small_fuel_detector.py gzphotodetectorsaver.py obstacle_monitor.py target_memory.py exploration_memory.py`

## Full-World Search Follow-Up

- Latest manual run confirmed the mission can achieve eligibility, but it landed after `1` red and `8` yellow confirmations.
- Updated mission behavior so eligibility no longer ends exploration. The drone now continues searching until the mission time budget approaches landing reserve.
- High red pass always runs after the low yellow pass, even if yellow/red eligibility was already achieved early.
- Added `FULL_WORLD_SCORE_SWEEP` that alternates low/high altitude searches for both colours using broader headings.
- Expanded local range limits for the 40m x 40m world while preserving return-home safety.
- Strengthened duplicate suppression by increasing mission duplicate thresholds and adding bearing-plus-depth duplicate suppression when depth-localized N/E positions jitter.
- Verification:
  - `python3 -m py_compile competition_mission.py target_memory.py small_fuel_detector.py gzphotodetectorsaver.py obstacle_monitor.py exploration_memory.py`

## Crash Safety Follow-Up

- Manual run `mission_log_20260514_225113` stopped on `critical_state` after contact: roll reached `40.2 deg`.
- Root cause from CSV: before the crash, center/front clearance was still about `4.06m`, but right clearance was only about `1.58m`; the previous safety gate was mostly center-depth based.
- Safety changes:
  - Reduced move step from `0.45m` to `0.32m` and return step from `0.65m` to `0.45m`.
  - Restored conservative range limits: soft `17.5m`, hard `22.0m`, resume `13.5m`.
  - Added pre-move side clearance gate: left/right must both be at least `1.75m`.
  - Added pre-move front/lower gates and mid-step aborts on clearance or attitude drift.
  - Lowered critical attitude threshold from `18 deg` to `10 deg`.
  - Reduced blocked-streak escape threshold from `7` to `4`.
- Log-based verification: the crash move would now be blocked because right clearance was below the `1.75m` side gate.
- Syntax verification:
  - `python3 -m py_compile competition_mission.py obstacle_monitor.py target_memory.py small_fuel_detector.py gzphotodetectorsaver.py exploration_memory.py`

## Repeat Crash Recovery Follow-Up

- Manual run `mission_log_20260514_225701` still crashed. The log showed the previous safety patch logged unsafe clearance but did not enforce it before moving/yawing.
- Root cause: blocked handling still performed in-place yaw turns near obstacles. At `LOW_YELLOW_PASS_03_00`, the vehicle attempted a large yaw change in a constrained area, then hit `pitch=29.6`, `roll=-37.2`.
- Fixes:
  - Unsafe pre-move clearance now immediately triggers recovery instead of movement.
  - Unsafe post-yaw clearance now triggers recovery instead of another in-place yaw turn.
  - Added `last_safe_position` memory and `recover_to_last_safe()`.
  - Large yaw rotations are now split into `25 deg` increments with clearance checks before each increment.
  - Extra full-world sweep is limited to one conservative low/high cycle.
- Log-state verification:
  - The `LOW_YELLOW_PASS_01_01` state with right clearance `1.746m` is now unsafe.
  - The `LOW_YELLOW_PASS_03_00` crash state with front `1.51m`, right `1.24m`, lower `0.33m` is now unsafe.
- Syntax verification:
  - `python3 -m py_compile competition_mission.py obstacle_monitor.py target_memory.py small_fuel_detector.py gzphotodetectorsaver.py exploration_memory.py`

## Breadth Recovery Follow-Up

- Latest manual run avoided the crash but got stuck repeatedly recovering to the same near-obstacle pose, then completed with only `1` red and `0` yellow.
- Root cause: `last_safe_position` was a single newest pose, so recovery could target a barely-safe pose inside a constrained pocket.
- Changes:
  - Added a short safe-pose history and recovery selection that prefers an older/farther safe pose.
  - Reduced movement step to `0.30m`.
  - Rebalanced clearances so known crash states remain unsafe while borderline stuck states can move: front gate `2.00m`, side gate `1.65m`.
  - Added per-heading failure limits so blocked headings are skipped after two failed moves.
  - Added pass-level escape after repeated blocked headings, using return-home/recovery before changing pass.
- Verification:
  - `python3 -m py_compile competition_mission.py obstacle_monitor.py target_memory.py small_fuel_detector.py gzphotodetectorsaver.py exploration_memory.py`
  - Log-state check still rejects the crash states `front=1.51/right=1.24` and `right=1.58`, while allowing sticky states around `front=2.02` and `right=1.68`.

## Extended Sweep Follow-Up

- Manual run `mission_log_20260514_233158` achieved eligibility without crashing: `1` red, `1` yellow, score `150`.
- Remaining issue: the run still only visited `8` cells because the post-eligibility sweep stopped after one low/high cycle and each heading advanced only `0.90m`.
- Changes:
  - Increased sweep steps per heading from `3` to `5`.
  - Added up to `3` full-world sweep cycles while still respecting the landing reserve.
  - Rotates sweep headings by `20 deg` each cycle so repeated cycles probe different rays instead of retracing the same local loop.
  - Left crash safety gates unchanged: front `2.00m`, side `1.65m`, lower `0.85m`.
- Verification:
  - `python3 -m py_compile competition_mission.py obstacle_monitor.py target_memory.py small_fuel_detector.py gzphotodetectorsaver.py exploration_memory.py`
  - Config check: sweep steps `5`, max cycles `3`, rotation `20.0`, front gate `2.0`, side gate `1.65`.

## Time Efficiency Follow-Up

- Manual run `mission_log_20260514_234538` reached `40` visited cells, confirmed `2` red and `1` yellow, and landed on time budget instead of a crash.
- Remaining issue: the route was safe and broad, but spent too much time on short stop/yaw/scan cycles.
- Changes:
  - Increased move step from `0.30m` to `0.34m`.
  - Kept crash safety gates unchanged: front `2.00m`, side `1.65m`, lower `0.85m`.
  - Reduced routine stopped scanning to `1` frame every `2` movement steps. Background perception still runs continuously, and candidate events still trigger investigation.
  - Raised non-critical attitude tolerance from `5 deg` to `6 deg` to avoid wasting time on harmless pitch blips; critical stop remains `10 deg`.
- Verification:
  - `python3 -m py_compile competition_mission.py obstacle_monitor.py target_memory.py small_fuel_detector.py gzphotodetectorsaver.py exploration_memory.py`
  - Config check: move step `0.34`, max attitude `6.0`, critical attitude `10.0`, stop scan interval `2`, stop scan frames `1`, front gate `2.0`, side gate `1.65`.

## Global Coverage Follow-Up

- Manual run `mission_log_20260515_014433` confirmed eligibility and extra yellow detections, but ended with `critical_state` during the first local sweep cycle.
- Root cause direction: increasing step size and loosening attitude tolerance traded away safety margin, while the search planner still relied on local sweep headings instead of explicit world coverage goals.
- Changes:
  - Restored conservative motion: move step `0.30m`, non-critical attitude gate `5 deg`, critical attitude still `10 deg`.
  - Added explicit global ring/sector coverage goals using local NED only.
  - Coverage rings: `4m`, `7m`, `10m`, `13m`, `16m`.
  - Coverage sectors: `0`, `45`, `90`, `135`, `180`, `-135`, `-90`, `-45` degrees relative to start yaw.
  - The global coverage pass runs low and high altitude before fallback and before the old repeated local full-world sweep.
  - Each sector still uses `move_in_heading()` in small guarded steps, so obstacle clearance remains local/reactive.
- Verification:
  - `python3 -m py_compile competition_mission.py obstacle_monitor.py target_memory.py small_fuel_detector.py gzphotodetectorsaver.py exploration_memory.py`
  - Config check: move step `0.30`, max attitude `5.0`, rings `[4.0, 7.0, 10.0, 13.0, 16.0]`, `40` global goals per altitude, front gate `2.0`, side gate `1.65`.

## Blind Coverage Follow-Up

- Latest manual run reached much farther with global rings and scored `450`, but spent too much time actively investigating repeated candidate clusters during coverage.
- Mission order now starts with blind global ring coverage for both low/high altitudes, then revisits stored candidate waypoints for active confirmation.
- During blind coverage:
  - Passive detector updates `TargetMemory` and remembers candidate N/E locations.
  - Active investigation, YOLO bursts, and routine stopped scans are deferred.
  - Confirmation photo bursts are suppressed to avoid heavy I/O during map coverage.
- Candidate memory lifetime now spans the mission budget so deferred candidates do not expire before revisit.
- Coverage rings are now `[4.0, 8.0, 12.0, 16.0]` with a looser `1.6m` reached radius and `14` guarded steps per goal, making the ring sweep less granular and faster.
- Added `CANDIDATE_REVISIT`, which ranks stored candidates, flies to a standoff waypoint, faces the estimated target, scans, and then runs active investigation.
- Verification:
  - `python3 -m py_compile competition_mission.py target_memory.py small_fuel_detector.py gzphotodetectorsaver.py obstacle_monitor.py exploration_memory.py`
  - Config check: `32` global goals per altitude, candidate stale lifetime `540s`, global goal steps `14`.
  - Synthetic candidate check produced a standoff revisit waypoint from stored target N/E samples.

## Blind Coverage Critical-State Follow-Up

- Manual run `mission_log_20260515_021350` showed blind coverage successfully deferred investigations, but ring-major goal order caused tight pivots around the spawn pocket.
- Root cause details:
  - Goal order tried all sectors at the `4m` ring before going outward, so after moving east it immediately tried to pivot toward adjacent/behind sectors.
  - The `0.30m` movement command was considered complete at `0.20m` target error, so many steps only moved about `0.10m` to `0.15m`.
  - Routine photo bursts were still triggered after blind movement steps, adding I/O during pure coverage.
- Changes:
  - Global coverage goals are now spoke-major: complete `[4, 8, 12, 16]m` on one heading before turning to the next spoke.
  - Heading order now alternates around the start yaw: `[0, 45, -45, 90, -90, 135, -135, 180]`.
  - Move completion radius reduced to `0.10m` so a `0.30m` command produces more actual translation.
  - Blind mode skips large yaw turns when side clearance is tight instead of rotating in place.
  - Blind-mode routine blocked recovery prefers the latest safe pose instead of retreating to older spawn-pocket poses; instability/lower-clearance aborts now use older safe-pose retreat.
  - Routine move-complete photo bursts are suppressed while active investigation is disabled.
- Verification:
  - `python3 -m py_compile competition_mission.py target_memory.py small_fuel_detector.py gzphotodetectorsaver.py obstacle_monitor.py exploration_memory.py`
  - Config check: first 12 coverage goals are `(4,8,12,16)m` on heading `0`, then `(4,8,12,16)m` on heading `45`, then `(4,8,12,16)m` on heading `-45`.

## ROS2 Bridge Follow-Up

- ROS2 Humble, `rclpy`, `sensor_msgs`, and `ros_gz_bridge` are installed locally.
- `px4_msgs` is not importable, so full ROS2 PX4 offboard command control is not safe to switch on yet.
- Kept MAVSDK as the command path because arming, takeoff, and Offboard commands are already working in manual runs.
- Added optional ROS2 sensor bridge mode:
  - `USE_ROS2_SENSOR_BRIDGE=1 python3 competition_mission.py`
  - `ROS2_IMAGE_TOPIC` and `ROS2_DEPTH_TOPIC` can override topic names.
  - If ROS2 sensor mode is unavailable, the mission falls back to direct Gazebo transport.
- Added `ROS2_BRIDGE.md` with the `ros_gz_bridge parameter_bridge` command for the `x500_vision` RGB/depth topics.
- Verification:
  - `python3 -m py_compile competition_mission.py ros2_sensor_bridge.py`
  - `ros2 pkg prefix ros_gz_bridge` resolved to `/opt/ros/humble`.
  - Import/config check reported `ROS2_AVAILABLE=True` and ROS2 bridge disabled by default.
  - `ros2 topic list` currently shows only `/parameter_events` and `/rosout`, so the simulator/bridge topics were not active in this shell during verification.

## ROS2 Bridge Run Follow-Up

- Manual run with `USE_ROS2_SENSOR_BRIDGE=1` confirmed the ROS2 sensor bridge works: RGB/depth frames arrived and the mission flew normally.
- The run reached `red=3`, `yellow=2`, score `400`, then crashed later during the old post-eligibility `FULL_WORLD_SCORE_SWEEP`.
- Root cause direction: after coverage/revisit/fallback already achieved eligibility, the extra sweep kept investigating repeated red candidates and moved into constrained shelves until `critical_state`.
- Change:
  - Default behavior now skips the extra full-world scoring sweep once eligibility is met.
  - If eligibility is first reached during `FULL_WORLD_SCORE_SWEEP`, the sweep now stops before starting the next low/high scoring pass.
  - Extra scoring remains available only by explicit opt-in:
    - `EXTRA_SCORING_AFTER_ELIGIBILITY=1 python3 competition_mission.py`
- Verification:
  - `python3 -m py_compile competition_mission.py ros2_sensor_bridge.py target_memory.py small_fuel_detector.py gzphotodetectorsaver.py obstacle_monitor.py exploration_memory.py`
  - Config check: `EXTRA_SCORING_AFTER_ELIGIBILITY=False` by default.

## Early Blind-Coverage Critical-State Follow-Up

- Manual run `mission_log_20260516_074939` failed during blind global coverage before scoring/investigation became relevant.
- Root cause direction: a lower-frame clearance abort (`lower=0.82`) and then repeated unstable holds left the drone in a bad pocket until roll exceeded the critical threshold.
- Change:
  - Raised `MIN_LOWER_MOVE_CLEARANCE_M` from `0.85m` to `1.05m`.
  - Mid-step clearance/attitude aborts now retreat to safe-pose history instead of holding in place.
  - Pre-move instability now retreats instead of issuing another hold at the current pose.
  - Movement logs now print `lower=` so the next run can confirm whether lower-frame clearance is driving aborts.
  - Set `YOLO_CONFIG_DIR=/tmp/Ultralytics` by default to keep Ultralytics settings warnings out of mission logs.
- Verification:
  - `python3 -m py_compile competition_mission.py ros2_sensor_bridge.py target_memory.py small_fuel_detector.py gzphotodetectorsaver.py obstacle_monitor.py exploration_memory.py`
  - Config check: `MIN_LOWER_MOVE_CLEARANCE_M=1.05`.

## Fast Frontier Coverage Follow-Up

- Manual run `mission_log_20260516_075730` stayed safe and visited `42` cells, but it still spent too much time in one area.
- Root cause direction:
  - Fixed ring goals kept enumerating unreachable sectors from inside the same shelf pocket.
  - Candidate revisit chased an already-confirmed or low-value colour while yellow was still missing.
  - The default fallback dropped into local sweep behavior instead of continuing broad exploration.
- Change:
  - Default coverage is now open-frontier coverage, not fixed ring coverage.
  - `USE_FIXED_RING_COVERAGE=1` can opt back into the old ring-goal planner for comparison.
  - Frontier coverage chooses open, less-visited headings and advances in multi-step strides.
  - Missing colours use frontier search strides instead of `FULL_WORLD_SCORE_SWEEP`.
  - Candidate revisit focuses on colours still needed for eligibility unless extra scoring is explicitly enabled.
- Verification:
  - `python3 -m py_compile competition_mission.py ros2_sensor_bridge.py target_memory.py small_fuel_detector.py gzphotodetectorsaver.py obstacle_monitor.py exploration_memory.py`
  - Config check: `USE_FIXED_RING_COVERAGE=False`, `FRONTIER_LOW_STRIDES=10`, `FRONTIER_HIGH_STRIDES=8`, `FRONTIER_STEPS_PER_STRIDE=9`.

## Macro Frontier World-Coverage Follow-Up

- Manual run `mission_log_20260516_081611` achieved eligibility safely (`red=1`, `yellow=1`) but still covered mostly one long corridor before landing.
- Root cause direction:
  - The frontier scorer was too locally greedy, so it followed the best open hallway to the far edge.
  - High/frontier recovery then spent time trying large turns near the edge.
  - The mission landed once eligibility was reached, leaving world coverage incomplete.
- Change:
  - Frontier coverage now uses macro compass sectors `[0, 45, -45, 90, -90, 135, -135, 180]` relative to start yaw.
  - When switching far-away macro sectors, the drone recenters toward the start before fanning into the next sector.
  - Blind tight-turn skipping now uses `BLIND_TIGHT_TURN_SIDE_CLEARANCE_M=2.05` instead of the older broad `3.0m` threshold.
  - After eligibility, default behavior continues passive frontier exploration until the landing reserve.
  - Set `CONTINUE_FRONTIER_AFTER_ELIGIBILITY=0` to restore early landing after eligibility.
- Verification:
  - `python3 -m py_compile competition_mission.py ros2_sensor_bridge.py target_memory.py small_fuel_detector.py gzphotodetectorsaver.py obstacle_monitor.py exploration_memory.py`
  - Config check: `CONTINUE_FRONTIER_AFTER_ELIGIBILITY=True`, `FRONTIER_MACRO_HEADINGS_DEG=[0, 45, -45, 90, -90, 135, -135, 180]`, `FRONTIER_SECTOR_RESET_RANGE_M=10.0`, `FRONTIER_SECTOR_RESUME_RANGE_M=7.0`.
