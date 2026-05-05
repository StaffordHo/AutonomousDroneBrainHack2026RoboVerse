# RoboVerse 2026 Qualifier Environment Setup Documentation

## Purpose

This document records the setup process used to prepare an Ubuntu 22.04 machine for the RoboVerse 2026 Qualifier, including PX4 SITL, Gazebo Harmonic, MAVSDK, OpenVINS, RoboVerse world files, and the final simulation launch procedure.

It will be updated as the project progresses.

---

## System Context

- Machine: Lenovo Legion 5 Pro 16ACH6H
- User: `stafford99`
- Operating system: Ubuntu 22.04 Jammy
- Simulator: Gazebo Sim v8.11.0
- PX4: PX4-Autopilot SITL
- Main challenge world: `roboverse.sdf`
- Main vehicle options:
  - `x500_vision`
  - `x500_depth`

---

## Part 1: Initial APT Update and Upgrade

### 1. Update package lists

```bash
sudo apt update
```

During the first update, the system reported that many packages could be upgraded. It also displayed non-fatal warnings related to repositories that do not support the `i386` architecture and a Gazebo repository key stored in the legacy trusted keyring.

### 2. Upgrade packages

```bash
sudo apt upgrade
```

The upgrade was initially blocked because `unattended-upgrade` was holding the APT lock:

```text
Could not get lock /var/lib/dpkg/lock-frontend. It is held by process ... unattended-upgr
```

### 3. Check unattended-upgrade process

```bash
ps -p 10987 -o pid,cmd
```

The process was confirmed to be:

```text
/usr/bin/python3 /usr/bin/unattended-upgrade
```

### 4. Inspect unattended-upgrades log

```bash
tail -f /var/log/unattended-upgrades/unattended-upgrades.log
```

### 5. Stop unattended upgrades if needed

```bash
sudo systemctl stop unattended-upgrades
sudo kill 10987
```

In this case, the process had already exited by the time `kill` was run.

### 6. Repair and continue APT operations

```bash
sudo dpkg --configure -a
sudo apt --fix-broken install
sudo apt update
sudo apt upgrade
```

The upgrade completed successfully, including package triggers such as `ca-certificates`, `ufw`, `man-db`, and `initramfs-tools`.

---

## Part 2: Install Development Essentials

### 7. Install build and utility tools

```bash
sudo apt install -y build-essential curl git wget software-properties-common
```

### 8. Install Python tooling

```bash
sudo apt install -y python3-pip python3-venv
```

### 9. Install Python OpenCV

```bash
sudo apt install -y python3-opencv
```

### 10. Install OpenCV development headers

```bash
sudo apt install -y libopencv-dev
```

### 11. Install Gazebo message and transport packages

```bash
sudo apt install -y libgz-msgs10-dev
sudo apt-get install -y python3-gz-transport13 python3-gz-msgs10
```

---

## Part 3: Install PX4 and MAVSDK

### 12. Clone PX4-Autopilot

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

### 13. Run PX4 Ubuntu setup script

```bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

A reboot is recommended after running the PX4 setup script.

### 14. Install MAVSDK Python package

```bash
python3 -m pip install --user mavsdk
```

### 15. Download MAVSDK C++ development package

```bash
cd /tmp
wget https://github.com/mavlink/MAVSDK/releases/download/v3.17.1/libmavsdk-dev_3.17.1_ubuntu22.04_amd64.deb
```

### 16. Install MAVSDK C++ development package

The original command contained a typo:

```bash
sudo apt install libmavsdk-dev_3.17.1._ubuntu22.04_amd64.deb
```

The corrected command is:

```bash
sudo apt install ./libmavsdk-dev_3.17.1_ubuntu22.04_amd64.deb
```

The important differences are:

- Use `./` for a local `.deb` file.
- Remove the extra dot after `3.17.1`.

---

## Part 4: Install and Build OpenVINS

### 17. Install OpenVINS dependencies

```bash
sudo apt-get install -y libeigen3-dev libboost-all-dev libceres-dev libgz-transport13-dev libgz-msgs10-dev
```

### 18. Clone OpenVINS

```bash
mkdir -p ~/tools
cd ~/tools
git clone https://github.com/rpng/open_vins/
```

### 19. Enter OpenVINS MSCKF directory

```bash
cd ~/tools/open_vins/ov_msckf/
```

### 20. Create build directory

```bash
mkdir -p build
cd build
```

### 21. Configure OpenVINS without ROS

```bash
cmake -DENABLE_ROS=OFF ..
```

### 22. Build OpenVINS

```bash
make -j4
```

The build completed successfully, reaching 100% and creating targets such as:

- `ov_msckf_lib`
- `run_simulation`
- `test_sim_meas`
- `test_sim_repeat`

### 23. Install OpenVINS

```bash
sudo make install
```

The install placed headers under:

```text
/usr/local/include/open_vins/
```

and installed binaries such as:

```text
/usr/local/bin/run_simulation
/usr/local/bin/test_sim_meas
/usr/local/bin/test_sim_repeat
```

---

## Part 5: Setup Environment Variables

### 24. Edit `~/.bashrc`

The following command should be used to edit the file:

```bash
nano ~/.bashrc
```

Do not run `~/.bashrc` directly. Running it directly may produce:

```text
Permission denied
```

Add the following lines to the end of `~/.bashrc`:

```bash
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
export OGRE_RTT_MODE=Copy
export PX4_GZ_SIM_RENDER_ENGINE=ogre
export GZ_SIM_RENDER_ENGINE=ogre
```

Save and exit Nano:

```text
Ctrl + O
Enter
Ctrl + X
```

Apply the changes:

```bash
source ~/.bashrc
```

Verify:

```bash
echo $PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION
echo $OGRE_RTT_MODE
echo $PX4_GZ_SIM_RENDER_ENGINE
echo $GZ_SIM_RENDER_ENGINE
```

Expected output:

```text
python
Copy
ogre
ogre
```

---

## Part 6: OpenCV Include Symlink

### 25. Create OpenCV symlink

First confirm OpenCV headers exist:

```bash
ls /usr/include/opencv4/opencv2
```

If the headers are present, create the symlink:

```bash
sudo ln -s /usr/include/opencv4/opencv2 /usr/include/opencv2
```

Do not use a trailing slash on the destination path.

Verify:

```bash
ls -l /usr/include/opencv2
```

---

## Part 7: Install ROS Humble Gazebo Harmonic Bridge

### 26. Install `ros-humble-ros-gzharmonic`

```bash
sudo apt install ros-humble-ros-gzharmonic
```

The package installed successfully, including related packages such as:

- `ros-humble-ros-gzharmonic`
- `ros-humble-ros-gzharmonic-bridge`
- `ros-humble-ros-gzharmonic-image`
- `ros-humble-ros-gzharmonic-interfaces`
- `ros-humble-ros-gzharmonic-sim`
- `ros-humble-ros-gzharmonic-sim-demos`

---

## Part 8: Initial PX4 SITL Test

### Test PX4 with default `x500`

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

The test was successful. PX4 started, Gazebo launched, the world became ready, the model spawned, and PX4 entered the `pxh>` shell.

Important successful indicators:

```text
px4 starting.
INFO  [init] Gazebo world is ready
INFO  [init] Spawning Gazebo model
INFO  [gz_bridge] world: default, model: x500_0
INFO  [px4] Startup script returned successfully
pxh>
```

Warnings such as the following were considered non-fatal for this setup stage:

```text
Preflight Fail: ekf2 missing data
Preflight Fail: No connection to the GCS
```

---

## Part 9: RoboVerse World Setup

### Required files

The RoboVerse-specific setup requires these files:

1. `roboverse.sdf`
2. `base6.glb`
3. `start_px4.sh`

### Copy world files into PX4 Gazebo worlds directory

```bash
cp ~/Downloads/roboverse.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
cp ~/Downloads/base6.glb ~/PX4-Autopilot/Tools/simulation/gz/worlds/
```

Verify:

```bash
ls -l ~/PX4-Autopilot/Tools/simulation/gz/worlds/roboverse.sdf
ls -l ~/PX4-Autopilot/Tools/simulation/gz/worlds/base6.glb
```

Observed file sizes:

```text
roboverse.sdf: 4889 bytes
base6.glb: 38063584 bytes
```

### Copy project-provided launch script

```bash
cp ~/Downloads/start_px4.sh ~/
chmod +x ~/start_px4.sh
```

Verify:

```bash
ls -l ~/start_px4.sh
```

Expected permissions include executable bits:

```text
-rwxrwxr-x
```

---

## Part 10: Run RoboVerse Simulation

### Launch script

```bash
cd ~
~/start_px4.sh
```

The provided launch script shows the following options:

```text
--- Select the vehicle model ---
1) x500_vision
2) x500_depth

--- Select the simulation world ---
1) roboverse
2) aprilworld
3) empty

--- Start QGroundControl? ---
1) Yes
2) No
```

### Tested launch option

For RoboVerse with vision model:

```text
Vehicle: x500_vision
World: roboverse
QGroundControl: No
```

The launch succeeded with key messages:

```text
INFO  [init] Gazebo world is ready
INFO  [gz_bridge] world: roboverse, model: x500_vision_0
INFO  [px4] Startup script returned successfully
pxh>
```

---

## Part 11: Fix for Missing RoboVerse Mesh

### Problem

Gazebo initially showed mostly white blocks or failed to load the full RoboVerse environment. Running Gazebo directly showed:

```text
Parser configurations requested resolved uris, but uri [file:///base6.glb] could not be resolved.
Failed to load a world.
```

The problem was that `roboverse.sdf` referenced:

```xml
<uri>file:///base6.glb</uri>
```

This points to `/base6.glb` at the root of the filesystem, not the file inside the PX4 worlds directory.

### Fix

Replace both occurrences of:

```xml
<uri>file:///base6.glb</uri>
```

with:

```xml
<uri>file:///home/stafford99/PX4-Autopilot/Tools/simulation/gz/worlds/base6.glb</uri>
```

Command-line fix:

```bash
sed -i 's|file:///base6.glb|file:///home/stafford99/PX4-Autopilot/Tools/simulation/gz/worlds/base6.glb|g' ~/PX4-Autopilot/Tools/simulation/gz/worlds/roboverse.sdf
```

Verify:

```bash
grep -n "base6\|glb" ~/PX4-Autopilot/Tools/simulation/gz/worlds/roboverse.sdf
```

After this correction, RoboVerse loaded correctly.

---

## Part 12: NVIDIA Driver Issue and Gazebo Black Screen

### Problem

Gazebo initially opened as a black screen and crashed. Checking NVIDIA showed:

```bash
nvidia-smi
```

Output:

```text
Failed to initialize NVML: Driver/library version mismatch
NVML library version: 535.288
```

This indicates a mismatch between the loaded NVIDIA kernel module and NVIDIA user-space libraries, likely caused by the earlier system upgrade.

### Fix

Reboot the machine:

```bash
sudo reboot
```

After reboot, `nvidia-smi` worked and Gazebo rendering was restored.

---

## Part 13: Sensor Topic Investigation

### `x500_vision` topic check

Command:

```bash
gz topic -l | grep -E "camera|image|depth|rgb|scan"
```

For `x500_vision`, no usable RGB camera image topic was available. The only camera-related topic was:

```text
/world/roboverse/model/x500_vision_0/link/camera_link/sensor/camera_imu/imu
```

This means `x500_vision` is not suitable for direct image-based object detection in the current setup.

### `x500_depth` topic check

When launching `x500_depth` in `roboverse`, useful topics became available:

```text
/camera_info
/depth_camera
/depth_camera/points
/world/roboverse/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info
/world/roboverse/model/x500_depth_0/link/camera_link/sensor/IMX214/image
```

This makes `x500_depth` the better candidate for the qualifier because it provides:

- RGB image topic for detecting red/yellow fuel barrels.
- Depth camera topic for obstacle avoidance.
- Point cloud topic for 3D perception or avoidance.

Recommended vehicle for qualifier development:

```text
x500_depth
```

Recommended world:

```text
roboverse
```

---

## Part 14: Qualifier Challenge Requirements Summary

The RoboVerse Qualifier requires teams to autonomously navigate a spaceport without GNSS, using object detection and obstacle avoidance.

Key challenge details:

- Time limit: 10 minutes.
- Multiple attempts are allowed within the 10-minute window.
- Best attempt is used for scoring.
- Spaceport size: approximately 40 m × 40 m × 8 m.
- Each grid is approximately 4 m × 4 m.
- Yellow barrels are placed only on ground level.
- Red barrels are not placed on ground level.
- University teams must detect at least one yellow barrel and at least one red barrel for ranking eligibility.

Scoring:

- Yellow fuel barrel: 50 points each.
- Red fuel barrel: 100 points each.
- Bonus points are awarded for detecting all yellow or red fuels within 5 minutes.

Manual control using keyboard, mouse, controller, joystick, or gamepad is disallowed.

---

## Part 15: Recommended Qualifier Development Direction

### Proposed stack

- MAVSDK Python for autonomous drone control.
- Gazebo image topic for barrel detection.
- Gazebo depth topic or point cloud topic for obstacle avoidance.
- OpenCV for simple colour-based detection as the first implementation.

### Recommended vehicle

Use:

```text
x500_depth
```

instead of:

```text
x500_vision
```

because `x500_depth` exposes the required camera image and depth topics.

### Recommended high-level mission flow

```text
Start PX4 + Gazebo RoboVerse
Connect MAVSDK
Arm drone
Take off
Run autonomous search pattern
Read RGB camera image
Detect yellow/red barrels using OpenCV
Use depth data for obstacle checks
Log detections
Continue until time limit or search complete
Land
```

### Immediate next technical step

Create a camera subscriber that reads:

```text
/world/roboverse/model/x500_depth_0/link/camera_link/sensor/IMX214/image
```

and displays/saves frames for verifying OpenCV-based detection.

---

## Camera Frame Verification

A Python camera subscriber was created in:

```text
~/roboverse_qualifier/camera_test.py
```

After resolving the NumPy/OpenCV mismatch, the script successfully saved:

```text
~/roboverse_qualifier/camera_frame.png
```

The saved frame clearly shows coloured fuel barrels in the RoboVerse environment, including two red barrels and one yellow barrel. This confirms that the `x500_depth` RGB camera stream is suitable for the first OpenCV-based barrel detection prototype.

Next perception step:

- Initial HSV colour thresholding successfully detected the large red barrels and yellow barrel.
- False positives occurred because the yellow hazard stickers on the red barrels were also classified as yellow barrels.
- Fix implemented successfully: red barrels are detected first, then yellow detections are filtered using minimum contour area, vertical barrel aspect ratio, and centre-inside-red-bounding-box rejection.
- Verified detection output: two red barrels and one yellow barrel were correctly detected from `camera_frame.png`.
- Continue to log detections by colour, image location, timestamp, and confidence estimate.
- Next improvement: make detection robust to drone viewpoint changes, object distance, perspective distortion, partial occlusion, and lighting/shadow variation.

---

## Depth Camera / Obstacle Avoidance Update

Depth topic inspection completed:

```text
/depth_camera -> gz.msgs.Image, pixel_format_type: R_FLOAT32
/depth_camera/points -> gz.msgs.PointCloudPacked
```

This means obstacle avoidance can use `/depth_camera` directly as a floating-point depth image. The immediate plan is to subscribe to `/depth_camera`, extract the centre region of the depth image, compute a robust minimum/median distance, and stop/avoid if the forward distance is below a safety threshold.

---

## Autonomous Search Update

The altitude-aware autonomous search mission ran successfully. The drone launched, performed a multi-lane exploration pattern, executed altitude changes for high/low scanning, saved evidence images, and landed.

Observed result:

```text
Final search summary: red=15, yellow=9, total=24
```

This is not yet a reliable barrel count. The high number indicates repeated detections and false positives from red/yellow structures such as ladders, thin coloured features, and large barrel-like objects. Several confirmed bounding boxes were thin or very small, for example widths below 30 pixels or tall narrow yellow boxes, which suggests non-barrel features are still passing the filter.

Next technical fixes:

- Use depth or point cloud data to reject detections that are unlikely to be small barrel-sized objects.
- Add obstacle avoidance using the `/depth_camera` or `/depth_camera/points` topic before each motion step.
- Replace pixel-centre duplicate counting with pose/bearing-aware detection clustering during yaw scans.
- Save fewer but better evidence images by requiring stronger detection confidence before confirmation.

---

## Handoff / Agentic AI Continuation Prompt Update

A continuation prompt was prepared so an agentic AI can take over from the current RoboVerse qualifier progress. The prompt captures the current environment, working scripts, known issues, and the recommended next direction: moving from HSV colour thresholding to a trained object detector for actual small red/yellow fuel barrels, while keeping the MAVSDK navigation, depth obstacle monitoring, and evidence/score logic.

---

## Model Training Direction Update

The current colour-threshold detector is useful as a prototype, but it is not robust enough for the random qualifier map because it can confuse red ladders, large decorative barrels, coloured structures, and small fuel barrels. The next direction is to train an object detection model specifically for the small red and yellow fuel barrels.

Recommended approach:

- Use synthetic data generated from RoboVerse/Gazebo rather than manually collecting all data.
- Train an object detector first, not a reinforcement learning navigation policy.
- Classes: `red_fuel_barrel`, `yellow_fuel_barrel`.
- Start with YOLO-format annotations because it is simple and widely supported.
- Generate screenshots across different drone positions, yaw angles, altitudes, lighting/viewpoints, and barrel placements.
- Use the trained detector for perception, then keep the existing MAVSDK autonomy/search stack for navigation.
- Later, randomise barrel spawn locations and map layouts to improve generalisation.

Immediate next task:

- Build a dataset structure under `~/roboverse_qualifier/datasets/fuel_barrels_v1/`.
- Capture training images from RoboVerse.
- Manually label a small initial dataset first, then train a baseline detector.
- After baseline detection works, automate random scene generation and annotation if possible.

---

## Latest Autonomous Search Logic Update

The obstacle avoidance layer is confirmed to be active. During the latest autonomous search test, the drone reported front depth readings and stopped/yawed away when obstacles were too close, for example at approximately 1.49 m, 1.37 m, 1.18 m, and 1.24 m.

A logic issue was identified in the scan flow: scans with zero detections still printed `New fuel detected. Proceeding to next search area.` This happened because the yaw-scan helper treated completion of the perception task as a successful detection, even when the task ended due to timeout with no new confirmed fuel.

Fix to apply:

- `perception_scan()` now returns `(summary, found_new_detection)`.
- `yaw_for_or_until_detection()` returns `True` only if the perception task actually confirmed a new fuel detection.
- `yaw_scan()` now prints either `New fuel detected. Proceeding to next search area.` or `No fuel detected. Continuing exploration.` correctly.
- Evidence and score updates still happen only when a new confirmed fuel barrel is detected.

---

## Current Working Launch Procedure

1. Open terminal.
2. Run:

```bash
cd ~
~/start_px4.sh
```

3. Select:

```text
2) x500_depth
1) roboverse
2) No
```

4. In another terminal, confirm topics:

```bash
gz topic -l | grep -E "camera|image|depth|rgb|scan"
```

5. Confirm that image and depth topics are present:

```text
/world/roboverse/model/x500_depth_0/link/camera_link/sensor/IMX214/image
/depth_camera
/depth_camera/points
```

---

## Troubleshooting Notes

### `AttributeError: 'list' object has no attribute 'get'` after switching to small fuel detector

After switching `integrated_stationary_mission.py` from the old large-barrel detector to `small_fuel_detector.py`, the mission reached takeoff and entered the perception loop, but failed at:

```text
AttributeError: 'list' object has no attribute 'get'
```

Cause: `detect_small_fuel_barrels(frame)` returns a tuple:

```python
detections, yellow_mask, red_mask, raw_detections
```

but `logger.update(detections)` expects `detections` to be a list of detection dictionaries. If the function output is not unpacked, the logger receives the full tuple/list structure instead of just the detections list.

Fix applied:

```python
detections, _, _, _ = detect_small_fuel_barrels(frame)
```

instead of:

```python
detections = detect_small_fuel_barrels(frame)
```

After this fix, the mission completed without crashing, but the small-fuel detector returned `red=0, yellow=0` after takeoff. This means the code path works, but the actual small fuel barrels were not detected from the drone's post-takeoff camera viewpoint. Next debugging step: save the live hover camera frame and run `small_fuel_detector.py` on that frame to retune thresholds/area filters for the airborne viewpoint.


### Python OpenCV / NumPy mismatch

During the first Python camera subscriber test, importing OpenCV failed with:

```text
A module that was compiled using NumPy 1.x cannot be run in NumPy 2.2.6
AttributeError: _ARRAY_API not found
ImportError: numpy.core.multiarray failed to import
```

Cause: the installed `cv2` module was compiled against NumPy 1.x, but Python was loading NumPy 2.2.6, likely from a user-level `pip` installation.

Fix applied:

```bash
python3 -m pip install --user "numpy<2"
python3 -c "import numpy; print(numpy.__version__)"
python3 -c "import cv2; print(cv2.__version__)"
```

Verified working versions:

```text
numpy 1.26.4
cv2 4.5.4
```

After this fix, `camera_test.py` successfully received frames from the Gazebo camera:

```text
Received image: 1920x1080, format=3
Saved frame to camera_frame.png
```

A non-critical abort occurred after saving the image:

```text
terminate called without an active exception
Aborted (core dumped)
```

This appears to happen during cleanup/shutdown of the Gazebo transport subscriber after the frame has already been saved. The next version of the camera script should use a continuous loop with `KeyboardInterrupt`, or force exit after saving, to avoid cleanup-related aborts during testing.

### APT lock issue

If APT is locked by unattended upgrades:

```bash
ps -p <PID> -o pid,cmd
sudo systemctl stop unattended-upgrades
sudo dpkg --configure -a
sudo apt --fix-broken install
```

Avoid deleting lock files unless the locking process is confirmed dead.

### MAVSDK `.deb` install issue

Use:

```bash
sudo apt install ./libmavsdk-dev_3.17.1_ubuntu22.04_amd64.deb
```

not:

```bash
sudo apt install libmavsdk-dev_3.17.1._ubuntu22.04_amd64.deb
```

### Gazebo black screen after upgrade

Check:

```bash
nvidia-smi
```

If there is a driver/library mismatch, reboot:

```bash
sudo reboot
```

### RoboVerse mesh missing

Check:

```bash
gz sim -v 4 ~/PX4-Autopilot/Tools/simulation/gz/worlds/roboverse.sdf
```

If `base6.glb` cannot be resolved, fix the SDF path as shown in Part 11.

---

## Status

Current status:

- Ubuntu environment setup completed.
- PX4 SITL installed and tested.
- MAVSDK installed.
- OpenVINS built and installed.
- ROS Humble Gazebo Harmonic bridge installed.
- RoboVerse world files copied.
- `base6.glb` path issue fixed.
- NVIDIA driver issue resolved by reboot.
- RoboVerse simulation launches successfully.
- `x500_depth` confirmed to expose image and depth topics.
- `x500_depth` successfully launched in `roboverse`.
- PX4 reported `SYS_AUTOSTART=4002`, Gazebo world ready, model `x500_depth_0`, and `Startup script returned successfully`.
- OakD-Lite camera sensors were loaded, including `IMX214` and `StereoOV7251`.
- MAVSDK connection test completed successfully. The script connected to PX4 on UDP port `14540` and printed `Drone connected successfully!`.
- Gazebo camera image stream verified successfully. Echoing `/world/roboverse/model/x500_depth_0/link/camera_link/sensor/IMX214/image` produced image data with `pixel_format_type: RGB_INT8`, confirming that the RGB camera stream is publishing correctly.
- Python-side image decoding confirmed. `camera_test.py` saved `camera_frame.png`, and the image clearly shows two red barrels and one yellow barrel in the RoboVerse environment.
- Static image barrel detection completed successfully after filtering yellow sticker false positives.
- Live barrel detector completed successfully. `live_barrel_detector.py` continuously detected two red barrels and one yellow barrel from the Gazebo camera stream and stopped cleanly with `Ctrl+C`.
- Duplicate filtering / detection logging completed successfully. The logger initially showed `red=0, yellow=0`, then confirmed three detections after repeated frames: two red barrels and one yellow barrel. It then held the confirmed count at `red=2, yellow=1, total=3` instead of recounting the same barrels every frame.

Next task:

- Add detection logging and duplicate filtering so the same barrel is not counted repeatedly.
- MAVSDK takeoff test completed successfully after changing the connection string to `udpin://0.0.0.0:14540`.
- `takeoff_test.py` confirmed: drone connection, global/home/local health checks, arming, takeoff to 2.5 m, 8-second hover, landing, and script completion.
- Offboard velocity/local movement test completed successfully. `offboard_test.py` connected to PX4, confirmed local position, armed, took off, entered offboard mode, moved forward, moved right, yawed right, hovered, stopped offboard, and landed.
- Non-critical MAVSDK/PX4 warning observed: `Received ack for not-existing command: 176! Ignoring...`; mission execution continued normally.
- Integrated stationary mission completed successfully. `integrated_stationary_mission.py` connected to PX4, took off, ran the camera detector while hovering, confirmed two red barrels and one yellow barrel, saved evidence images, printed the perception summary, and landed.
- Observation: after takeoff, the camera viewpoint changed and some barrels became partially visible near the lower image boundary. This can reduce contour area or distort aspect ratio, causing missed detections.
- Partial-barrel tolerance was added, but it introduced a new duplicate-counting problem: the same red barrel can be detected once as the main visible barrel and again as a small bottom-clipped red contour near the image boundary.
- Example integrated mission result: the system reported `red=4, yellow=1, total=5`, even though the scene visually contained two red barrels and two yellow barrels. The extra red detections came from bottom-edge fragments of the same red barrels.
- Red duplicate merging improved successfully. Integrated stationary mission now reports the correct red count: `red=2`.
- Remaining issue: two stacked/overlapping yellow barrels are still merged into one large yellow contour, so the system reports `yellow=1` even when two yellow barrel tops/bodies are visible.
- While adding the yellow contour-splitting heuristic, `integrated_stationary_mission.py` hit a syntax error: `SyntaxError: 'return' outside function`, indicating that part of the pasted block was indented outside `detect_barrels()`. The safer fix is to overwrite the script using a here-document rather than manual Nano edits.
- Important correction: the large red/yellow industrial barrels previously detected were not the actual qualifier fuel barrels. They were visually useful for testing the camera pipeline and colour detector, but they should be treated as non-target objects/decoys.
- Actual fuel barrels are much smaller: the red fuel barrel appears inside a raised compartment box, while the yellow fuel barrel appears on the ground/middle of the scene. The detector must therefore be retuned for small target barrels rather than large full-size barrels.
- New perception direction: detect small red/yellow fuel barrels using colour segmentation plus scale filtering, vertical/cylindrical shape constraints, and contextual rules. Red barrels may be elevated/not on ground level; yellow barrels are ground-level targets.
- Next: create a new small-fuel-barrel detector and avoid counting the large decorative barrels as targets.
- While creating `yaw_scan_mission.py`, a syntax error occurred: `SyntaxError: 'await' outside function`, caused by pasting the yaw-scan block outside the async `main()` function. Safer fix: create a separate yaw-scan script that imports perception functions from `integrated_stationary_mission.py` instead of manually editing a large file.
- Yaw scan mission ran successfully: the drone connected, armed, took off, entered offboard mode, performed right/left/right yaw scan, ran perception concurrently, saved evidence images, landed, and completed.

## V3 Updates: Advanced Navigation & Mapping (Qualifier Final Architecture)

### 1. Goal + Avoidance Vector Navigation
- **Rationale**: The previous lane-following logic used `VelocityBodyYawspeed` moving strictly forward on a timer. If an obstacle was detected, the drone stopped completely. This was unreliable because battery voltage drops caused the drone to travel less distance over time, missing large portions of the map. Furthermore, stopping blindly at obstacles led to getting stuck.
- **Solution**: Integrated the `AvoidancePlanner.py` module. The drone now calculates a continuous set of flight velocities to avoid obstacles (using a depth histogram) while heavily biasing its choice toward a defined geographic target waypoint (Goal Vector).

### 2. Geographic Waypoint Search Grid
- **Rationale**: Time-based navigation was unreliable. We needed rigid geographic bounds to ensure the entire arena is searched.
- **Solution**: Built a strict NED waypoint pattern (30 meters forward, shifting right by 6 meters per lane, covering an 18-meter width). The drone uses the `AvoidancePlanner` to dynamically maneuver around obstacles while progressing toward these rigid waypoints.

### 3. Dynamic Yawing for Sensor Visibility
- **Rationale**: When using `PositionNedYaw` to reach a waypoint, the flight controller pitched aggressively, causing the depth camera to point at the floor. The planner thought the floor was a wall and reacted violently. Also, returning along a lane required flying backwards, blinding the forward-facing depth sensor.
- **Solution**: Switched back to `VelocityBodyYawspeed` but implemented dynamic yawing. The drone continuously calculates the angle to the target waypoint and physically turns its nose to face it. If the drone is facing >45 degrees away from the target, it stops moving forward and simply spins until the depth camera has a clear view of the flight path.

### 4. Global Occupancy Grid Mapping
- **Rationale**: A requirement for the competition finals is mapping the environment globally.
- **Solution**: Integrated `GlobalMapper.py`. Using the Gazebo IMX214 camera intrinsics, the `float32` depth map is projected into the global North-East-Down (NED) frame. As the drone explores, it builds a massive 2D matrix of obstacle points. 
- **Git Hotfix**: The initial map output (`global_obstacles.npy`) was 1.2 GB. This broke the GitHub push due to the 100MB file limit. The `.npy` file was successfully removed from the Git index, added to `.gitignore`, and the repository was successfully pushed.

### 5. Transition to VelocityNedYaw & Geofencing
- **Rationale**: The drone was sometimes "climbing" over walls or flying upwards into the sky upon collision. This was caused by using `VelocityBodyYawspeed`, where the vertical velocity (VZ) was tied to the drone's tilted body frame. When the drone pitched to brake or dodge, its "down" vector pointed diagonally, causing uncontrolled climbing.
- **Solution**: Upgraded the entire flight loop to `VelocityNedYaw`. This locks the vertical velocity to the true World Z-axis (NED). The drone now holds its 3.5m altitude with rock-solid precision regardless of its pitch or roll angle.
- **Geofencing**: Implemented a hard 18-meter radial geofence in `navigate_to_waypoint`. If the drone reaches the edge of the arena, its outward velocity is automatically clamped to 0.0, physically preventing it from exiting the map.

**Current Status:** 
- Advanced Navigation Architecture (V3.2) deployed with strict altitude hold and geofencing.
- All AvoidancePlanner math bugs (NaN/Inf) resolved.
- Code successfully pushed to GitHub.
- System is ready for the RoboVerse 2026 Qualifier!
```
