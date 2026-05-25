# Full-World Survey Workflow

Use this when the map layout is fixed but barrel locations may move. The survey
run explores the world, records confirmed and candidate barrel locations, and
exports second-run waypoint files.

## Check ROS2 Usage

```bash
cd ~/roboverse_qualifier/rrt_assisted_mission
python3 check_ros2_usage.py
```

By default the mission uses Gazebo transport. ROS2 is only used when
`USE_ROS2_SENSOR_BRIDGE=1` and the Python ROS2 packages are available.

## Survey Run

For the qualifier requirement that the drone conducts a full map exploration,
prefer the deterministic grid survey first:

```bash
cd ~/roboverse_qualifier/rrt_assisted_mission
FULL_MAP_TIME_LIMIT_S=585 \
FULL_MAP_GRID_SPACING_M=8 \
python3 full_map_grid_survey.py
```

This treats the 40 m x 40 m world as a camera-coverage grid and scans a small
panorama at each view point. It is more conservative around narrow obstacle
zones than the frontier/NBV survey.

The exploratory NBV survey is still available:

```bash
cd ~/roboverse_qualifier/rrt_assisted_mission
SURVEY_TIME_LIMIT_S=585 \
NBV_FRONTIER_GOALS_PER_PASS=10 \
OPEN_CRUISE_STEP_M=0.62 \
python3 survey_world_mission.py
```

The output files are written to `survey_outputs/`:

- `waypoints_<timestamp>.json`
- `waypoints_<timestamp>.csv`
- `coverage_route_<timestamp>.csv`
- `visited_cells_<timestamp>.csv`

The JSON/CSV waypoint rows include:

- `status`: `confirmed` or `candidate`
- `target_n`, `target_e`: estimated barrel location in local NED metres
- `visit_n`, `visit_e`: stand-off waypoint to approach before facing the barrel
- `yaw_deg`: yaw from the visit point toward the target
- `alt_d`: suggested scan altitude
- `priority`: higher means visit earlier in a scoring run

## Second Run From Survey Waypoints

```bash
cd ~/roboverse_qualifier/rrt_assisted_mission
python3 score_from_survey_waypoints.py
```

By default this loads the latest `survey_outputs/waypoints_*.json` and visits
up to 12 survey waypoints. To select a specific survey file:

```bash
SURVEY_WAYPOINT_FILE=survey_outputs/waypoints_YYYYMMDD_HHMMSS.json \
python3 score_from_survey_waypoints.py
```

If barrel positions are randomized between runs, you can also make the second
run fast-follow the sampled survey route after the target waypoints:

```bash
SURVEY_SCORE_FOLLOW_ROUTE=1 \
SURVEY_SCORE_MAX_ROUTE_POINTS=30 \
python3 score_from_survey_waypoints.py
```

The original frontier fallback can still be enabled, but it is more aggressive
near obstacles:

```bash
SURVEY_SCORE_FOLLOW_WITH_FRONTIER=1 python3 score_from_survey_waypoints.py
```

## Useful Survey Tunables

```bash
SURVEY_TIME_LIMIT_S=900              # longer mapping run
SURVEY_NBV_CYCLES=5                 # more next-best-view passes
SURVEY_FRONTIER_STRIDES=12          # more sector strides per frontier pass
SURVEY_INVESTIGATE_CANDIDATES=1     # spend time actively confirming candidates
SURVEY_REVISIT_CANDIDATES=1         # revisit remembered blobs at the end
SOFT_RANGE_LIMIT_M=21.5             # allow deeper exploration before recentering
HARD_RANGE_LIMIT_M=24.0             # absolute range guard
```
