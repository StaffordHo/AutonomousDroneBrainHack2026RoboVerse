# RRT-Assisted RoboVerse Mission

This folder is an experimental copy of the current mission stack with a local
RRT assist layer added.

The normal frontier planner is still the primary navigator. RRT only activates
after repeated failed moves, then proposes a short local heading through the
remembered blocked-cell map and the live depth clearance gate.

## Run

```bash
cd ~/roboverse_qualifier/rrt_assisted_mission
python3 competition_mission.py
```

With manual teleop override:

```bash
cd ~/roboverse_qualifier/rrt_assisted_mission
TELEOP_ENABLED=1 python3 competition_mission.py
```

## Useful Knobs

```bash
RRT_ASSIST_ENABLED=0 python3 competition_mission.py
RRT_ASSIST_MAX_ITERATIONS=220 python3 competition_mission.py
RRT_ASSIST_MAX_RANGE_M=9.0 python3 competition_mission.py
RRT_ASSIST_MIN_FAILED_STEPS=1 python3 competition_mission.py
UNCHARTED_NARROW_BONUS=30 python3 competition_mission.py
UNCHARTED_NEAR_HOME_OUTWARD_BONUS=10 python3 competition_mission.py
NARROW_UNCHARTED_STEP_M=0.16 python3 competition_mission.py
OPEN_CRUISE_STEP_M=0.65 python3 competition_mission.py
NBV_FRONTIER_GOALS_PER_PASS=8 python3 competition_mission.py
NBV_FRONTIER_RINGS_M=7,11,15,18 python3 competition_mission.py
```

Suggested first test:

```bash
RRT_ASSIST_MAX_ITERATIONS=200 RRT_ASSIST_MAX_RANGE_M=8.5 UNCHARTED_NARROW_BONUS=28 OPEN_CRUISE_STEP_M=0.62 NBV_FRONTIER_GOALS_PER_PASS=8 python3 competition_mission.py
```

The mission expects YOLO weights under `Codes/`, for example
`Codes/yolov8s_roboverse.pt` or `Codes/yolov8n_roboverse.pt`. These weights are
kept locally and ignored by Git; copy, symlink, or download them into place
before running a fresh clone.

## What This Variant Tries

- Stronger reward for projected cells that have never been visited.
- Extra bias toward narrow but passable openings when those openings lead to new cells.
- A frontier/next-best-view layer that sends the drone to high-information ring-sector viewpoints before falling back to greedy local frontier movement.
- Smaller `0.18m` probing steps when clearance is tight, instead of pushing a full normal step.
- Larger open-space cruise steps when front, side, and lower clearances are comfortably open.
- RRT assist is skipped while attitude is unstable, so it does not add commands during a recovery spike.
