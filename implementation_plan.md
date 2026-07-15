# Implementation Plan — Panda Arc Motion for Object Photography

How to run everything: see [README.md](README.md).

## Goal

Record an **image dataset of a physical object photographed from known viewpoints at a constant
distance**:

- The object sits at a fixed spot — the **center of an imaginary sphere** of manually chosen
  radius `r`.
- A **phone is mounted on the end-effector** and acts as the camera.
- The robot moves the camera along a **vertical arc in the Y-Z plane**, centered on the object.
  As the camera rises in Z, it shifts along Y so the camera-object distance always stays
  exactly `r`.
- **During development the arc is 60°** (matches the current script and keeps wrist rotation /
  reachability easy). The eventual target is a full **180°** sweep — from the object's height on
  one side, over the top, down to the object's height on the far side.
- The arc has **9 photo poses**, evenly spaced. The robot stops at each one; a photo is taken
  while stationary.
- The **camera must aim at the object at every pose** (end-effector orientation rotates along
  the arc so the object stays centered in frame).
- The sphere center is defined from the **start pose + the given radius**: the object is placed
  at distance `r` in +Y from the starting camera position, at the same height.
- **Safety constraint:** no part of the arm may ever enter the object's sphere (add it as a
  keep-out collision object in MoveIt).

## Current implementation status

The existing script already does the core mechanics — start from a CSV joint configuration,
step through arc waypoints in the Y-Z plane at 5 % speed, pause at each waypoint, log exact
poses. What still differs from the goal:

| | Current script | Goal |
|---|---|---|
| Arc sweep | 60° | 60° for development, 180° eventually |
| Waypoints | 8 | 9 (`_number_of_points:=9`, no code change) |
| Camera orientation | frozen | rotates to always aim at the object |
| Object sphere keep-out | none | sphere collision object in MoveIt |

Remaining code changes (both in `panda_semicircle_motion.py`):

1. **Camera orientation tracking** — compute, for each waypoint, the orientation that points
   the phone camera at the sphere center, instead of copying the start orientation.
2. **Sphere keep-out** — add the object's sphere as a MoveIt collision object
   (`PlanningSceneInterface`) so no plan takes any part of the arm through it.

## How the current script works (`panda_semicircle_motion.py`)

1. Reads a **7-joint start configuration** from a CSV file (`joint_start.csv`).
2. Moves the arm to that joint configuration and waits 2 s.
3. Reads the Cartesian pose actually reached — this becomes the arc origin.
4. Generates **8 waypoints** along a **60° arc of radius 3 cm** in the **Y-Z plane**:
   - `delta_y = r * (1 - cos θ)`, `delta_z = r * sin θ`
   - motion starts mostly **upward (+Z)** and curves toward **+Y (right)**
   - the arc's center is at distance `r` in +Y from the start pose, at the same height —
     that center is where the object goes
   - **X and end-effector orientation stay constant**
5. Moves to each waypoint one by one (each waypoint is planned and executed separately),
   **waiting 2 s between waypoints**, and logs the exact pose reached at every stop.

Safety defaults: velocity and acceleration scaled to **5 %**, and **nothing moves unless
`_execute:=true`** — without it the script only loads/validates everything and exits
(safety / dry-run mode).

## Files

| File | Role |
|---|---|
| `panda_semicircle_motion.py` | Main script: start pose from CSV → 8-waypoint arc, pausing at each point |
| `panda_moveit_ctrl_server_node.py` | Starts `PandaRobotService` — ROS service server for robot/gripper control (vel/acc 0.4, `panda_hand` end-effector, gripper loaded) |
| `panda_moveit_ctrl_node.py` | Standalone test node: opens and closes the gripper (not part of the main 3-terminal flow) |
| `joint_start.csv` | Single line, 7 comma-separated joint values (rad) — the start configuration |

## Notes / caveats

- Despite the name "semicircle", the default trajectory is a **60° arc**, not 180°.
- Each waypoint is reached with an independent MoveIt plan (`set_pose_target` + `go`), so the
  path **between** waypoints is not guaranteed to follow the arc exactly — only the waypoints
  themselves are on the arc.
- If any waypoint fails to plan/execute, the trajectory stops there with an error.
- The dry-run mode (`_execute:=false`) exits **before planning**, so it does not catch
  unreachable waypoints — do a real run at 5 % speed with the e-stop ready to verify
  reachability.
