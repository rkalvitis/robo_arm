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

All development-phase features are **implemented** (2026-07-15), pending test on the robot:

| | Status |
|---|---|
| Arc sweep | 60° (development value; 180° eventually) |
| Waypoints | default 9 |
| Camera orientation | rotates by −θ about X at each waypoint so the camera always aims at the object (`_track_object`, default `true`); math verified numerically — constant distance, ~0 aim error |
| Object sphere keep-out | sphere collision object added to the MoveIt planning scene at the computed center (`_object_radius`, default half the arc radius, `0` disables; must be < arc radius). Hand links (`_keepout_ignored_links`, incl. the coarse `*_sc` capsule bodies of newer panda_moveit_config) are exempted in the Allowed Collision Matrix — without this the hand, which must get within `radius` of the object, is permanently "in collision" and **all planning fails** (confirmed in demo-mode tests 2026-07-15: Cartesian 0 % feasible + OMPL TIMED_OUT at waypoint 1; exempting only the visible hand links was not enough — the `panda_hand_sc` capsule was the remaining contact). After adding the sphere the script now calls `/check_state_validity`, logs the exact contact pairs, and auto-exempts any link still touching the sphere |

### Motion-reliability fix (robot stopped after waypoint 1)

Observed on the robot: arm reaches the initial pose, makes one small move, then stops.
Likely causes: with `_radius:=0.03` consecutive waypoints are only ~4 mm apart at 5 % speed
(nearly invisible motion), and tiny `set_pose_target` + `go()` moves are exactly where
OMPL planning / Franka controller goal tolerances fail. Fix in `move_to_pose`:

1. Each segment is planned as a **straight-line Cartesian path** (`compute_cartesian_path`),
   then **re-timed to 5 % speed** (`retime_trajectory` — Cartesian plans ignore the move
   group's speed scaling, so without re-timing they'd run at full speed).
2. If the Cartesian plan covers < 99.9 % of the segment, **fall back** to the original
   `set_pose_target` + `go()`.
3. If the controller reports failure but the end-effector is actually **within 5 mm of the
   target**, log a warning and continue instead of aborting (Franka's controller often
   reports "goal tolerance violated" even though the arm arrived).

## How the current script works (`panda_semicircle_motion.py`)

1. Reads a **7-joint start configuration** from a CSV file (`joint_start.csv`).
2. Moves the arm to that joint configuration and waits 2 s.
3. Reads the Cartesian pose actually reached — this becomes the arc origin, and the
   **object (sphere center) is placed at distance `r` in +Y** from it, at the same height.
4. Adds the **keep-out sphere** at the center (unless `_object_radius:=0`).
5. Generates **9 waypoints** along a **60° arc of radius `r`** in the **Y-Z plane**:
   - `delta_y = r * (1 - cos θ)`, `delta_z = r * sin θ` — constant distance `r` from the object
   - motion starts mostly **upward (+Z)** and curves toward **+Y (right)**
   - **X stays constant**; orientation rotates by −θ about X so the camera keeps aiming at
     the object (frozen instead if `_track_object:=false`)
6. Moves to each waypoint one by one (straight-line Cartesian segment, re-timed to 5 % speed,
   with pose-target fallback), **waiting 2 s between waypoints**, and logs the exact pose
   reached at every stop.

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
- Segments between waypoints are straight lines (chords of the arc), not the arc itself —
  for adjacent waypoints a few mm apart the deviation is micrometers.
- If any waypoint fails to plan/execute (and the arm is not within 5 mm of it), the
  trajectory stops there with an error.
- The dry-run mode (`_execute:=false`) exits **before planning**, so it does not catch
  unreachable waypoints — do a real run at 5 % speed with the e-stop ready to verify
  reachability.
- The camera-object distance will be **3–5 cm; default/test value is 4 cm** (`_radius:=0.04`).
  At that scale the **whole motion spans only ~4 cm** (waypoints ~5 mm apart) — at 5 % speed
  it is easy to mistake for "not moving". Check the terminal for "Waypoint N/9 reached" vs
  "Trajectory stopped at waypoint N".
- The pose being controlled is the MoveIt end-effector frame (flange/hand), **not the phone
  lens** — at a 4 cm radius the offset between the two matters, and the physical hand/phone
  may not even fit that close to the object.
