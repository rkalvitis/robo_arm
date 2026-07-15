# SynthVLA — Panda Arc Motion for Data Collection

Controlled, repeatable end-effector arc motion on a **Franka Emika Panda**, built on ROS + MoveIt.
The robot moves to a known start configuration, then steps through a precise arc of waypoints in the
Y-Z plane, pausing at each one — so that data (e.g. camera frames + exact end-effector poses) can be
captured at each static waypoint.

**Workspace path (on the robot PC):** `/home/leon/shared_ws/SynthVLA`

## What the motion does (`panda_semicircle_motion.py`)

1. Reads a **7-joint start configuration** from a CSV file (`joint_start.csv`).
2. Moves the arm to that joint configuration and waits 2 s.
3. Reads the Cartesian pose actually reached — this becomes the arc origin.
4. Generates **8 waypoints** along a **60° arc of radius 3 cm** in the **Y-Z plane**:
   - `delta_y = r * (1 - cos θ)`, `delta_z = r * sin θ`
   - motion starts mostly **upward (+Z)** and curves toward **+Y (right)**
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

## Prerequisites

- Franka Panda reachable at `172.16.0.2` (for real execution); robot unlocked in Franka Desk, FCI enabled
- Workspace clear and emergency stop within reach when executing for real

## Run the process

Run these 3 commands **in sequence**, each in its **own terminal**.

In **every terminal**, first go to the workspace and source it:

```bash
cd /home/leon/shared_ws/SynthVLA
source devel/setup.bash
```

(Running from the workspace root also matters for Terminal 3: the `_joint_file` path
`src/panda_moveit_ctrl/scripts/joint_start.csv` is relative to the current directory.)

### Terminal 1 — Franka control + MoveIt (`move_group`)

```bash
roslaunch panda_moveit_config franka_control.launch robot_ip:=172.16.0.2
```

### Terminal 2 — Robot control service server

```bash
rosrun panda_moveit_ctrl panda_moveit_ctrl_server_node.py
```

### Terminal 3 — Arc motion

```bash
rosrun panda_moveit_ctrl panda_semicircle_motion.py _joint_file:=src/panda_moveit_ctrl/scripts/joint_start.csv _execute:=true
```

- `_execute:=true` — moves the **real robot**
- `_execute:=false` or omitted — dry run / simulation only, no motion

## Tunable parameters (Terminal 3)

All are private ROS params (`_name:=value`):

| Param | Default | Meaning |
|---|---|---|
| `_joint_file` | `joint_start.csv` next to the script | CSV with the 7 start joint values |
| `_execute` | `false` | `true` = move the real robot |
| `_radius` | `0.03` (m) | Arc radius |
| `_arc_degrees` | `60.0` | Arc sweep angle |
| `_number_of_points` | `8` | Number of waypoints along the arc |
| `_wait_between_points` | `2.0` (s) | Pause at each waypoint |

Example with custom arc:

```bash
rosrun panda_moveit_ctrl panda_semicircle_motion.py _execute:=true _radius:=0.05 _arc_degrees:=90 _number_of_points:=12
```

## Notes / caveats

- Despite the name "semicircle", the default trajectory is a **60° arc**, not 180°.
- Each waypoint is reached with an independent MoveIt plan (`set_pose_target` + `go`), so the
  path **between** waypoints is not guaranteed to follow the arc exactly — only the waypoints
  themselves are on the arc.
- If any waypoint fails to plan/execute, the trajectory stops there with an error.
