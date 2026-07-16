# SynthVLA — Panda Arc Motion for Data Collection

Controlled, repeatable end-effector arc motion on a **Franka Emika Panda**, built on ROS + MoveIt,
used to photograph an object from known viewpoints at a constant distance (phone mounted on the
end-effector).

Goal, script internals, and remaining work: see [implementation_plan.md](implementation_plan.md).

**Workspace path (on the robot PC):** `/home/leon/shared_ws/SynthVLA`

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
- `_execute:=false` or omitted — dry run, no motion

## Tunable parameters (Terminal 3)

All are private ROS params (`_name:=value`):

| Param | Default | Meaning |
|---|---|---|
| `_joint_file` | `joint_start.csv` next to the script | CSV with the 7 start joint values |
| `_execute` | `false` | `true` = move the real robot |
| `_radius` | `0.04` (m) | Arc radius = camera-object distance (target range 3–5 cm) |
| `_arc_degrees` | `60.0` | Arc sweep angle |
| `_number_of_points` | `9` | Number of waypoints (photo poses) along the arc |
| `_wait_between_points` | `2.0` (s) | Pause at each waypoint |
| `_track_object` | `false` | `false` = hand/phone orientation stays **fixed relative to the world** (identical to the initial pose at every waypoint). `true` = rotate the camera along the arc so it always aims at the object |
| `_object_radius` | `0.5 × _radius` (m) | Radius of the keep-out collision sphere around the object. **A value in meters** (the 0.5 is a ratio, only used for the default). `0` disables it; must be smaller than `_radius` |
| `_keepout_ignored_links` | hand + finger links incl. `*_sc` capsule variants | Links allowed to touch the keep-out sphere (the hand carries the camera and must get that close); the rest of the arm is still kept out. The script also auto-detects and exempts (with a warning) any link still in contact with the sphere at the start pose |
| `_rod_radius` | `0.005` (m) | Radius of the collision cylinder for the object's **support rod** (real rod is 2 mm; default includes a 3 mm margin). Vertical, from the ground to the object; **no link is exempt from it**. `0` disables |
| `_object_height` | `0.55` (m) | Height of the object above the **ground** = length of the support rod (the rod hangs that far below the object in the planning scene, always at full height) |
| `_camera_offset` | `0.10` (m) | Flange → phone-lens distance along the pointing axis. The object is placed `camera_offset + radius` below the flange, so the **lens** keeps `radius` to the object and the hand (~10.5 cm long) clears the object and rod. **Measure and set once the phone is mounted** |
| `_output_file` | `arc_poses_<date>_<time>.csv` **next to the script** (`src/panda_moveit_ctrl/scripts/`) | CSV file recording, for the start pose and every waypoint reached: the 7 joint values, end-effector position, and orientation quaternion. Written incrementally, so an aborted run keeps everything up to the failure. A relative path given explicitly is resolved against the terminal's current directory |

Example with custom arc:

```bash
rosrun panda_moveit_ctrl panda_semicircle_motion.py _execute:=true _radius:=0.05 _arc_degrees:=90 _number_of_points:=12
```
