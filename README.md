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
| `_radius` | `0.03` (m) | Arc radius |
| `_arc_degrees` | `60.0` | Arc sweep angle |
| `_number_of_points` | `8` | Number of waypoints along the arc |
| `_wait_between_points` | `2.0` (s) | Pause at each waypoint |

Example with custom arc:

```bash
rosrun panda_moveit_ctrl panda_semicircle_motion.py _execute:=true _radius:=0.05 _arc_degrees:=90 _number_of_points:=12
```
