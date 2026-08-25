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
roslaunch panda_moveit_config franka_control.launch robot_ip:=172.16.0.2 load_gripper:=false
```

`load_gripper:=false` is required: the Franka gripper has been physically replaced by the
custom phone holder. Without it MoveIt plans around a phantom gripper occupying the same
space as the holder (and `panda_semicircle_motion.py` attaches the holder mesh there).

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

## Testing in simulation (demo mode, no robot)

Same flow, but Terminal 1 runs the simulated robot instead of `franka_control.launch`:

```bash
roslaunch panda_moveit_config demo.launch load_gripper:=false
```

Then run Terminal 3 **with `_execute:=true`** — it is safe here because `demo.launch` only
drives a fake controller in RViz; the launch file decides sim vs real, not the `_execute`
flag. RViz shows the attached phone holder on the flange — check its orientation against the
physical holder. Note that `_execute:=false` is **not** a simulation: it validates the
parameters and exits before planning, so it cannot catch unreachable waypoints.

## Tunable parameters (Terminal 3)

All are private ROS params (`_name:=value`):

| Param | Default | Meaning |
|---|---|---|
| `_joint_file` | `joint_start.csv` next to the script | CSV with the 7 start joint values |
| `_execute` | `false` | `true` = move the real robot |
| `_radius` | `0.04` (m) | Arc radius = camera-object distance (target range 3–5 cm) |
| `_arc_degrees` | `60.0` | Arc sweep angle |
| `_arc_direction` | `-1.0` | Side of the world Y axis the arc descends toward: `-1` = sweep toward **−Y** (default since 2026-07-20 — the phone/lens side of the flange, unchanged with `phone_mount_conf3.stl`), `+1` = toward +Y (the original direction). The camera-aim rotation flips together with it; any value is normalized to its sign |
| `_number_of_points` | `9` | Number of waypoints (photo poses) along the arc |
| `_wait_between_points` | `2.0` (s) | Pause at each waypoint (used only with `_confirm_each_pose:=false`) |
| `_confirm_each_pose` | `false` | `true` = the script stops at **every waypoint** and waits for **Enter** before moving to the next pose (photo workflow); `false` = automatic `_wait_between_points`-second pauses. Independent of this flag, the script **always waits for Enter at the initial pose** — that's the moment to place the object at the logged sphere-center position before the arc starts |
| `_return_to_start` | `true` | After the **last waypoint** (and the final photo pause: Enter with `_confirm_each_pose:=true`, a `_wait_between_points` pause otherwise) the robot returns to the initial configuration by **retracing the executed stops in reverse** — the exact corridor just driven forward, keeping the photo pass's clearances to the object, rod, and screen. Also runs when the arc **aborts at an unfeasible waypoint**: the stops reached so far are retraced back to the start. (A free joint-space plan is not used: the keep-out sphere is ACM-exempt for the hand, so such a plan may legally sweep the phone through the object region — observed in sim 2026-08-25.) Per reverse segment: straight Cartesian line, falling back to a joint-space move to that stop's recorded joints. `false` = stay where the run ended |
| `_track_object` | `true` | `true` = the hand rotates along the arc so the camera **always faces the object**; the flange orbits at `radius + camera_offset` so the lens keeps `radius`. `false` = orientation frozen relative to the world (tested 2026-07-16: far waypoints become physically unreachable, object leaves the frame) |
| `_free_roll` | `true` | Only with `_track_object:=true`. `true` = the **image roll about the camera axis is free**: at every waypoint the script tries roll angles (nearest the previous waypoint's roll first, in `_roll_step_deg` steps up to ±`_roll_max_deg`) and moves to the first pose the arm can reach. The roll never moves the lens or the aim — the object stays centered in frame — and the CSV records the orientation actually reached, which is all reconstruction (3DGS/COLMAP) needs. `false` = the original fixed zero-roll poses (photo rotation stays consistent) |
| `_roll_step_deg` | `15.0` | Roll increment between candidates tried at each waypoint (with `_free_roll:=true`) |
| `_roll_max_deg` | `180.0` | Maximum roll magnitude tried, ± around zero; `180` = full freedom (with `_free_roll:=true`) |
| `_object_radius` | `0.5 × _radius` (m) | Radius of the keep-out collision sphere around the object. **A value in meters** (the 0.5 is a ratio, only used for the default). `0` disables it; must be smaller than `_radius` |
| `_keepout_ignored_links` | hand + finger links incl. `*_sc` capsule variants | Links allowed to touch the keep-out sphere (the hand carries the camera and must get that close); the rest of the arm is still kept out. The script also auto-detects and exempts (with a warning) any link still in contact with the sphere at the start pose |
| `_rod_radius` | `0.005` (m) | Radius of the collision cylinder for the object's **support rod** (real rod is 3 mm **diameter** = 1.5 mm radius; default includes a 3.5 mm margin). Vertical, from the ground to the object; **no link is exempt from it**. `0` disables. The default margin blocks waypoint 8 of the 160° arc (diagnosed in sim 2026-08-25) — `_rod_radius:=0.003` still leaves 1.5 mm clearance |
| `_object_height` | `0.58` (m) | Height of the object above the **ground** = length of the support rod (real setup measured 2026-08-25: 58 cm) (the rod hangs that far below the object in the planning scene, always at full height) |
| `_screen_mesh` | `background-white-screen.stl` next to the script | STL (binary, in **meters**) of the white **background screen**: a half-cylinder shell (wall radius 11–11.25 cm, 19.4 cm tall) with a 6 mm floor plate whose Ø4.4 mm hole sits at the arc center — the hole slides over the support rod, so the screen is placed at the object's x/y and the insect sits at the center of the semicircle. Added as a **world obstacle: no link may touch it** (unlike the keep-out sphere). **Copy the STL to the robot PC next to the script**, or the script exits with an error. `''` disables it |
| `_screen_height` | `0.425` (m) | Height of the screen's **bottom** above the ground (with the defaults the screen spans 42.5–61.9 cm, so the 58 cm insect sits inside it; the rim ends up above the starting lens height — the phone works inside the enclosure on upper waypoints, and MoveIt vetoes any pose where the holder would touch the wall) |
| `_screen_yaw_deg` | `auto` | Screen rotation about the world Z axis. `auto` = derived from `_arc_direction` so the **opening faces the camera side**: yaw `0` (wall bulges toward +Y, behind the insect) with the default −Y arc, `180` with `_arc_direction:=1`. A number forces that yaw |
| `_screen_scale` | `1.0` | Vertex scale for the screen STL (this one is modeled in meters, unlike the mm phone-mount STLs) |
| `_lens_xyz` | ultra-wide lens of the iPhone 15 Pro, measured from `phone_mount_conf3.stl`: `-0.0225,-0.0239,0.1170` (m) | Phone-lens position in the **flange frame** (`panda_link8`). The object is placed `radius` from the **lens along the camera axis**, and the arc keeps the lens (not the flange) at `radius`. `_lens_xyz:=''` falls back to the legacy scalar `_camera_offset` behavior |
| `_lens_axis` | `-0.70711,0.0,0.70711` | Direction the camera looks, unit vector in the flange frame (the phone sits at 45° in the cradle). With the default, the **start pose must pitch the flange 45°** so the camera looks straight down — the script warns if the camera axis is >5° off vertical |
| `_camera_offset` | `0.10` (m) | **Legacy**, only used with `_lens_xyz:=''`: lens assumed on the flange z axis, `camera_offset` below it |
| `_holder_mesh` | `phone_mount_conf3.stl` next to the script | STL (binary, in mm) of the phone holder **with the phone**, attached rigidly to `panda_link8` as collision geometry. **Copy the STL to the robot PC next to the script** (`src/panda_moveit_ctrl/scripts/`), or the script exits with an error. `''` disables the attachment |
| `_holder_z_offset` | `-0.008` (m) | Mesh z shift so its ISO 9409-1-A50 mounting face (at z = +8 mm in mesh coordinates) sits flush on the flange surface |
| `_holder_yaw_deg` | `-90.0` | Rotation of the mesh about the flange z axis. Derived from the dowel pin: the mesh's pin hole is on its +Y axis, the flange's pin is on +X of `panda_link8` |
| `_output_file` | `arc_poses_<date>_<time>.csv` **next to the script** (`src/panda_moveit_ctrl/scripts/`) | CSV file recording, for the start pose and every waypoint reached: the 7 joint values, end-effector position, orientation quaternion, and the world lens position. Written incrementally, so an aborted run keeps everything up to the failure. A relative path given explicitly is resolved against the terminal's current directory |

Example with custom arc:

```bash
rosrun panda_moveit_ctrl panda_semicircle_motion.py _execute:=true _radius:=0.05 _arc_degrees:=90 _number_of_points:=12
```

## Custom phone-holder hand

The Franka gripper is replaced by a 3D-printed phone holder
(`franka_phone_holder_merged_backface.stl`, millimeters). `panda_semicircle_motion.py`
attaches it to `panda_link8` as an `AttachedCollisionObject` so MoveIt plans around the real
geometry. Alignment was derived from the mesh's DIN ISO 9409-1-A50 mounting face:

- outer rim Ø63 (the flange diameter) centered on the mesh origin → mesh axis = flange axis
- mounting face at z = +8 mm → attached with a −8 mm z offset (flush on the flange)
- 4 bolt holes Ø5.5 on the Ø50 pitch circle at ±45°/±135°, dowel-pin hole Ø6 at +90° (mesh
  +Y). The pin on the Panda flange lies on **+X of `panda_link8`** (from the Franka Hand
  mesh: pin at +45° in the hand frame, hand mounted at −45° yaw) → the holder mounts at
  **−90° yaw**

**Verify before the first real run:** start a dry run (`_execute:=false` — the holder is
attached before the script exits) and compare the attached mesh in RViz with the physical
holder; the cradle side must point the same way.

### Phone and camera (`phone_mount_conf3.stl`)

The full assembly mesh shares the holder's coordinate frame. It is the current
configuration (2026-07-22): the phone is flipped 180° in the cradle vs the original
`Mount+phone.stl` (camera bump near the cradle center, like conf2) and additionally slid
10.3 mm toward the flange −Y. Measurements (phone part is an exact rigid transform of the
earlier mesh, matched to <1e-5 mm and re-verified by a fresh lens-ring circle fit, 0.3 mm
agreement):

- the iPhone 15 Pro (146.6 × 70.6 mm) sits at **45°** in the cradle; the camera looks along
  `(-0.70711, 0, 0.70711)` in the flange frame (unchanged)
- lens-ring centers (flange frame, mm): **ultra-wide (−22.5, −23.9, 117.0)** — bottom-left
  lens of the bump seen from the back in portrait; main (−22.5, −43.1, 117.0); telephoto
  (−35.9, −33.6, 103.6)
- the arc keeps the **ultra-wide lens** at `_radius` from the object and aimed at it
  (`_lens_xyz` / `_lens_axis`)
- the lens still sits on the **−Y side** of `panda_link8`, so the default −Y arc descent
  (`_arc_direction`) is unchanged
- previous configurations, all **not used**: `Mount+phone.stl` (bump toward the +Z end,
  ultra-wide at (−22.7, −68.0, 116.0)), `phone mount edit.stl` (lens on the +Y side), and
  `phone_mount_conf2.stl` (conf3 without the 10.3 mm slide, ultra-wide at
  (−22.5, −13.6, 117.0))

Because of the 45° cradle, the **start joint configuration must pitch the flange 45°** so
the camera starts looking straight down; the script warns if the camera axis is more than 5°
off vertical at the start pose. `joint_start.csv` from the gripper era does not satisfy this.
