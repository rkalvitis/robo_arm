# Implementation Plan — Panda Arc Motion for Object Photography

How to run everything: see [README.md](README.md).

## Goal

Record an **image dataset of a physical object photographed from known viewpoints at a constant
distance**:

- The object sits at a fixed spot — the **center of an imaginary sphere** of manually chosen
  radius `r`.
- A **phone is mounted on the end-effector** and acts as the camera.
- The camera **starts directly above the object, looking straight down** at it, and moves
  along a **vertical arc in the Y-Z plane** centered on the object: it sweeps toward −Y by
  default (`_arc_direction`, the phone/lens side of the flange; `+1` restores the +Y sweep),
  descending along the sphere while the camera-object distance stays exactly `r`, and
  **continues past the side view (90°) to underneath the object** (~170°, looking up at it).
- Bottom views are the whole point of the wire-thin support rod: the object hangs on a 2 mm
  rod so the camera can photograph it from below. On the final waypoints the hand works
  close to the rod (at 170° the camera is ~`r·sin(10°)` ≈ 1.7 cm off the rod axis for
  r = 10 cm) — the rod collision cylinder makes MoveIt reject anything that would touch it.
- The arc has **9 photo poses**, evenly spaced. The robot stops at each one; a photo is taken
  while stationary.
- The **camera must aim at the object at every pose** (end-effector orientation rotates along
  the arc so the object stays centered in frame). Confirmed by testing on 2026-07-16: a fixed
  world orientation was tried and rejected — the far waypoints become physically unreachable
  (waypoint 8 needed 32 s of planning, waypoint 9 impossible) and the object leaves the frame.
  Tracking adds no roll, so the photos' rotation stays consistent. With tracking, the flange
  orbits at `radius + camera_offset`; the lens stays at `radius` from the object.
- The sphere center is defined from the **start pose + the given radius**: the object is placed
  at distance `r` **straight below** the starting camera position (the hand's start is the
  highest point of the whole trajectory).
- **Safety constraint:** no part of the arm may ever enter the object's sphere (add it as a
  keep-out collision object in MoveIt).
- **Support rod:** the object is held by a thin vertical rod (2 mm radius) reaching from the
  ground up to the object, which sits **55 cm above the ground** (`_object_height`). The arm
  must never touch the rod — modeled as a vertical collision cylinder hanging 55 cm below the
  object (`_rod_radius`, default 5 mm = rod + margin, no link exempt).
- The object and rod sit directly under the initial start pose, and the rod is always
  collision-checked at **full height** (an earlier auto-trim fallback was removed after the
  start joints were updated — if the rod blocks a waypoint, the fix is the start pose or the
  radius, not the model).
- **Lens transform (`_lens_xyz` / `_lens_axis`, flange frame; supersedes the scalar
  `_camera_offset` on 2026-07-17):** the frame MoveIt controls is the flange, but the photos
  are taken by the iPhone 15 Pro **ultra-wide lens**, which sits at
  `(-0.0225, -0.0239, 0.1170)` m in the `panda_link8` frame and looks along
  `(-0.70711, 0, 0.70711)` — the phone lies at 45° in the holder cradle. Both values come
  from `phone_mount_conf3.stl` (2026-07-22 configuration: original `Mount+phone.stl`
  lens-ring circle fits ±0.7 mm, carried over by the exact rigid transform of the phone
  part, re-verified by a fresh circle fit on the new mesh, 0.3 mm). The object is placed
  `radius` from the lens **along the camera axis**, and each waypoint is a rigid rotation of
  the whole start pose about the object center around the world X axis, so the lens (not the
  flange) keeps exactly `radius` and stays aimed with zero roll (verified numerically:
  distance error <1e-12 m, aim error <1e-5 deg over a 170° arc; the math reduces exactly to
  the previous formula when the lens lies on the flange z axis). `_lens_xyz:=''` restores
  the legacy scalar-offset behavior. **Consequence of the 45° cradle: the start joints must
  pitch the flange 45° so the camera starts looking straight down** — the script warns above
  5° error. The lens world position is now logged as three extra CSV columns.
- **Pose recording:** every run writes a CSV (`_output_file`, default
  `arc_poses_<timestamp>.csv`) with one row per stop — waypoint index, arc angle, the 7 joint
  values, end-effector position, and orientation quaternion — flushed after each stop, for
  matching photos to exact camera poses.

## Custom phone-holder hand (added 2026-07-17)

The Franka gripper is **physically replaced** by the 3D-printed phone holder
(`franka_phone_holder_merged_backface.stl`, modeled in millimeters). Integration is
script-level (plan A): `panda_semicircle_motion.py` loads the binary STL without pyassimp,
scales it to meters, and attaches it to `panda_link8` as an `AttachedCollisionObject`
(published on `/attached_collision_object`), so MoveIt collision-checks the true holder
geometry against the support rod and the arm. The attachment happens before any planning,
in dry-run mode too (for RViz verification), and the holder is auto-exempted from the
keep-out sphere like the old hand links.

Alignment, derived from the mesh's DIN ISO 9409-1-A50 mounting face and the Franka Hand
mesh/manual (drawing 5.3):

| Quantity | Value | Source |
|---|---|---|
| x/y offset | 0 | mesh Ø63 rim centered on origin |
| z offset | −8 mm | mounting face at z = +8 mm in mesh coords |
| yaw | −90° | mesh pin hole at +90° (mesh +Y); flange pin on +X of `panda_link8` (Franka Hand mesh: pin at +45° in hand frame, hand mounts at −45°) |

Operational consequences:
- `franka_control.launch` must run with **`load_gripper:=false`** (otherwise MoveIt plans
  around a phantom gripper overlapping the holder).
- `panda_moveit_ctrl_server_node.py` updated: `load_gripper=False`, `ee="panda_link8"`.
- `panda_moveit_ctrl_node.py` (gripper open/close test) is obsolete — there is no gripper.
- The default mesh (`phone_mount_conf3.stl`, holder + phone) is 123 k triangles; if
  planning gets slow, decimate a copy and point `_holder_mesh` at it.

`phone_mount_conf3.stl` (same coordinate frame; current since 2026-07-22, phone flipped
180° in the cradle vs `Mount+phone.stl` with the camera bump near the cradle center, then
slid 10.3 mm toward flange −Y vs conf2) adds the iPhone 15 Pro: it sits at **45° in the
cradle**, and the three camera lens rings were located by circle fitting on the original
mesh (spread ≤0.7 mm) and carried over by the exact rigid transform of the phone part
(residual <1e-5 mm; re-verified by a fresh circle fit, 0.3 mm). In the flange frame (mm):
ultra-wide (−22.5, −23.9, 117.0), main (−22.5, −43.1, 117.0), telephoto (−35.9, −33.6,
103.6); camera axis (−0.70711, 0, 0.70711) — unchanged. Ring identification:
bump seen from the back in portrait = left column top/bottom + right middle; on the 15 Pro
ultra-wide is bottom-left (verify once by covering lenses at 0.5×). Note the tabulated point
is the **lens-ring top surface**; the optical entrance pupil sits a couple of mm behind it —
at r = 3–5 cm consider calibrating `_radius` against a test photo.

The lens sits on the **−Y side** of `panda_link8`; since 2026-07-20 the arc therefore
descends toward **−Y** by default (`_arc_direction`, sign flips both the waypoint rotation
and the aim rotation about world X; `_arc_direction:=1` restores the original +Y sweep).
`phone mount edit.stl` (phone rotated 180° in the cradle → lens on the +Y side, same 45°
axis) was analyzed the same day but **not adopted** — the sweep side is flipped in software
instead.

## Current implementation status

All development-phase features are **implemented and verified in demo-mode simulation**
(2026-07-15: full run, 90° arc, r=0.05 m, 12 waypoints — all reached; logged positions match
the planned arc to <1 mm and the final orientation matches the expected 90° rotation exactly).
The lens-based arc math was verified numerically offline (2026-07-17). Not yet run on the
real robot.

**Next step before real-object runs — start pose:** `joint_start.csv` predates the phone
holder and aims the flange z axis down; with the 45° cradle the flange must be pitched 45°
so the **camera** looks down. Record a new start configuration (freedrive with the phone
aiming at the floor, then save the joints); the script warns if the camera axis is >5° off
vertical.

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
   **object (sphere center) is placed at distance `r` straight below it** (camera starts
   looking down at the object; the start is the highest point of the trajectory).
4. Adds the **support rod** collision cylinder (ground → object) and the **keep-out sphere**
   at the center (unless disabled via `_rod_radius:=0` / `_object_radius:=0`).
5. Generates **9 waypoints** along a **60° arc of radius `r`** in the **Y-Z plane**, then
   **always waits for Enter** before the first arc move (2026-07-20) — the moment to place
   the physical object at the logged sphere-center position:
   - `delta_y = direction · r · sin θ`, `delta_z = r * (cos θ − 1)` — constant distance `r`
     from the object
   - motion **descends from the top** of the sphere toward **−Y by default**
     (`_arc_direction`; `+1` = toward +Y); 90° = side view
   - **X stays constant**; orientation rotates by −θ about X so the camera keeps aiming at
     the object (frozen instead if `_track_object:=false`)
6. Moves to each waypoint one by one (straight-line Cartesian segment, re-timed to 5 % speed,
   with pose-target fallback) and logs the exact pose reached at every stop. Between
   waypoints: `_wait_between_points` seconds (default 2 s), or with
   `_confirm_each_pose:=true` an **Enter press per pose** (take the photo, confirm, the arm
   moves on).

Safety defaults: velocity and acceleration scaled to **5 %**, and **nothing moves unless
`_execute:=true`** — without it the script only loads/validates everything and exits
(safety / dry-run mode).

## Files

| File | Role |
|---|---|
| `panda_semicircle_motion.py` | Main script: start pose from CSV → 8-waypoint arc, pausing at each point |
| `panda_moveit_ctrl_server_node.py` | Starts `PandaRobotService` — ROS service server for robot control (vel/acc 0.4, `panda_link8` end-effector, no gripper) |
| `panda_moveit_ctrl_node.py` | Standalone gripper test node — obsolete since the phone holder replaced the gripper |
| `joint_start.csv` | Single line, 7 comma-separated joint values (rad) — the start configuration. **Needs re-recording for the 45° camera cradle** |
| `phone_mount_conf3.stl` | **Current** holder + iPhone 15 Pro assembly (mm; since 2026-07-22, phone flipped 180° in the cradle, bump near the cradle center, slid 10.3 mm toward flange −Y vs conf2) — attached to `panda_link8` as collision geometry (default `_holder_mesh`), and source of the lens/camera measurements. Must be copied to the robot PC next to the script |
| `phone_mount_conf2.stl` | Same as conf3 without the 10.3 mm slide (ultra-wide at link8 (−22.5, −13.6, 117.0) mm) — superseded the same day, **not used** |
| `Mount+phone.stl` | Previous assembly (bump toward the +Z end; ultra-wide at link8 (−22.7, −68.0, 116.0) mm) — **not used**; origin of the original lens-ring circle fits |
| `phone mount edit.stl` | Alternative assembly with the phone rotated 180° in the cradle (lens on the +Y flange side; ultra-wide at link8 (−53.9, 64.6, 85.6) mm) — analyzed 2026-07-20, **not used** |
| `franka_phone_holder_merged_backface.stl` | Holder-only mesh (mm, same frame/attach pose) — alternative `_holder_mesh` if the phone is not mounted |

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
