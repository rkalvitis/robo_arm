# Popillia 3DGS Dataset Collection Plan

Goal: capture a phone-camera image dataset of the popillia beetle for 3D Gaussian
Splatting reconstruction. The Franka Panda (this repo) is a **repeatable camera
stand**; **OptiTrack (`../optitrack`) is the pose backbone** — it tracks the
`phone` and `popillia` rigid bodies, and `match_photos.py` produces per-photo
camera poses **in the insect body frame** (which is what turns "insect rotates
under a fixed camera" into a virtual camera orbit). The robot's own pose CSV is
only a free cross-check. Training happens in `../3DGS` (Inria fork), which needs
a COLMAP-format scene.

## Locked decisions (2026-07-21)

| Decision | Value |
|---|---|
| Rotation | Insect rotated **by hand in 10° steps** using a degree map under the stand (36 steps/ring). OptiTrack measures the *actual* angle, so hand precision only affects coverage evenness, not pose accuracy. |
| Lens | **Ultra-wide** (the lens the robot script already tracks: `_lens_xyz = (-0.0227, -0.0680, 0.1160)` in link8 frame). |
| Hand-eye | Done **tomorrow, after** image collection — valid only if phone + markers are untouched in the cradle between capture and calibration. |
| Lighting | No tent — even room lighting, accepted as a known risk (see item 3). |
| Arc | Reuse the proven run: **160° / 8 stops, r = 0.04 m, −Y sweep, `_confirm_each_pose:=true`** (all 8 reached on 2026-07-20). |

## The data chain (what must connect to what)

```
photo (DNG, EXIF time) ──ms-clock offset──> mocap timeline (120 Hz)
mocap: phone body pose ──hand-eye (T_body→lens)──> lens pose
lens pose ──expressed in popillia body frame──> camera pose per photo (frames.csv)
frames.csv + intrinsics ──converter (NEW)──> COLMAP sparse/0 ──> 3DGS training
```

Everything up to `frames.csv` already exists in the optitrack repo
(`record-poses`, `ms_clock.py`, `match_photos.py`). Everything after it is a
post-capture gap (see bottom) — **none of it blocks tomorrow**, but two things
do change what you must do during capture: lock the focus (intrinsics validity)
and don't touch the phone (hand-eye validity).

## What is MISSING — must be handled tomorrow before/during capture

1. **Markers + rigid bodies for the new hardware state.** The `phone` (id 1269)
   and `popillia` (id 1268) bodies in `calibration/rigid_bodies.yaml` predate
   the printed cradle. Attach markers to the phone/cradle and the rotating part
   of the stand (markers MUST rotate with the insect), redefine both bodies in
   Motive with the **exact case-sensitive names `phone` and `popillia`**, and
   re-dump with `pixi run dump-bodies`.
2. **Popillia pivot on the pin tip.** In Motive, move the `popillia` body's
   pivot to the rod/pin tip where the insect sits (procedure: optitrack repo
   `RECORD_DATASET.md`, Phase 3). This makes the streamed origin the insect
   itself — the whole pipeline assumes it.
3. **Lighting check (no tent).** The insect rotates under fixed room lights,
   so any directional component gets baked in inconsistently: shadows and
   specular highlights move across the shell between rotations, which 3DGS
   sees as a contradiction (popillia's metallic elytra make speculars the
   bigger worry). Quick test before committing: shoot the beetle at 0°, 90°,
   180° rotation and compare — if shadows/highlights visibly migrate, add a
   cheap diffuser (white paper/cloth between light and object) or reposition.
   Also watch for the **arm and phone casting a shadow on the object** at some
   arc stops — the rig moves between rings, so its shadow is inconsistent too.
   Marker visibility is still worth a rehearsal: verify in Motive that **both
   bodies track at every arc stop, through a full 360° hand rotation, with the
   arm in place** — the low/under stops occlude the most.
4. **Remote shutter.** You cannot tap the phone — it's on the robot at 4 cm
   from the subject; touching it shakes the frame and stresses the mount. Use a
   Bluetooth shutter remote / volume-button on wired earphones / timer.
5. **Locked camera settings for the whole session.** In the camera app: select
   ultra-wide, RAW/ProRAW, then **lock focus, exposure and white balance** and
   don't change them until after the hand-eye/intrinsics session. Intrinsics
   are only valid at one focus setting — the post-capture ultra-wide intrinsics
   calibration must be shot at this same locked focus (board at ~the same 4 cm
   working distance).
6. **Markers out of frame.** At 4 cm the ultra-wide sees a lot; check a test
   photo for marker balls in frame. Popillia markers rotate with the insect so
   they're at least *consistent*, but they're clutter — keep them below/behind
   the framing if possible.
7. **Storage + battery.** ~9 elevations × 36 rotations ≈ **324 ProRAW photos ≈
   10–25 GB**. Check free space, start charged (or power the phone — cable
   strain-relieved so it doesn't tug the mount).
8. **Object placement.** Last run the beetle was ~1 cm off the computed center.
   Place it at the sphere-center coordinates the script logs at the
   place-object pause, then verify with a test photo (centered, in focus).
   With OptiTrack, placement error doesn't corrupt poses — only framing/focus.
9. **Radius sanity.** The entrance pupil sits a few mm behind the fitted lens
   ring, so treat `_radius:=0.04` as nominal: take one test photo at the start
   pose and nudge the object (or radius) until focus/framing is right.

## Step-by-step: tomorrow

### Phase 0 — Setup (~45 min)
1. Physical: stand with degree map, insect pinned, markers attached (item 1),
   phone in cradle on the Franka, remote shutter paired. Do the lighting
   rotation test (item 3) once the phone is roughly in position.
2. Motive: fresh camera calibration (wanding) if the cameras were moved;
   define/verify `phone` + `popillia` bodies; pivot to pin tip (item 2);
   `pixi run dump-bodies`.
3. Robot PC (3 terminals, `cd /home/leon/shared_ws/SynthVLA && source
   devel/setup.bash`):
   - T1: `roslaunch panda_moveit_config franka_control.launch robot_ip:=172.16.0.2 load_gripper:=false`
   - T2: `rosrun panda_moveit_ctrl panda_moveit_ctrl_server_node.py`
   - T3 (dry first): `rosrun panda_moveit_ctrl panda_semicircle_motion.py _joint_file:=src/panda_moveit_ctrl/scripts/joint_start.csv _execute:=false` — check RViz holder mesh vs reality, rod/sphere placement. Use the robot PC's `joint_start.csv` (the Mac copy is stale/malformed).
4. Phone: ultra-wide, RAW, lock focus/exposure/WB at the working distance
   (item 5). Confirm EXIF subsecond timestamps are present in a test DNG.
5. Tracking rehearsal: with the arm at the start pose and at the lowest stop
   (freedrive or dry positions), rotate the stand 360° and watch Motive — both
   bodies must stay tracked throughout (item 3).

### Phase 1 — Capture (~2.5–3 h)
1. Mac: `pixi run record-poses` (leave running for the ENTIRE session — do not
   restart between stops).
2. **Clock sync shot #1:** `pixi run ms-clock`, photograph the screen with the
   session phone, write down the displayed time (`HH:MM:SS.mmm`).
3. Robot: T3 with `_execute:=true _confirm_each_pose:=true _radius:=0.04
   _object_height:=0.57`. At the initial-pose pause, place the beetle at the
   logged sphere-center coords, take the test photo, adjust (items 8–9).
4. **Capture loop**, per arc stop (start pose = ring 0, then 8 stops):
   - for each of 36 degree-map positions: rotate the stand to the next 10°
     mark → hands off → wait ~2–3 s for rod wobble to settle → shoot with the
     remote → next.
   - bad shot? just retake — matching is by timestamp, not photo order, so
     extras/retakes are harmless.
   - ring done → Enter on the robot PC → arm moves to the next stop.
5. **Clock sync shot #2:** repeat the ms-clock photo (exposes clock drift).
6. Ctrl-C the recorder. Bag lands in `datasets/phone_<timestamp>/`.

### Phase 2 — Hand-eye + intrinsics (same day, ~45 min)
**Do not remove the phone from the cradle or touch any marker first.**
1. Intrinsics (ultra-wide, at the SAME locked focus): photograph the ChArUco
   board (the fine 1.5 mm board) at ~4 cm from many angles →
   `pixi run calibrate-intrinsics`. Target RMS well under the current 2.4 px.
2. Hand-eye: with `record-poses` running (own calibration session folder,
   `calibration/phone_<ts>/`), shoot the board from **many diverse
   orientations** — freedrive the Franka to get rotation diversity, hold still
   for each shot → `pixi run calibrate-handeye` (+ `calibrate-handeye-eth` as
   second opinion). The old `hand_eye.yaml` (8 photos, 146 px reprojection) is
   the cautionary tale: take 20+ well-spread shots.

### Phase 3 — Offload + verify (same evening)
1. AirDrop all DNGs into the session folder per the contract in the optitrack
   repo: `photos/*.dng`, clock shots in `clock/*.dng`.
2. Run `pixi run match -- ... --clock-shows <t1> --clock-shows <t2>` →
   `frames.csv` (poses in the popillia frame by default — exactly what 3DGS
   needs). Check: row count ≈ photo count, `gap_ms` ≈ 10, small
   `pos_spread_mm` / `rot_spread_deg`, drift warning absent.
3. Eyeball the camera constellation in `notebooks/camera_positions.ipynb` —
   you should see ~9 rings of ~36 cameras on a sphere around the origin.
4. Keep the robot's `arc_poses_*.csv` from the script directory as the
   cross-check artifact.

## What is MISSING — post-capture pipeline (doesn't block tomorrow)

In priority order; the 3DGS fork's `fineview_pipeline/` already contains most
of the reusable math:

1. **`--hand-eye` in `match_photos.py`** (optitrack Stage 2.5): compose
   `T_world_lens = T_world_body · T_body_lens` from the new `hand_eye.yaml`
   before building the relative track. Currently `frames.csv` is the *marker
   body* pose, not the lens.
2. **Converter `frames.csv` → COLMAP text scene** (new, small): invert
   cam→world to COLMAP world→cam (`t = −R·C`), mind the ROS/OpenCV axis
   convention (x-right y-down z-forward). Reuse from
   `3DGS/fineview_pipeline/`: `geometry.py` (rotmat2qvec, recentering,
   reprojection check) and the `cameras.txt`/`images.txt` writers in
   `export_colmap.py`. Only the ingest is new — `fineview_io.py` is
   rig-specific.
3. **Undistort ultra-wide images** with the new intrinsics and reduce to
   PINHOLE; recenter the principal point (the 3DGS loader ignores distortion
   and assumes centered pp).
4. **Masks (mandatory):** background doesn't rotate with the insect, so it is
   inconsistent in the object frame. Segment foreground (rembg/SAM), write
   RGBA PNGs with mask-as-alpha — the fork's alpha-mask loss path
   (`scene/cameras.py`, `train.py`) consumes this directly; pattern in
   `export_colmap.py:_process_and_save_image`.
5. **Init point cloud:** the COLMAP path crashes without `points3D`. Cheapest:
   synthetic random cloud in a small box around the origin →
   `seed_points.write_points3d` / `pcd_to_ply`. Alternative: COLMAP
   `point_triangulator` with poses fixed.
6. **Open question (parked):** focus stacking vs single focus
   (`thesis/optitrack/focus-problem.md`). Tomorrow = single locked focus; if
   DoF proves too thin, that's a follow-up capture, not a redo of the pipeline.

## Failure modes to watch

- Restarting `record-poses` mid-session → split bags, offset ambiguity. Don't.
- Changing focus/lens mid-session → intrinsics invalid for part of the photos.
- Touching phone/markers before the hand-eye session → hand-eye invalid for
  today's photos, unrecoverable.
- Marker occlusion at low arc stops → `gap_ms` spikes in `frames.csv`; rehearse
  in Phase 0.5 and re-shoot any ring that lost tracking.
- Forgetting a ms-clock shot → no photo↔mocap alignment. Take #1 before the
  first photo, #2 after the last.
