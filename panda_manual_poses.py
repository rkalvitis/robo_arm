#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Manually demonstrated photo poses for the Franka Panda.

Replaces the computed arc with poses YOU pick: every stop is a
joint configuration recorded by freedriving the arm, and every run
replays exactly those configurations - same arm posture, same
camera pose, every iteration. No lens model, no tracking math, no
roll decisions.

RECORD MODE (_mode:=record) - move_group running:

    rosrun panda_moveit_ctrl panda_manual_poses.py _mode:=record _execute:=true

  Freedrive to each photo pose (check the framing on the phone),
  press Enter to record it; 'undo' removes the last pose, 'done'
  saves the file (_poses_file, default manual_poses.csv next to
  this script; one row of 7 joint values per pose, first row =
  first photo stop). If the file already EXISTS its poses are NOT
  overwritten blindly: the script walks through them one by one,
  and with _execute:=true the arm DRIVES to each existing pose
  first so you decide with the real framing in view (after a
  freedrive it clears the Franka monitored stop automatically).
  Enter/'k' keeps a pose (the arm then drives to the next one),
  'r' replaces it with the current joints, 'i' inserts the
  current joints before it (the arm drives back to the existing
  pose afterwards), 'x' deletes it, 'done' keeps all the
  remaining ones - then new poses are appended at the end.
  Without _execute:=true the walkthrough is motionless.

RUN MODE (default) - the arm steps through the recorded poses:

    rosrun panda_moveit_ctrl panda_manual_poses.py _execute:=true

  1. attaches the phone holder as collision geometry;
  2. moves to the INITIAL pose from _joint_file (joint_start.csv,
     same file as the arc script) with a joint-space plan - every
     run starts AND ends there;
  3. determines the object position: _object_xyz if given,
     otherwise DIRECTLY UNDERNEATH the ultra-wide lens at the
     reached initial pose - _object_distance (default 3 cm)
     straight down in the world from the lens position, which
     comes from the phone-mount model (_lens_xyz). The
     obstacle scene (keep-out sphere, support rod, background
     screen) is built around it so the moves between poses are
     collision-checked. The demonstrated poses were physically
     shown, but the planner may still refuse one that sits within
     a safety margin - shrink the margin or drop that obstacle;
  4. ALWAYS waits for Enter at the initial pose (place/check the
     object at the logged position);
  5. visits poses 1..N: joint-space plan to the exact recorded
     joints, CSV log per pose (initial pose = row 0, same format
     as the arc script), photo pause between poses
     (_confirm_each_pose:=true = Enter, otherwise
     _wait_between_points seconds);
  6. ALWAYS returns: after the last pose (and its photo pause),
     or after an abort, the arm revisits the reached poses in
     reverse and finishes at the initial pose.
"""

from __future__ import print_function

import copy
import datetime
import math
import os
import sys

import numpy as np
import rospy
import moveit_commander

from geometry_msgs.msg import Point
from moveit_msgs.srv import GetStateValidity

import panda_semicircle_motion as arc

# Only present on the robot PC (franka_ros); record-mode motion
# uses it to clear the monitored-stop state the robot enters when
# the guiding button is released after freedriving.
try:
    from franka_msgs.msg import ErrorRecoveryActionGoal
except ImportError:
    ErrorRecoveryActionGoal = None


MODE_RECORD = "record"
MODE_RUN = "run"

POSES_FILE = "manual_poses.csv"


def load_poses(file_path):
    """
    Reads the poses CSV: one row per pose, 7 joint values each.
    """

    file_path = os.path.abspath(os.path.expanduser(file_path))

    if not os.path.isfile(file_path):
        raise IOError(
            "Poses file not found: {} - record it first with "
            "_mode:=record".format(file_path)
        )

    values = np.loadtxt(file_path, delimiter=",", ndmin=2)

    if values.shape[1] != 7:
        raise ValueError(
            "Every row must contain exactly 7 joint values, "
            "got {} columns.".format(values.shape[1])
        )

    if not np.all(np.isfinite(values)):
        raise ValueError(
            "The poses file contains NaN or infinite values."
        )

    return [row.tolist() for row in values]


def current_joints(move_group):
    """
    Reads the current 7 joint values, or None on a bad read.
    """

    joints = move_group.get_current_joint_values()

    if len(joints) != 7:
        rospy.logerr(
            "Read %d joint values instead of 7 - not recorded.",
            len(joints)
        )
        return None

    return list(joints)


def make_recovery_publisher():
    """
    Publisher for the Franka error-recovery action goal, or None
    when franka_msgs is unavailable (e.g. in simulation).
    """

    if ErrorRecoveryActionGoal is None:
        return None

    return rospy.Publisher(
        "/franka_control/error_recovery/goal",
        ErrorRecoveryActionGoal,
        queue_size=1
    )


def attempt_error_recovery(recovery_pub):
    """
    Best-effort clearing of the Franka monitored-stop state that
    freedriving (guiding button) leaves behind - without it, the
    first planned motion after a freedrive is rejected by the
    controller. Harmless in simulation.
    """

    if recovery_pub is None:
        return

    recovery_pub.publish(ErrorRecoveryActionGoal())
    rospy.sleep(1.0)


def move_to_recorded(move_group, joints, label, execute,
                     recovery_pub):
    """
    Drives the arm to a registered pose during record mode (after
    clearing a freedrive stop), or logs why it will not move.
    """

    if not execute:
        rospy.loginfo(
            "(motion disabled - pass _execute:=true so the arm "
            "drives to %s)",
            label
        )
        return True

    attempt_error_recovery(recovery_pub)

    if arc.move_to_joint_position(move_group, joints, label=label):
        return True

    rospy.logwarn(
        "Could not move to %s - you can still decide about it "
        "(keep/replace/insert/delete), just without the arm "
        "sitting there.",
        label
    )
    return False


def record_mode(move_group, poses_file, execute, recovery_pub):
    """
    Interactive recording. When the poses file already exists, its
    poses are NOT overwritten blindly: the script walks through
    them one by one - and (with _execute:=true) the arm DRIVES to
    each existing pose first, so the decision is made with the
    real framing in view: keep moves on to the next pose, replace
    and insert take the CURRENT (freedriven) joints, delete drops
    it. After the walkthrough, new poses are appended at the end.
    So inserting an intermediate pose (e.g. between 7 and 8):
    keep 1..7 (the arm steps through them), freedrive to the new
    pose, 'i' at pose 8, then 'done' twice.
    """

    poses_file = os.path.abspath(os.path.expanduser(poses_file))

    existing = []

    if os.path.isfile(poses_file):
        try:
            existing = load_poses(poses_file)
        except Exception as error:
            rospy.logwarn(
                "Could not read the existing %s (%s) - starting "
                "from scratch.",
                poses_file,
                str(error)
            )

    poses = []

    if existing:
        rospy.loginfo(
            "%d existing pose(s) found in %s. Going through them "
            "one by one - for each: Enter/'k' = KEEP it, "
            "'r' = REPLACE it with the current joints, "
            "'i' = INSERT the current joints BEFORE it, "
            "'x' = DELETE it, 'done' = keep this and all the "
            "remaining poses and jump to appending.",
            len(existing),
            poses_file
        )

        index = 0
        needs_move = True

        while index < len(existing):

            if needs_move:
                move_to_recorded(
                    move_group,
                    existing[index],
                    "existing pose {}".format(index + 1),
                    execute,
                    recovery_pub
                )
                needs_move = False

            try:
                answer = input(
                    ">>> Existing pose %d/%d (will be pose %d): "
                    "[k]eep / [r]eplace / [i]nsert-before / "
                    "[x] delete / done: "
                    % (index + 1, len(existing), len(poses) + 1)
                ).strip().lower()
            except EOFError:
                answer = "done"

            if answer in ("done", "q", "quit"):
                poses.extend(existing[index:])
                rospy.loginfo(
                    "Kept the remaining %d existing pose(s).",
                    len(existing) - index
                )
                break

            if answer in ("", "k", "keep"):
                poses.append(existing[index])
                index += 1
                needs_move = True
                continue

            if answer in ("x", "del", "delete"):
                rospy.loginfo(
                    "Deleted existing pose %d.", index + 1
                )
                index += 1
                needs_move = True
                continue

            if answer in ("r", "replace", "i", "insert"):
                joints = current_joints(move_group)

                if joints is None:
                    continue

                poses.append(joints)

                rospy.loginfo(
                    "%s pose %d with the current joints: %s",
                    "Replaced" if answer.startswith("r")
                    else "Inserted as",
                    len(poses),
                    ["{:.4f}".format(v) for v in joints]
                )

                if answer.startswith("r"):
                    index += 1
                # insert: stay at the same existing pose, it is
                # asked about again right after the inserted one
                # (the arm drives back to it first).
                needs_move = True
                continue

            rospy.loginfo("Unknown command: %s", answer)
    else:
        rospy.loginfo(
            "Record mode: put the robot in freedrive (guiding "
            "mode). Freedrive to each photo pose, check the "
            "framing on the phone, then press Enter here."
        )

    rospy.loginfo(
        "Appending: Enter = record the current joints as the next "
        "pose, 'undo' removes the last pose, 'done' saves and "
        "exits."
    )

    while True:
        try:
            answer = input(
                ">>> Pose %d: Enter = record, 'undo', 'done': "
                % (len(poses) + 1)
            ).strip().lower()
        except EOFError:
            rospy.logwarn("stdin closed - saving what was recorded.")
            break

        if answer in ("done", "d", "q", "quit"):
            break

        if answer in ("undo", "u"):
            if poses:
                dropped = poses.pop()
                rospy.loginfo(
                    "Removed pose %d: %s",
                    len(poses) + 1,
                    ["{:.4f}".format(v) for v in dropped]
                )
            else:
                rospy.loginfo("Nothing to undo.")
            continue

        joints = current_joints(move_group)

        if joints is None:
            continue

        poses.append(joints)

        rospy.loginfo(
            "Recorded pose %d: %s",
            len(poses),
            ["{:.4f}".format(v) for v in joints]
        )

    if not poses:
        rospy.logerr("No poses recorded - nothing written.")
        return 1

    with open(poses_file, "w") as handle:
        for joints in poses:
            handle.write(
                ",".join("{:.9f}".format(v) for v in joints) + "\n"
            )

    rospy.loginfo(
        "%d pose(s) written to %s",
        len(poses),
        poses_file
    )

    return 0


def build_obstacles(scene, move_group, robot, object_xyz,
                    object_radius, rod_radius, object_height,
                    screen_mesh, screen_scale, screen_height,
                    screen_yaw_deg, keepout_ignored_links):
    """
    Builds the obstacle scene around the given object position:
    keep-out sphere (hand links exempt), support rod, background
    screen. Returns False on a hard failure.
    """

    center = Point(object_xyz[0], object_xyz[1], object_xyz[2])

    if rod_radius > 0.0:
        arc.add_support_rod(
            scene, move_group, center, rod_radius, object_height
        )
    else:
        rospy.logwarn("Support rod disabled (_rod_radius <= 0).")

    if screen_mesh:
        try:
            arc.add_background_screen(
                scene, move_group, center, object_height,
                screen_mesh, screen_scale, screen_height,
                screen_yaw_deg
            )
        except Exception as error:
            rospy.logerr(
                "Unable to add the background screen: %s. Fix "
                "_screen_mesh or pass _screen_mesh:=\"''\" to "
                "skip it deliberately.",
                str(error)
            )
            return False
    else:
        rospy.logwarn("Background screen disabled (_screen_mesh:='').")

    if object_radius > 0.0:
        arc.add_object_keepout(
            scene, move_group, center, object_radius
        )

        try:
            arc.allow_keepout_collisions(keepout_ignored_links)
        except Exception as error:
            rospy.logerr(
                "Unable to update the collision matrix: %s. "
                "Planning may fail; _object_radius:=0 disables "
                "the sphere.",
                str(error)
            )
    else:
        rospy.logwarn("Keep-out sphere disabled (_object_radius <= 0).")

    try:
        if not arc.ensure_state_clear(robot):
            rospy.logerr(
                "The current state is considered in collision "
                "with the obstacle scene - planning will fail."
            )
    except Exception as error:
        rospy.logerr("Unable to verify state validity: %s", str(error))

    return True


def diagnose_target_state(robot, joints, label):
    """
    Checks whether a TARGET joint configuration is itself valid in
    the current planning scene and logs the colliding body pairs -
    so a refused pose reports WHAT it collides with instead of
    just failing. Also distinguishes the other failure mode: a
    valid pose that the planner merely could not find a path to.

    Returns True when the target is valid (pathfinding failure),
    False when it is in collision, None when the check failed.
    """

    try:
        rospy.wait_for_service("/check_state_validity", timeout=5.0)

        check_validity = rospy.ServiceProxy(
            "/check_state_validity",
            GetStateValidity
        )

        state = robot.get_current_state()
        names = list(state.joint_state.name)
        positions = list(state.joint_state.position)

        for index in range(7):
            joint_name = "panda_joint{}".format(index + 1)
            if joint_name in names:
                positions[names.index(joint_name)] = joints[index]

        state.joint_state.position = positions

        response = check_validity(
            robot_state=state,
            group_name=arc.PLANNING_GROUP
        )
    except Exception as error:
        rospy.logwarn(
            "Unable to collision-check the target state of %s: %s",
            label,
            str(error)
        )
        return None

    if response.valid:
        rospy.logwarn(
            "%s itself is VALID (no collision at the target) - "
            "the failure was pathfinding (e.g. TIMED_OUT), not an "
            "obstacle.",
            label
        )
        return True

    pairs = sorted({
        "{} <-> {}".format(
            contact.contact_body_1,
            contact.contact_body_2
        )
        for contact in response.contacts
    })

    rospy.logerr(
        "%s is IN COLLISION in the planning scene: %s. If this "
        "pose was physically demonstrated with the real obstacles "
        "in place, disable the virtual one that blocks it: "
        "_screen_mesh:=\"''\" / _rod_radius:=0 / _object_radius:=0.",
        label,
        "; ".join(pairs) if pairs else "no contact pair reported"
    )

    return False


def return_along_poses(move_group, reached_poses, init_joints):
    """
    Revisits the reached poses in reverse with joint-space plans
    and finishes at the initial pose. reached_poses = the poses
    successfully visited so far, in forward order (the arm sits at
    or near the last one; revisiting it first is a cheap no-op
    that also recovers from a partially-failed move).
    """

    for index in range(len(reached_poses) - 1, -1, -1):

        if rospy.is_shutdown():
            rospy.logwarn("ROS shut down. Stopping the return.")
            return False

        if not arc.move_to_joint_position(
                move_group,
                reached_poses[index],
                label="pose {} (return)".format(index + 1)):
            rospy.logerr(
                "Return stopped before pose %d - the arm stays "
                "where it is.",
                index + 1
            )
            return False

    if rospy.is_shutdown():
        return False

    return arc.move_to_joint_position(
        move_group,
        init_joints,
        label="the initial pose (return)"
    )


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("panda_manual_poses", anonymous=False)

    script_dir = os.path.dirname(os.path.abspath(__file__))

    mode = rospy.get_param("~mode", MODE_RUN).strip().lower()

    poses_file = rospy.get_param(
        "~poses_file",
        os.path.join(script_dir, POSES_FILE)
    )

    if mode not in (MODE_RECORD, MODE_RUN):
        rospy.logerr(
            "_mode must be '%s' or '%s', got: %s",
            MODE_RECORD, MODE_RUN, mode
        )

        moveit_commander.roscpp_shutdown()
        return 1

    try:
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        move_group = moveit_commander.MoveGroupCommander(
            arc.PLANNING_GROUP
        )
    except Exception as error:
        rospy.logerr(
            "Unable to initialize MoveGroupCommander: %s",
            str(error)
        )

        moveit_commander.roscpp_shutdown()
        return 1

    if mode == MODE_RECORD:
        # With _execute:=true the arm DRIVES to each existing pose
        # during the walkthrough; without it the walkthrough is
        # motionless (old behavior, safe default).
        record_execute = rospy.get_param("~execute", False)

        if record_execute:
            move_group.set_max_velocity_scaling_factor(
                arc.VELOCITY_SCALE
            )
            move_group.set_max_acceleration_scaling_factor(
                arc.ACCELERATION_SCALE
            )
            move_group.set_goal_joint_tolerance(
                arc.JOINT_TOLERANCE
            )
            move_group.set_planning_time(arc.PLANNING_TIME)
            move_group.set_num_planning_attempts(
                arc.PLANNING_ATTEMPTS
            )

            rospy.sleep(2.0)

            # The moves must respect the real holder geometry.
            record_holder = rospy.get_param(
                "~holder_mesh",
                os.path.join(script_dir, arc.HOLDER_MESH_FILE)
            )

            if record_holder:
                try:
                    arc.attach_phone_holder(
                        scene,
                        record_holder,
                        rospy.get_param("~holder_z_offset",
                                        arc.HOLDER_Z_OFFSET),
                        rospy.get_param("~holder_yaw_deg",
                                        arc.HOLDER_YAW_DEG)
                    )
                except Exception as error:
                    rospy.logerr(
                        "Unable to attach the phone holder: %s",
                        str(error)
                    )
                    moveit_commander.roscpp_shutdown()
                    return 1

            rospy.logwarn(
                "Record mode WITH motion: the arm will drive to "
                "each existing pose (no obstacle scene - the "
                "object position is only known at run time). Keep "
                "the workspace clear and the emergency stop at "
                "hand; motions run at %.0f%% speed.",
                arc.VELOCITY_SCALE * 100.0
            )

        result = record_mode(
            move_group,
            poses_file,
            record_execute,
            make_recovery_publisher() if record_execute else None
        )
        moveit_commander.roscpp_shutdown()
        return result

    # ------------------------- run mode -------------------------

    execute_motion = rospy.get_param("~execute", False)

    wait_between_points = rospy.get_param(
        "~wait_between_points", arc.WAIT_BETWEEN_POINTS
    )

    confirm_each_pose = rospy.get_param(
        "~confirm_each_pose", arc.CONFIRM_EACH_POSE
    )

    # Initial pose: every run starts AND ends here (same file the
    # arc script uses).
    joint_file = rospy.get_param(
        "~joint_file",
        os.path.join(script_dir, "joint_start.csv")
    )

    # World position of the object, 'x,y,z' in the planning frame.
    # '' (default) = directly underneath the ultra-wide lens at
    # the reached initial pose: _object_distance straight down in
    # the world from the lens position (from the mount model).
    object_xyz_text = rospy.get_param("~object_xyz", "")

    # Lens-object distance used when _object_xyz is not given.
    object_distance = rospy.get_param("~object_distance", 0.03)

    # Planning budget per move, and the escalated budget used for
    # ONE automatic retry when a pose fails although its target
    # state is collision-free (pure pathfinding failure): the
    # planner is randomized and anytime, so more time genuinely
    # helps with narrow passages.
    planning_time = rospy.get_param(
        "~planning_time", arc.PLANNING_TIME
    )
    retry_planning_time = rospy.get_param(
        "~retry_planning_time", 60.0
    )

    object_radius = rospy.get_param("~object_radius", 0.02)
    rod_radius = rospy.get_param("~rod_radius",
                                 arc.SUPPORT_ROD_RADIUS)
    object_height = rospy.get_param("~object_height", 0.58)

    screen_mesh = rospy.get_param(
        "~screen_mesh",
        os.path.join(script_dir, arc.SCREEN_MESH_FILE)
    )
    screen_scale = rospy.get_param("~screen_scale",
                                   arc.SCREEN_MESH_SCALE)
    screen_height = rospy.get_param("~screen_height",
                                    arc.SCREEN_HEIGHT_ABOVE_GROUND)
    # No computed arc here, so no direction to derive "auto" from:
    # 0 = wall behind the object toward +Y (the current setup).
    screen_yaw_deg = float(rospy.get_param("~screen_yaw_deg", 0.0))

    keepout_ignored_links = [
        link.strip()
        for link in rospy.get_param(
            "~keepout_ignored_links",
            arc.KEEPOUT_IGNORED_LINKS
        ).split(",")
        if link.strip()
    ]

    holder_mesh = rospy.get_param(
        "~holder_mesh",
        os.path.join(script_dir, arc.HOLDER_MESH_FILE)
    )
    holder_z_offset = rospy.get_param("~holder_z_offset",
                                      arc.HOLDER_Z_OFFSET)
    holder_yaw_deg = rospy.get_param("~holder_yaw_deg",
                                     arc.HOLDER_YAW_DEG)

    # Lens transform: used for the lens_x/y/z CSV columns and for
    # deriving the default object position from the initial pose.
    lens_xyz_text = rospy.get_param("~lens_xyz",
                                    arc.LENS_XYZ_LINK8)
    lens_axis_text = rospy.get_param("~lens_axis",
                                     arc.LENS_AXIS_LINK8)
    try:
        lens_xyz = np.array(
            arc.parse_vector3(lens_xyz_text, "_lens_xyz")
        )
        lens_axis = np.array(
            arc.parse_vector3(lens_axis_text, "_lens_axis")
        )
    except ValueError as error:
        rospy.logerr("Invalid lens parameter: %s", str(error))
        moveit_commander.roscpp_shutdown()
        return 1

    if object_distance <= 0.0:
        rospy.logerr(
            "_object_distance must be greater than zero, got %.4f",
            object_distance
        )
        moveit_commander.roscpp_shutdown()
        return 1

    object_xyz = None

    if object_xyz_text:
        try:
            object_xyz = arc.parse_vector3(
                object_xyz_text, "_object_xyz"
            )
        except ValueError as error:
            rospy.logerr("Invalid _object_xyz: %s", str(error))
            moveit_commander.roscpp_shutdown()
            return 1

    output_file = rospy.get_param("~output_file", "")

    if not output_file:
        output_file = os.path.join(
            script_dir,
            "manual_poses_{}.csv".format(
                datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            )
        )

    try:
        poses = load_poses(poses_file)
    except Exception as error:
        rospy.logerr("Error reading the poses file: %s", str(error))
        moveit_commander.roscpp_shutdown()
        return 1

    try:
        init_joints = arc.load_joint_position(joint_file)
    except Exception as error:
        rospy.logerr("Error reading the joint file: %s", str(error))
        moveit_commander.roscpp_shutdown()
        return 1

    total = len(poses)

    rospy.loginfo("Poses file: %s (%d poses)", poses_file, total)
    rospy.loginfo("Initial pose file: %s", joint_file)
    rospy.loginfo("Execution enabled: %s", execute_motion)
    rospy.loginfo(
        "Object position: %s",
        "({:.4f}, {:.4f}, {:.4f})".format(*object_xyz)
        if object_xyz else
        "directly underneath the ultra-wide lens at the initial "
        "pose, %.1f cm straight down" % (object_distance * 100.0)
    )
    rospy.loginfo(
        "The run always starts and ends at the initial pose."
    )
    rospy.loginfo("Recording poses to: %s", output_file)

    move_group.set_max_velocity_scaling_factor(arc.VELOCITY_SCALE)
    move_group.set_max_acceleration_scaling_factor(
        arc.ACCELERATION_SCALE
    )
    move_group.set_goal_joint_tolerance(arc.JOINT_TOLERANCE)
    move_group.set_planning_time(planning_time)
    move_group.set_num_planning_attempts(arc.PLANNING_ATTEMPTS)

    rospy.sleep(2.0)

    if holder_mesh:
        try:
            arc.attach_phone_holder(
                scene, holder_mesh, holder_z_offset, holder_yaw_deg
            )
        except Exception as error:
            rospy.logerr(
                "Unable to attach the phone holder: %s",
                str(error)
            )
            moveit_commander.roscpp_shutdown()
            return 1
    else:
        rospy.logwarn("Phone holder attachment disabled.")

    if not execute_motion:
        rospy.logwarn(
            "Safety mode active: the robot will not be moved. "
            "Add _execute:=true to run."
        )
        moveit_commander.roscpp_shutdown()
        return 0

    rospy.logwarn(
        "Check that the workspace is clear and the emergency stop "
        "is available."
    )

    # Remove stale obstacles left over from previous runs.
    scene.remove_world_object(arc.KEEPOUT_OBJECT_NAME)
    scene.remove_world_object(arc.ROD_OBJECT_NAME)
    scene.remove_world_object(arc.SCREEN_OBJECT_NAME)
    rospy.sleep(0.5)

    # Every run starts at the initial pose.
    if not arc.move_to_joint_position(
            move_group,
            init_joints,
            label="the initial pose (joint_start.csv)"):
        diagnose_target_state(
            robot, init_joints, "the initial pose"
        )
        moveit_commander.roscpp_shutdown()
        return 1

    rospy.sleep(arc.WAIT_AFTER_INITIAL_POSITION)

    arc.print_current_pose(move_group, "Initial pose reached")

    start_pose = copy.deepcopy(move_group.get_current_pose().pose)

    if object_xyz is None:
        # The object (and the rod under it) sits DIRECTLY
        # UNDERNEATH the ultra-wide lens: _object_distance
        # straight down in the world from the lens position at the
        # initial pose (the lens transform comes from the phone
        # mount model). This is a vertical drop, NOT along the
        # camera axis - the two differ when the camera is not
        # aimed straight down.
        lens_world, axis_world, _ = arc.compute_camera_geometry(
            start_pose, lens_xyz, lens_axis, object_distance
        )

        object_xyz = [
            lens_world[0],
            lens_world[1],
            lens_world[2] - object_distance
        ]

        rospy.loginfo(
            "Object position: directly underneath the ultra-wide "
            "lens at the initial pose - x=%.6f, y=%.6f, z=%.6f "
            "(%.1f cm straight below the lens at %.6f, %.6f, "
            "%.6f)",
            object_xyz[0], object_xyz[1], object_xyz[2],
            object_distance * 100.0,
            lens_world[0], lens_world[1], lens_world[2]
        )

        tilt_deg = math.degrees(
            math.acos(max(-1.0, min(1.0, -axis_world[2])))
        )

        if tilt_deg > 5.0:
            rospy.logwarn(
                "The camera axis is %.1f degrees away from "
                "straight down at the initial pose: the object "
                "sits below the lens but will be off-center in "
                "the frame by about that angle.",
                tilt_deg
            )

    if not build_obstacles(
            scene, move_group, robot, object_xyz,
            object_radius, rod_radius, object_height,
            screen_mesh, screen_scale, screen_height,
            screen_yaw_deg, keepout_ignored_links):
        moveit_commander.roscpp_shutdown()
        return 1

    try:
        log_handle, log_writer = arc.open_pose_log(output_file)
    except Exception as error:
        rospy.logerr("Unable to open the output file: %s", str(error))
        moveit_commander.roscpp_shutdown()
        return 1

    # The initial pose is recorded as row 0, like the arc script.
    arc.log_current_pose(
        log_handle, log_writer, move_group, 0, 0.0, lens_xyz
    )

    # Object placement/check moment, always confirmed.
    arc.wait_for_enter(
        "Robot at the initial pose, %d recorded poses ready. "
        "Place/check the object at the logged position and the "
        "framing on the phone, clear the workspace, then confirm "
        "to start." % total
    )

    reached_poses = []
    aborted = False

    for index, joints in enumerate(poses, start=1):

        if rospy.is_shutdown():
            rospy.logwarn("ROS shut down. Stopping.")
            aborted = True
            break

        moved = arc.move_to_joint_position(
            move_group,
            joints,
            label="pose {}/{}".format(index, total)
        )

        if not moved:
            verdict = diagnose_target_state(
                robot, joints, "pose {}".format(index)
            )

            if verdict and retry_planning_time > planning_time:
                # Target is valid - give the randomized planner a
                # much bigger budget once before giving up.
                rospy.logwarn(
                    "Retrying pose %d with a %.0f s planning "
                    "budget (was %.0f s)...",
                    index,
                    retry_planning_time,
                    planning_time
                )

                move_group.set_planning_time(retry_planning_time)

                moved = arc.move_to_joint_position(
                    move_group,
                    joints,
                    label="pose {}/{} (long retry)".format(
                        index, total
                    )
                )

                move_group.set_planning_time(planning_time)

        if not moved:
            rospy.logerr(
                "Stopped at pose %d/%d. Poses reached so far are "
                "logged in %s",
                index, total, output_file
            )

            aborted = True
            break

        reached_poses.append(joints)

        arc.print_current_pose(
            move_group, "Pose {} reached".format(index)
        )

        arc.log_current_pose(
            log_handle, log_writer, move_group, index, 0.0,
            lens_xyz
        )

        if index < total:
            if confirm_each_pose:
                arc.wait_for_enter(
                    "Pose %d/%d done - take the photo, then "
                    "confirm to move on." % (index, total)
                )
            else:
                rospy.loginfo(
                    "Waiting %.2f s before the next pose...",
                    wait_between_points
                )
                rospy.sleep(wait_between_points)

    log_handle.close()

    if not aborted:
        rospy.loginfo("All %d poses have been visited.", total)

    rospy.loginfo("Joint values and poses saved to: %s", output_file)

    # The run ALWAYS ends at the initial pose (also after an
    # abort, over the poses reached so far).
    if not rospy.is_shutdown():

        if not aborted and reached_poses:
            if confirm_each_pose:
                arc.wait_for_enter(
                    "Last pose done - take the final photo, then "
                    "confirm to return to the initial pose."
                )
            else:
                rospy.sleep(wait_between_points)

        rospy.loginfo(
            "Returning to the initial pose through the reached "
            "poses in reverse..."
        )

        if return_along_poses(move_group, reached_poses,
                              init_joints):
            rospy.loginfo("Back at the initial pose.")
        else:
            rospy.logerr("Return incomplete.")

    move_group.stop()
    move_group.clear_pose_targets()
    moveit_commander.roscpp_shutdown()

    return 1 if aborted else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
