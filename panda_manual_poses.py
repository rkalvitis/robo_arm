#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Manually demonstrated photo poses for the Franka Panda.

Replaces the computed arc with poses YOU pick: every stop is a
joint configuration recorded by freedriving the arm, and every run
replays exactly those configurations - same arm posture, same
camera pose, every iteration. No lens model, no tracking math, no
roll decisions.

RECORD MODE (_mode:=record) - robot in freedrive/guiding mode,
move_group running; this mode never moves the robot:

    rosrun panda_moveit_ctrl panda_manual_poses.py _mode:=record

  Freedrive to each photo pose (check the framing on the phone),
  press Enter to record it; 'undo' removes the last pose, 'done'
  saves the file (_poses_file, default manual_poses.csv next to
  this script; one row of 7 joint values per pose, first row =
  first photo stop).

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

import panda_semicircle_motion as arc


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


def record_mode(move_group, poses_file):
    """
    Interactive recording: freedrive, Enter records the current
    joints, 'undo' drops the last pose, 'done' writes the file.
    """

    poses_file = os.path.abspath(os.path.expanduser(poses_file))

    if os.path.isfile(poses_file):
        rospy.logwarn(
            "%s exists and will be OVERWRITTEN when you type "
            "'done'.",
            poses_file
        )

    rospy.loginfo(
        "Record mode: put the robot in freedrive (guiding mode). "
        "Freedrive to each photo pose, check the framing on the "
        "phone, then press Enter here. 'undo' removes the last "
        "recorded pose, 'done' saves and exits."
    )

    poses = []

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

        joints = move_group.get_current_joint_values()

        if len(joints) != 7:
            rospy.logerr(
                "Read %d joint values instead of 7 - not recorded.",
                len(joints)
            )
            continue

        poses.append(list(joints))

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
        result = record_mode(move_group, poses_file)
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
    move_group.set_planning_time(arc.PLANNING_TIME)
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

        if not arc.move_to_joint_position(
                move_group,
                joints,
                label="pose {}/{}".format(index, total)):
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
