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
  2. if _object_xyz is given, builds the obstacle scene around it
     (keep-out sphere, support rod, background screen - same
     params as panda_semicircle_motion.py) so the moves BETWEEN
     poses are collision-checked against it. The demonstrated
     poses themselves were physically shown, but the planner may
     still refuse one that sits within a safety margin - shrink
     the margins or drop the obstacle in that case;
  3. moves to pose 1 with a joint-space plan, then ALWAYS waits
     for Enter (place/check the object);
  4. visits poses 2..N: joint-space plan to the exact recorded
     joints, CSV log per pose (same format as the arc script),
     photo pause between poses (_confirm_each_pose:=true = Enter,
     otherwise _wait_between_points seconds);
  5. with _return_to_start:=true (default) returns by revisiting
     the recorded poses in reverse - also after an abort, using
     the poses reached so far.
"""

from __future__ import print_function

import copy
import datetime
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


def return_along_poses(move_group, reached_poses):
    """
    Revisits the recorded poses in reverse (excluding the one the
    arm currently sits at) with joint-space plans, ending at
    pose 1. reached_poses = the poses successfully visited so far,
    in forward order.
    """

    for index in range(len(reached_poses) - 2, -1, -1):

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

    return True


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

    return_to_start = rospy.get_param(
        "~return_to_start", arc.RETURN_TO_START
    )

    # World position of the object, 'x,y,z' in the planning frame
    # (e.g. from the arc script's "Object (sphere center)" log or
    # from calibrate_lens.py's solved object). '' = no obstacle
    # scene: moves between poses are only checked against the
    # robot itself and the attached holder.
    object_xyz_text = rospy.get_param("~object_xyz", "")

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

    # Lens transform, used ONLY for the lens_x/y/z CSV columns.
    lens_xyz_text = rospy.get_param("~lens_xyz",
                                    arc.LENS_XYZ_LINK8)
    try:
        lens_xyz = np.array(
            arc.parse_vector3(lens_xyz_text, "_lens_xyz")
        )
    except ValueError as error:
        rospy.logerr("Invalid _lens_xyz: %s", str(error))
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

    total = len(poses)

    rospy.loginfo("Poses file: %s (%d poses)", poses_file, total)
    rospy.loginfo("Execution enabled: %s", execute_motion)
    rospy.loginfo(
        "Object position: %s",
        "({:.4f}, {:.4f}, {:.4f})".format(*object_xyz)
        if object_xyz else "not given - no obstacle scene"
    )
    rospy.loginfo("Return to pose 1 at the end: %s", return_to_start)
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

    # Remove stale obstacles from previous runs, then build the
    # scene BEFORE any motion so even the move to pose 1 is
    # collision-checked against it.
    scene.remove_world_object(arc.KEEPOUT_OBJECT_NAME)
    scene.remove_world_object(arc.ROD_OBJECT_NAME)
    scene.remove_world_object(arc.SCREEN_OBJECT_NAME)
    rospy.sleep(0.5)

    if object_xyz is not None:
        if not build_obstacles(
                scene, move_group, robot, object_xyz,
                object_radius, rod_radius, object_height,
                screen_mesh, screen_scale, screen_height,
                screen_yaw_deg, keepout_ignored_links):
            moveit_commander.roscpp_shutdown()
            return 1
    else:
        rospy.logwarn(
            "No _object_xyz given: moves between poses are only "
            "checked against the robot and the attached holder."
        )

    try:
        log_handle, log_writer = arc.open_pose_log(output_file)
    except Exception as error:
        rospy.logerr("Unable to open the output file: %s", str(error))
        moveit_commander.roscpp_shutdown()
        return 1

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

        if index == 1:
            # Object placement/check moment, always confirmed.
            arc.wait_for_enter(
                "Pose 1 reached (%d poses total). Place/check the "
                "object and the framing on the phone, clear the "
                "workspace, then confirm to continue." % total
            )
        elif index < total:
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

    if return_to_start and reached_poses and not rospy.is_shutdown():

        if not aborted:
            if confirm_each_pose:
                arc.wait_for_enter(
                    "Last pose done - take the final photo, then "
                    "confirm to return to pose 1."
                )
            else:
                rospy.sleep(wait_between_points)

        rospy.loginfo(
            "Returning to pose 1 through the recorded poses in "
            "reverse..."
        )

        if return_along_poses(move_group, reached_poses):
            rospy.loginfo("Back at pose 1.")
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
