#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Arc motion for Franka Emika Panda.

The script:

1. reads an initial joint configuration from a CSV file;
2. moves the robot to the initial configuration;
3. reads the Cartesian pose that was reached;
4. places the object (sphere center) at distance 'radius' straight
   BELOW that pose - the camera starts looking down at it;
5. generates waypoints along an arc in the Y-Z plane, centered on
   the object, descending from above toward the +Y side, so the
   camera-object distance stays constant;
6. rotates the end-effector at each waypoint so the camera keeps
   aiming at the object;
7. adds the object's sphere to the planning scene as a keep-out
   region so no part of the arm can plan through it;
8. waits 2 seconds between one waypoint and the next.

Example joint_start.csv:

0.013337267496607689,0.13310515648967364,0.12014143002274653,-1.703551451575958,0.007806931131451653,1.0049740870915318,0.9248341724704006
"""

from __future__ import print_function

import copy
import csv
import datetime
import math
import os
import sys

import numpy as np
import rospy
import moveit_commander

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    PlanningScene,
    PlanningSceneComponents
)
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from tf.transformations import quaternion_about_axis, quaternion_multiply


PLANNING_GROUP = "panda_arm"

RADIUS_METERS = 0.04
ARC_DEGREES = 60.0
NUMBER_OF_POINTS = 9

WAIT_BETWEEN_POINTS = 2.0
WAIT_AFTER_INITIAL_POSITION = 2.0

VELOCITY_SCALE = 0.05
ACCELERATION_SCALE = 0.05

JOINT_TOLERANCE = 0.002
POSITION_TOLERANCE = 0.002
ORIENTATION_TOLERANCE = 0.02

PLANNING_TIME = 10.0
PLANNING_ATTEMPTS = 10

# Cartesian segment planning between consecutive waypoints.
CARTESIAN_EEF_STEP = 0.002
CARTESIAN_JUMP_THRESHOLD = 5.0
CARTESIAN_MIN_FRACTION = 0.999

# If the controller reports failure but the end-effector is within
# this distance of the target, the waypoint is considered reached.
REACHED_DISTANCE_TOLERANCE = 0.005

# Default keep-out sphere radius as a fraction of the arc radius.
OBJECT_RADIUS_RATIO = 0.5

KEEPOUT_OBJECT_NAME = "object_keepout"

# The object sits on a thin vertical rod (2 mm radius) reaching up
# from the ground. Modeled with a safety margin; 0 disables it.
ROD_OBJECT_NAME = "support_rod"
SUPPORT_ROD_RADIUS = 0.005

POSE_LOG_HEADER = [
    "waypoint",
    "angle_deg",
    "joint1", "joint2", "joint3", "joint4",
    "joint5", "joint6", "joint7",
    "position_x", "position_y", "position_z",
    "orientation_x", "orientation_y", "orientation_z", "orientation_w"
]

# Links allowed to touch the keep-out sphere. The hand carries the
# camera and must get within 'radius' of the object, so it cannot be
# collision-checked against the sphere; the rest of the arm still is.
# The *_sc links are the coarse capsule collision bodies that newer
# panda_moveit_config versions add around the visible links.
KEEPOUT_IGNORED_LINKS = (
    "panda_link8,panda_hand,panda_leftfinger,panda_rightfinger,"
    "panda_hand_sc,panda_link7_sc,panda_link8_sc"
)


def load_joint_position(file_path):
    """
    Reads exactly 7 joint values from a CSV file.
    """

    file_path = os.path.abspath(os.path.expanduser(file_path))

    if not os.path.isfile(file_path):
        raise IOError(
            "Joint file not found: {}".format(file_path)
        )

    values = np.loadtxt(file_path, delimiter=",")
    values = np.asarray(values, dtype=float).reshape(-1)

    if values.size != 7:
        raise ValueError(
            "The file must contain exactly 7 joint values. "
            "Values found: {}".format(values.size)
        )

    if not np.all(np.isfinite(values)):
        raise ValueError(
            "The file contains invalid, NaN or infinite values."
        )

    return values.tolist()


def compute_sphere_center(start_pose, radius):
    """
    The object (sphere center) sits at distance 'radius' straight
    BELOW the starting camera position: the camera starts looking
    down at it, and the support rod continues below it to the
    ground.
    """

    center = copy.deepcopy(start_pose.position)
    center.z = start_pose.position.z - radius

    return center


def compute_aim_orientation(start_orientation, theta):
    """
    Rotates the start orientation by -theta about the world X axis.

    The waypoint positions rotate by -theta about the X axis around
    the sphere center, so applying the same rotation to the
    orientation keeps the camera aiming at the object.
    """

    q_start = [
        start_orientation.x,
        start_orientation.y,
        start_orientation.z,
        start_orientation.w
    ]

    q_rot = quaternion_about_axis(-theta, (1.0, 0.0, 0.0))

    return quaternion_multiply(q_rot, q_start)


def create_arc_waypoints(
        start_pose,
        radius,
        arc_degrees,
        number_of_points,
        track_object):
    """
    Generates an arc in the Y-Z plane, centered on the object,
    which sits at distance 'radius' straight below the start pose.

    +Y = right
    +Z = up

    The camera starts at the top of the sphere looking straight
    down at the object, then sweeps toward +Y, descending along the
    sphere. At theta = 90 degrees it looks at the object
    horizontally from the side; beyond that it goes below the
    object's height, toward the support rod and the ground.

    Formula (deltas from the start pose):

        delta_y = r * sin(theta)
        delta_z = r * (cos(theta) - 1)

    theta ranges from 0 to arc_degrees.

    If track_object is True, the end-effector orientation rotates
    along the arc so the camera keeps aiming at the sphere center;
    otherwise the orientation remains unchanged.
    """

    if radius <= 0.0:
        raise ValueError("The radius must be greater than zero.")

    if number_of_points <= 0:
        raise ValueError(
            "The number of waypoints must be greater than zero."
        )

    waypoints = []

    total_angle = math.radians(arc_degrees)

    rospy.loginfo(
        "Initial pose: x=%.6f, y=%.6f, z=%.6f",
        start_pose.position.x,
        start_pose.position.y,
        start_pose.position.z
    )

    for index in range(1, number_of_points + 1):

        theta = total_angle * float(index) / float(number_of_points)

        delta_y = radius * math.sin(theta)
        delta_z = radius * (math.cos(theta) - 1.0)

        waypoint = copy.deepcopy(start_pose)

        waypoint.position.x = start_pose.position.x
        waypoint.position.y = start_pose.position.y + delta_y
        waypoint.position.z = start_pose.position.z + delta_z

        if track_object:
            orientation = compute_aim_orientation(
                start_pose.orientation,
                theta
            )

            waypoint.orientation.x = orientation[0]
            waypoint.orientation.y = orientation[1]
            waypoint.orientation.z = orientation[2]
            waypoint.orientation.w = orientation[3]

        waypoints.append(waypoint)

        rospy.loginfo(
            "Waypoint %d/%d - angle %.2f deg - "
            "delta Y %.6f m - delta Z %.6f m - "
            "position Y %.6f - position Z %.6f",
            index,
            number_of_points,
            math.degrees(theta),
            delta_y,
            delta_z,
            waypoint.position.y,
            waypoint.position.z
        )

    return waypoints


def add_object_keepout(scene, move_group, center, object_radius):
    """
    Adds the object's sphere to the planning scene as a keep-out
    region so that no part of the arm can plan through it.
    """

    sphere_pose = PoseStamped()
    sphere_pose.header.frame_id = move_group.get_planning_frame()
    sphere_pose.pose.position.x = center.x
    sphere_pose.pose.position.y = center.y
    sphere_pose.pose.position.z = center.z
    sphere_pose.pose.orientation.w = 1.0

    scene.remove_world_object(KEEPOUT_OBJECT_NAME)
    rospy.sleep(0.5)

    scene.add_sphere(
        KEEPOUT_OBJECT_NAME,
        sphere_pose,
        object_radius
    )

    rospy.sleep(1.0)

    rospy.loginfo(
        "Keep-out sphere added: center x=%.6f, y=%.6f, z=%.6f - "
        "radius %.4f m",
        center.x,
        center.y,
        center.z,
        object_radius
    )


def add_support_rod(scene, move_group, center, rod_radius, rod_length):
    """
    Adds the object's support rod - a thin vertical cylinder that
    holds the object 'rod_length' above the ground - as a collision
    object. Unlike the keep-out sphere, NO link is exempt from it:
    nothing on the robot may ever touch the rod.
    """

    rod_pose = PoseStamped()
    rod_pose.header.frame_id = move_group.get_planning_frame()
    rod_pose.pose.position.x = center.x
    rod_pose.pose.position.y = center.y
    rod_pose.pose.position.z = center.z - rod_length / 2.0
    rod_pose.pose.orientation.w = 1.0

    scene.remove_world_object(ROD_OBJECT_NAME)
    rospy.sleep(0.5)

    try:
        scene.add_cylinder(
            ROD_OBJECT_NAME,
            rod_pose,
            rod_length,
            rod_radius
        )
    except AttributeError:
        # Older moveit_commander versions have no add_cylinder.
        scene.add_box(
            ROD_OBJECT_NAME,
            rod_pose,
            (2.0 * rod_radius, 2.0 * rod_radius, rod_length)
        )

    rospy.sleep(1.0)

    rospy.loginfo(
        "Support rod added: x=%.6f, y=%.6f, from z=%.6f (ground) "
        "up to z=%.6f - radius %.4f m (no link is exempt from it)",
        center.x,
        center.y,
        center.z - rod_length,
        center.z,
        rod_radius
    )


def open_pose_log(file_path):
    """
    Opens the CSV output file and writes the header. Rows are
    flushed after every waypoint, so an aborted run keeps all the
    poses recorded up to the failure.
    """

    file_path = os.path.abspath(os.path.expanduser(file_path))

    handle = open(file_path, "w")
    writer = csv.writer(handle)
    writer.writerow(POSE_LOG_HEADER)
    handle.flush()

    rospy.loginfo("Recording poses to: %s", file_path)

    return handle, writer


def log_current_pose(handle, writer, move_group, waypoint, angle_deg):
    """
    Appends the current joint values and end-effector pose to the
    output file.
    """

    joints = move_group.get_current_joint_values()
    pose = move_group.get_current_pose().pose

    writer.writerow(
        [waypoint, "{:.4f}".format(angle_deg)]
        + ["{:.9f}".format(value) for value in joints]
        + ["{:.9f}".format(value) for value in (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)]
    )

    handle.flush()


def allow_keepout_collisions(ignored_links):
    """
    Marks the given links as allowed to collide with the keep-out
    sphere in the Allowed Collision Matrix. Without this the hand,
    which must get within 'radius' of the object, puts the robot
    permanently in collision and every plan fails.
    """

    rospy.wait_for_service("/get_planning_scene", timeout=5.0)

    get_scene = rospy.ServiceProxy(
        "/get_planning_scene",
        GetPlanningScene
    )

    components = PlanningSceneComponents(
        components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    )

    acm = get_scene(components).scene.allowed_collision_matrix

    if KEEPOUT_OBJECT_NAME not in acm.entry_names:

        acm.entry_names.append(KEEPOUT_OBJECT_NAME)

        for entry in acm.entry_values:
            entry.enabled.append(False)

        acm.entry_values.append(
            AllowedCollisionEntry(
                enabled=[False] * len(acm.entry_names)
            )
        )

    keepout_index = acm.entry_names.index(KEEPOUT_OBJECT_NAME)

    for link in ignored_links:

        if link not in acm.entry_names:
            rospy.logwarn(
                "Link '%s' not found in the collision matrix. "
                "Skipping.",
                link
            )
            continue

        link_index = acm.entry_names.index(link)

        acm.entry_values[link_index].enabled[keepout_index] = True
        acm.entry_values[keepout_index].enabled[link_index] = True

    scene_diff = PlanningScene()
    scene_diff.is_diff = True
    scene_diff.allowed_collision_matrix = acm

    publisher = rospy.Publisher(
        "/planning_scene",
        PlanningScene,
        queue_size=1,
        latch=True
    )

    rospy.sleep(0.5)

    publisher.publish(scene_diff)

    rospy.sleep(0.5)

    rospy.loginfo(
        "Keep-out collisions allowed for links: %s",
        ", ".join(ignored_links)
    )


def find_keepout_contacts(robot):
    """
    Asks move_group whether the current state is collision-free and
    which bodies are in contact with the keep-out sphere.

    Returns (valid, sphere_links, other_pairs).
    """

    rospy.wait_for_service("/check_state_validity", timeout=5.0)

    check_validity = rospy.ServiceProxy(
        "/check_state_validity",
        GetStateValidity
    )

    response = check_validity(
        robot_state=robot.get_current_state(),
        group_name=PLANNING_GROUP
    )

    sphere_links = set()
    other_pairs = []

    for contact in response.contacts:

        bodies = {
            contact.contact_body_1,
            contact.contact_body_2
        }

        if KEEPOUT_OBJECT_NAME in bodies:
            bodies.discard(KEEPOUT_OBJECT_NAME)
            sphere_links.update(bodies)
        else:
            other_pairs.append(
                "{} <-> {}".format(
                    contact.contact_body_1,
                    contact.contact_body_2
                )
            )

    return response.valid, sorted(sphere_links), other_pairs


def ensure_state_clear_of_keepout(robot):
    """
    Verifies that the current state is not considered in collision
    with the keep-out sphere. Any link still touching the sphere is
    exempted (the camera mount has to sit this close to the object),
    with a warning so it is always visible what was exempted.
    """

    for _ in range(3):

        valid, sphere_links, other_pairs = find_keepout_contacts(
            robot
        )

        if valid:
            rospy.loginfo(
                "Current state is collision-free with the "
                "keep-out sphere in place."
            )
            return True

        if sphere_links:
            rospy.logwarn(
                "Links in collision with the keep-out sphere: %s. "
                "Exempting them from the collision check.",
                ", ".join(sphere_links)
            )

            allow_keepout_collisions(sphere_links)
            continue

        rospy.logerr(
            "Current state is in collision for reasons unrelated "
            "to the keep-out sphere: %s",
            "; ".join(other_pairs) if other_pairs else "unknown"
        )
        return False

    valid, _, _ = find_keepout_contacts(robot)
    return valid


def pose_distance(pose_a, pose_b):
    """
    Euclidean distance between the positions of two poses.
    """

    return math.sqrt(
        (pose_a.position.x - pose_b.position.x) ** 2 +
        (pose_a.position.y - pose_b.position.y) ** 2 +
        (pose_a.position.z - pose_b.position.z) ** 2
    )


def target_reached(move_group, target_pose):
    """
    Checks whether the end-effector is close enough to the target,
    regardless of what the controller reported.
    """

    current_pose = move_group.get_current_pose().pose

    distance = pose_distance(current_pose, target_pose)

    rospy.loginfo(
        "Distance from target: %.4f m",
        distance
    )

    return distance <= REACHED_DISTANCE_TOLERANCE


def plan_cartesian_segment(move_group, target_pose):
    """
    Plans a straight-line Cartesian segment from the current state
    to the target pose.
    """

    try:
        return move_group.compute_cartesian_path(
            [target_pose],
            CARTESIAN_EEF_STEP,
            CARTESIAN_JUMP_THRESHOLD
        )
    except TypeError:
        # Newer MoveIt versions dropped the jump_threshold argument.
        return move_group.compute_cartesian_path(
            [target_pose],
            CARTESIAN_EEF_STEP
        )


def retime_plan(move_group, robot, plan):
    """
    Re-times a plan to the configured velocity and acceleration
    scaling. Cartesian plans ignore the scaling factors set on the
    move group, so this step is required to keep the motion slow.
    """

    try:
        return move_group.retime_trajectory(
            robot.get_current_state(),
            plan,
            velocity_scaling_factor=VELOCITY_SCALE,
            acceleration_scaling_factor=ACCELERATION_SCALE
        )
    except TypeError:
        # Older MoveIt versions only accept the velocity factor.
        return move_group.retime_trajectory(
            robot.get_current_state(),
            plan,
            VELOCITY_SCALE
        )


def move_to_joint_position(move_group, joint_position):
    """
    Moves the robot to the specified joint configuration.
    """

    rospy.loginfo("Moving to the initial joint position...")

    move_group.set_joint_value_target(joint_position)

    success = move_group.go(wait=True)

    move_group.stop()
    move_group.clear_pose_targets()

    if not success:
        rospy.logerr(
            "Unable to reach the initial joint position."
        )
        return False

    rospy.loginfo("Initial joint position reached.")

    return True


def move_to_pose(move_group, robot, target_pose, index, total_points):
    """
    Moves toward a single Cartesian waypoint.

    Tries a straight-line Cartesian segment first (reliable for the
    small distances between consecutive waypoints), then falls back
    to regular pose-target planning. If the controller reports
    failure but the end-effector is within tolerance of the target,
    the waypoint is considered reached.
    """

    rospy.loginfo(
        "Planning waypoint %d/%d...",
        index,
        total_points
    )

    move_group.set_start_state_to_current_state()

    success = False

    (plan, fraction) = plan_cartesian_segment(
        move_group,
        target_pose
    )

    if fraction >= CARTESIAN_MIN_FRACTION:

        plan = retime_plan(move_group, robot, plan)

        try:
            success = move_group.execute(plan, wait=True)
        except Exception as error:
            rospy.logwarn(
                "Error while executing the Cartesian segment for "
                "waypoint %d: %s",
                index,
                str(error)
            )

        move_group.stop()

    else:
        rospy.logwarn(
            "Cartesian segment for waypoint %d only %.0f%% feasible. "
            "Falling back to pose-target planning.",
            index,
            fraction * 100.0
        )

        if fraction <= 0.0:
            rospy.logwarn(
                "0%% feasible usually means the current state is "
                "considered in collision - e.g. the keep-out sphere "
                "touching a link that is not in "
                "_keepout_ignored_links. Try _object_radius:=0 to "
                "check."
            )

    if not success and target_reached(move_group, target_pose):
        rospy.logwarn(
            "Controller reported failure but waypoint %d is within "
            "tolerance. Continuing.",
            index
        )
        success = True

    if not success:

        rospy.loginfo(
            "Pose-target planning for waypoint %d/%d...",
            index,
            total_points
        )

        move_group.set_start_state_to_current_state()
        move_group.set_pose_target(target_pose)

        try:
            success = move_group.go(wait=True)
        except Exception as error:
            rospy.logerr(
                "Error while moving to waypoint %d: %s",
                index,
                str(error)
            )

        move_group.stop()
        move_group.clear_pose_targets()

    if not success and target_reached(move_group, target_pose):
        rospy.logwarn(
            "Controller reported failure but waypoint %d is within "
            "tolerance. Continuing.",
            index
        )
        success = True

    if not success:
        rospy.logerr(
            "Unable to reach waypoint %d/%d.",
            index,
            total_points
        )
        return False

    rospy.loginfo(
        "Waypoint %d/%d reached.",
        index,
        total_points
    )

    return True


def print_current_pose(move_group, label):
    """
    Prints the current Cartesian pose.
    """

    pose = move_group.get_current_pose().pose

    rospy.loginfo(
        "%s - position: x=%.6f, y=%.6f, z=%.6f",
        label,
        pose.position.x,
        pose.position.y,
        pose.position.z
    )

    rospy.loginfo(
        "%s - orientation: x=%.6f, y=%.6f, z=%.6f, w=%.6f",
        label,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node(
        "panda_semicircle_motion",
        anonymous=False
    )

    default_joint_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "joint_start.csv"
    )

    joint_file = rospy.get_param(
        "~joint_file",
        default_joint_file
    )

    execute_motion = rospy.get_param(
        "~execute",
        False
    )

    wait_between_points = rospy.get_param(
        "~wait_between_points",
        WAIT_BETWEEN_POINTS
    )

    radius = rospy.get_param(
        "~radius",
        RADIUS_METERS
    )

    arc_degrees = rospy.get_param(
        "~arc_degrees",
        ARC_DEGREES
    )

    number_of_points = rospy.get_param(
        "~number_of_points",
        NUMBER_OF_POINTS
    )

    track_object = rospy.get_param(
        "~track_object",
        True
    )

    object_radius = rospy.get_param(
        "~object_radius",
        radius * OBJECT_RADIUS_RATIO
    )

    keepout_ignored_links = [
        link.strip()
        for link in rospy.get_param(
            "~keepout_ignored_links",
            KEEPOUT_IGNORED_LINKS
        ).split(",")
        if link.strip()
    ]

    rod_radius = rospy.get_param(
        "~rod_radius",
        SUPPORT_ROD_RADIUS
    )

    # Height of the object above the ground = length of the rod.
    object_height = rospy.get_param(
        "~object_height",
        0.55
    )

    output_file = rospy.get_param(
        "~output_file",
        ""
    )

    if not output_file:
        output_file = "arc_poses_{}.csv".format(
            datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        )

    rospy.loginfo("Joint file: %s", joint_file)
    rospy.loginfo("Execution enabled: %s", execute_motion)
    rospy.loginfo("Radius: %.4f m", radius)
    rospy.loginfo("Arc: %.2f degrees", arc_degrees)
    rospy.loginfo("Number of waypoints: %d", number_of_points)
    rospy.loginfo("Camera tracks the object: %s", track_object)
    rospy.loginfo(
        "Keep-out sphere radius: %.4f m (0 = disabled)",
        object_radius
    )
    rospy.loginfo(
        "Support rod radius: %.4f m (0 = disabled)",
        rod_radius
    )
    rospy.loginfo(
        "Object height above the ground: %.3f m",
        object_height
    )
    rospy.loginfo(
        "Output file for joint values and poses: %s",
        output_file
    )
    rospy.loginfo(
        "Wait between waypoints: %.2f seconds",
        wait_between_points
    )

    if arc_degrees > 90.0:
        rospy.logwarn(
            "Arc of %.1f degrees goes below the object's height: "
            "the hand will work close to the support rod on the "
            "final waypoints (intended for bottom views - MoveIt "
            "will reject any waypoint that would touch the rod).",
            arc_degrees
        )

    if object_radius >= radius:
        rospy.logerr(
            "The keep-out sphere radius (%.4f m) must be smaller "
            "than the arc radius (%.4f m), otherwise the camera "
            "poses themselves are in collision.",
            object_radius,
            radius
        )

        moveit_commander.roscpp_shutdown()
        return 1

    try:
        initial_joint_position = load_joint_position(joint_file)
    except Exception as error:
        rospy.logerr(
            "Error while reading the joint file: %s",
            str(error)
        )

        moveit_commander.roscpp_shutdown()
        return 1

    rospy.loginfo(
        "Initial configuration: %s",
        ["{:.9f}".format(value)
         for value in initial_joint_position]
    )

    try:
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        move_group = moveit_commander.MoveGroupCommander(
            PLANNING_GROUP
        )
    except Exception as error:
        rospy.logerr(
            "Unable to initialize MoveGroupCommander: %s",
            str(error)
        )

        moveit_commander.roscpp_shutdown()
        return 1

    move_group.set_max_velocity_scaling_factor(
        VELOCITY_SCALE
    )

    move_group.set_max_acceleration_scaling_factor(
        ACCELERATION_SCALE
    )

    move_group.set_goal_joint_tolerance(
        JOINT_TOLERANCE
    )

    move_group.set_goal_position_tolerance(
        POSITION_TOLERANCE
    )

    move_group.set_goal_orientation_tolerance(
        ORIENTATION_TOLERANCE
    )

    move_group.set_planning_time(
        PLANNING_TIME
    )

    move_group.set_num_planning_attempts(
        PLANNING_ATTEMPTS
    )

    rospy.sleep(2.0)

    current_joint_values = move_group.get_current_joint_values()

    if len(current_joint_values) != 7:
        rospy.logerr(
            "The planning group '%s' contains %d joints instead of 7.",
            PLANNING_GROUP,
            len(current_joint_values)
        )

        moveit_commander.roscpp_shutdown()
        return 1

    if not execute_motion:
        rospy.logwarn(
            "Safety mode active: the robot will not be moved."
        )

        rospy.logwarn(
            "To actually execute the motion add "
            "_execute:=true."
        )

        moveit_commander.roscpp_shutdown()
        return 0

    rospy.logwarn(
        "Check that the workspace is clear and "
        "that the emergency stop button is available."
    )

    if not move_to_joint_position(
            move_group,
            initial_joint_position):
        moveit_commander.roscpp_shutdown()
        return 1

    rospy.loginfo(
        "Waiting %.2f seconds at the initial position...",
        WAIT_AFTER_INITIAL_POSITION
    )

    rospy.sleep(WAIT_AFTER_INITIAL_POSITION)

    print_current_pose(
        move_group,
        "Initial pose reached"
    )

    start_pose = copy.deepcopy(
        move_group.get_current_pose().pose
    )

    try:
        log_handle, log_writer = open_pose_log(output_file)
    except Exception as error:
        rospy.logerr(
            "Unable to open the output file: %s",
            str(error)
        )

        moveit_commander.roscpp_shutdown()
        return 1

    # The start pose is recorded as waypoint 0 at angle 0.
    log_current_pose(log_handle, log_writer, move_group, 0, 0.0)

    sphere_center = compute_sphere_center(start_pose, radius)

    rospy.loginfo(
        "Object (sphere center): x=%.6f, y=%.6f, z=%.6f - "
        "camera-object distance %.4f m",
        sphere_center.x,
        sphere_center.y,
        sphere_center.z,
        radius
    )

    if rod_radius > 0.0:
        add_support_rod(
            scene,
            move_group,
            sphere_center,
            rod_radius,
            object_height
        )
    else:
        rospy.logwarn(
            "Support rod disabled (_rod_radius <= 0)."
        )

    if object_radius > 0.0:
        add_object_keepout(
            scene,
            move_group,
            sphere_center,
            object_radius
        )

        try:
            allow_keepout_collisions(keepout_ignored_links)

            if not ensure_state_clear_of_keepout(robot):
                rospy.logerr(
                    "The current state is still considered in "
                    "collision - planning will fail. Use "
                    "_object_radius:=0 to disable the sphere."
                )
        except Exception as error:
            rospy.logerr(
                "Unable to update the collision matrix: %s. "
                "The hand may be considered in collision with the "
                "keep-out sphere and planning may fail. Use "
                "_object_radius:=0 to disable the sphere.",
                str(error)
            )
    else:
        rospy.logwarn(
            "Keep-out sphere disabled (_object_radius <= 0)."
        )

    try:
        waypoints = create_arc_waypoints(
            start_pose=start_pose,
            radius=radius,
            arc_degrees=arc_degrees,
            number_of_points=number_of_points,
            track_object=track_object
        )
    except Exception as error:
        rospy.logerr(
            "Error while generating the waypoints: %s",
            str(error)
        )

        moveit_commander.roscpp_shutdown()
        return 1

    total_points = len(waypoints)

    for index, waypoint in enumerate(waypoints, start=1):

        if rospy.is_shutdown():
            rospy.logwarn(
                "ROS shut down. Stopping the motion."
            )
            break

        rospy.loginfo(
            "Moving to waypoint %d/%d...",
            index,
            total_points
        )

        success = move_to_pose(
            move_group=move_group,
            robot=robot,
            target_pose=waypoint,
            index=index,
            total_points=total_points
        )

        if not success:
            rospy.logerr(
                "Trajectory stopped at waypoint %d. Poses recorded "
                "so far are kept in %s",
                index,
                output_file
            )

            log_handle.close()
            moveit_commander.roscpp_shutdown()
            return 1

        print_current_pose(
            move_group,
            "Waypoint {} reached".format(index)
        )

        log_current_pose(
            log_handle,
            log_writer,
            move_group,
            index,
            arc_degrees * float(index) / float(total_points)
        )

        if index < total_points:
            rospy.loginfo(
                "Waiting %.2f seconds before the next waypoint...",
                wait_between_points
            )

            rospy.sleep(wait_between_points)

    rospy.loginfo(
        "All waypoints have been completed."
    )

    log_handle.close()

    rospy.loginfo(
        "Joint values and poses saved to: %s",
        os.path.abspath(os.path.expanduser(output_file))
    )

    move_group.stop()
    move_group.clear_pose_targets()

    moveit_commander.roscpp_shutdown()

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
