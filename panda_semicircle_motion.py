#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Arc motion for Franka Emika Panda.

The script:

1. reads an initial joint configuration from a CSV file;
1b. attaches the custom phone-holder mesh to the flange as
    collision geometry (the holder replaces the Franka gripper -
    launch franka_control.launch with load_gripper:=false);
2. moves the robot to the initial configuration;
3. reads the Cartesian pose that was reached;
4. computes the phone-lens position from that pose and the lens
   transform (_lens_xyz/_lens_axis, flange frame) and places the
   object (sphere center) at distance 'radius' from the LENS along
   the camera axis - straight below it when the start pose aims
   the camera down;
4b. once the initial pose is reached, ALWAYS waits for Enter
    before anything else moves - the moment to place the object
    under the lens (the exact position is in the log);
5. generates waypoints along an arc in the Y-Z plane, centered on
   the object, descending from above toward the -Y side by default
   (_arc_direction:=1 restores the original +Y sweep), so the
   lens-object distance stays constant;
6. rotates the whole pose rigidly at each waypoint so the camera
   keeps aiming at the object; the image ROLL about the camera
   axis is free (_free_roll, default true): at every waypoint the
   script tries roll angles - nearest the previous waypoint's roll
   first - and moves to the first pose the arm can actually reach.
   Any roll keeps the lens position, the lens-object distance and
   the aim unchanged, so the object stays centered in frame; the
   CSV records the pose actually reached, which is what matters
   for reconstruction (e.g. 3DGS);
7. adds the object's sphere to the planning scene as a keep-out
   region so no part of the arm can plan through it;
7b. adds the white background screen (_screen_mesh, a half-
    cylinder shell whose floor hole slides over the support rod)
    as a world obstacle: its x/y is placed on the rod axis so the
    insect sits inside the semicircle, its bottom sits
    _screen_height above the ground, and its opening faces the
    camera side of the arc (yaw picked from _arc_direction unless
    _screen_yaw_deg overrides it). NO link may touch it;
8. between waypoints: waits _wait_between_points seconds, or, with
   _confirm_each_pose:=true, waits for Enter at every stop instead
   (take the photo, press Enter, the arm moves on);
9. after the last waypoint (final photo: Enter with
   _confirm_each_pose:=true, _wait_between_points pause
   otherwise), plans a joint-space move back to the initial
   configuration with all obstacles (sphere, rod, screen) still in
   the planning scene - _return_to_start:=false leaves the arm at
   the last waypoint instead. Skipped if the arc was aborted.

Example joint_start.csv:

0.013337267496607689,0.13310515648967364,0.12014143002274653,-1.703551451575958,0.007806931131451653,1.0049740870915318,0.9248341724704006
"""

from __future__ import print_function

import copy
import csv
import datetime
import math
import os
import struct
import sys

import numpy as np
import rospy
import moveit_commander

from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents
)
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from tf.transformations import (
    quaternion_about_axis,
    quaternion_matrix,
    quaternion_multiply
)


PLANNING_GROUP = "panda_arm"

RADIUS_METERS = 0.04
ARC_DEGREES = 60.0
NUMBER_OF_POINTS = 9

# Side of the world Y axis the arc descends toward: -1.0 sweeps
# toward -Y (default since 2026-07-20 - the phone/lens side of the
# flange with Mount+phone.stl), +1.0 toward +Y (the original
# direction). The camera-aim rotation flips together with it.
ARC_DIRECTION = -1.0

# Image roll about the camera axis (only with _track_object:=true).
# True = at every waypoint try rolls in ROLL_STEP_DEG increments up
# to +/-ROLL_MAX_DEG, ordered by closeness to the roll used at the
# previous waypoint, and take the first reachable pose. The roll
# never moves the lens or the aim - the object stays centered in
# frame - it only rotates the image, and the true orientation is in
# the CSV. False = the original fixed zero-roll behavior.
FREE_ROLL = True
ROLL_STEP_DEG = 15.0
ROLL_MAX_DEG = 180.0

WAIT_BETWEEN_POINTS = 2.0
WAIT_AFTER_INITIAL_POSITION = 2.0

# When True the script stops at every waypoint and waits for Enter
# before moving to the next one (photo workflow); when False it
# pauses _wait_between_points seconds instead. Independent of this
# flag, the script ALWAYS waits for Enter once the robot is at the
# initial pose, so the object can be placed under the lens before
# the arc starts.
CONFIRM_EACH_POSE = False

# After the last waypoint the robot plans a collision-aware
# joint-space move back to the initial configuration (the
# obstacles stay in the planning scene). Only runs when the whole
# arc completed; a run aborted at a waypoint leaves the arm where
# it stopped.
RETURN_TO_START = True

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

# Same idea for the initial joint move: worst joint error (rad)
# below which "failure" from the controller is treated as arrived.
JOINT_REACHED_TOLERANCE = 0.01

# Default keep-out sphere radius as a fraction of the arc radius.
OBJECT_RADIUS_RATIO = 0.5

KEEPOUT_OBJECT_NAME = "object_keepout"

# The object sits on a thin vertical rod (3 mm diameter = 1.5 mm
# radius) reaching up from the ground. Modeled with a safety
# margin; 0 disables it.
ROD_OBJECT_NAME = "support_rod"
SUPPORT_ROD_RADIUS = 0.005

# White background screen behind the insect
# (background-white-screen.stl, modeled in METERS): a half-
# cylinder shell, wall radius 0.110-0.1125 m and 0.194 m tall,
# with a 6 mm floor plate whose 4.4 mm hole sits exactly at the
# arc center - the hole slides over the support rod, so the mesh
# x/y origin IS the rod axis and the insect sits at the center of
# the semicircle. The wall bulges toward mesh +Y. Added as a world
# obstacle: NO link is exempt from it (unlike the keep-out
# sphere).
SCREEN_OBJECT_NAME = "background_screen"
SCREEN_MESH_FILE = "background-white-screen.stl"
SCREEN_MESH_SCALE = 1.0  # this STL is modeled in meters

# Height of the screen's BOTTOM above the ground.
SCREEN_HEIGHT_ABOVE_GROUND = 0.425

# "auto" = open side toward the camera: with the default -Y arc
# the wall goes behind the insect at +Y (yaw 0); with
# _arc_direction:=1 the screen is yawed 180 degrees instead. A
# number forces that yaw about the world Z axis.
SCREEN_YAW_DEG = "auto"

# Legacy scalar lens offset: distance from the flange down to the
# lens along the flange z axis. Only used when _lens_xyz:='' is
# passed explicitly; the mesh-measured lens transform below is the
# default.
CAMERA_OFFSET_METERS = 0.10

# Ultra-wide lens of the iPhone 15 Pro in the phone holder, in the
# panda_link8 (flange) frame, measured from phone_mount_conf3.stl
# (the phone sits at 45 degrees in the cradle, flipped 180 degrees
# vs the original Mount+phone.stl, camera bump near the cradle
# center; conf3 = conf2 slid 10.3 mm toward link8 -Y). Values
# obtained by rigid-transforming the fitted lens-ring circles of
# the previous mesh (phone part matches to <1e-5 mm) and verified
# by a fresh circle fit on this mesh (0.3 mm agreement). On the
# iPhone 15 Pro the ultra-wide is the BOTTOM-LEFT lens of the bump
# (back view, portrait); main is at link8 (-0.0225, -0.0431,
# 0.1170), telephoto at (-0.0359, -0.0336, 0.1036). Verify which
# ring is the ultra-wide by covering the lenses one at a time with
# the Camera app at 0.5x.
LENS_XYZ_LINK8 = "-0.0225,-0.0239,0.1170"

# Direction the camera looks, unit vector in the panda_link8 frame
# (normal of the phone back): 45 degrees between the flange z axis
# and -x.
LENS_AXIS_LINK8 = "-0.70711,0.0,0.70711"

POSE_LOG_HEADER = [
    "waypoint",
    "angle_deg",
    "joint1", "joint2", "joint3", "joint4",
    "joint5", "joint6", "joint7",
    "position_x", "position_y", "position_z",
    "orientation_x", "orientation_y", "orientation_z", "orientation_w",
    "lens_x", "lens_y", "lens_z"
]

# Links allowed to touch the keep-out sphere. The hand carries the
# camera and must get within 'radius' of the object, so it cannot be
# collision-checked against the sphere; the rest of the arm still is.
# The *_sc links are the coarse capsule collision bodies that newer
# panda_moveit_config versions add around the visible links. The
# panda_hand/finger entries only exist while the URDF still loads
# the gripper; with the phone holder mounted they are simply absent.
KEEPOUT_IGNORED_LINKS = (
    "panda_link8,panda_hand,panda_leftfinger,panda_rightfinger,"
    "panda_hand_sc,panda_link7_sc,panda_link8_sc,phone_holder"
)

# The custom phone-holder hand - it REPLACES the Franka gripper, so
# launch franka_control.launch with load_gripper:=false. The mesh is
# rigidly attached to the flange as a collision object so MoveIt
# plans around the real holder geometry (the support rod especially).
#
# Alignment, derived from the mesh's DIN ISO 9409-1-A50 mounting
# face (mesh coordinates, millimeters):
#   - outer rim (diam. 63, the flange diameter) centered on the mesh
#     origin -> no x/y offset, mesh z axis = flange axis;
#   - mounting face at z = +8 -> HOLDER_Z_OFFSET = -0.008 puts it
#     flush on the flange surface (= panda_link8 origin);
#   - dowel-pin hole (diam. 6, on the diam. 50 pitch circle) at +90
#     deg = the mesh +Y axis. On the robot the flange pin lies on
#     the +X axis of panda_link8 (Franka Hand mesh: pin at +45 deg
#     in the hand frame, hand mounted at yaw -45 deg), so the holder
#     mounts at HOLDER_YAW_DEG = -90.
HOLDER_OBJECT_NAME = "phone_holder"
# Default mesh: holder WITH the phone - the phone is physically
# mounted, so its body must be collision-checked too (same frame
# and attach pose as the holder-only STL). The file must sit next
# to this script on the robot PC.
HOLDER_MESH_FILE = "phone_mount_conf3.stl"
HOLDER_ATTACH_LINK = "panda_link8"
HOLDER_MESH_SCALE = 0.001  # the STL is modeled in millimeters
HOLDER_Z_OFFSET = -0.008
HOLDER_YAW_DEG = -90.0

# Robot links the holder is allowed to touch (it is bolted to the
# flange). Links absent from the URDF are ignored by MoveIt.
HOLDER_TOUCH_LINKS = [
    "panda_link7", "panda_link8",
    "panda_link7_sc", "panda_link8_sc"
]


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


def parse_vector3(text, name):
    """
    Parses a 'x,y,z' string into a list of 3 floats.
    """

    values = [float(part) for part in text.split(",")]

    if len(values) != 3:
        raise ValueError(
            "{} must contain exactly 3 comma-separated values, "
            "got: {}".format(name, text)
        )

    return values


def rotate_about_x(vector, angle):
    """
    Rotates a 3D vector about the world X axis.
    """

    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    return np.array([
        vector[0],
        vector[1] * cos_a - vector[2] * sin_a,
        vector[1] * sin_a + vector[2] * cos_a
    ])


def compute_camera_geometry(start_pose, lens_xyz, lens_axis, radius):
    """
    Computes, in world coordinates, the starting lens position, the
    direction the camera looks, and the object (sphere center):
    the object is placed at distance 'radius' from the LENS, along
    the camera axis. lens_xyz / lens_axis are given in the flange
    (panda_link8) frame.
    """

    orientation = start_pose.orientation

    rotation = quaternion_matrix([
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    ])[:3, :3]

    flange_position = np.array([
        start_pose.position.x,
        start_pose.position.y,
        start_pose.position.z
    ])

    lens_position = flange_position + rotation.dot(lens_xyz)

    axis_world = rotation.dot(lens_axis)
    axis_world = axis_world / np.linalg.norm(axis_world)

    center = lens_position + radius * axis_world

    return lens_position, axis_world, center


def compute_aim_orientation(start_orientation, theta, direction):
    """
    Rotates the start orientation by -direction*theta about the
    world X axis.

    The waypoint positions rotate by -direction*theta about the X
    axis around the sphere center, so applying the same rotation to
    the orientation keeps the camera aiming at the object
    (direction +1 = arc toward +Y, -1 = toward -Y).
    """

    q_start = [
        start_orientation.x,
        start_orientation.y,
        start_orientation.z,
        start_orientation.w
    ]

    q_rot = quaternion_about_axis(
        -direction * theta, (1.0, 0.0, 0.0)
    )

    return quaternion_multiply(q_rot, q_start)


def make_flange_pose(lens_target, camera_axis, q_aim, roll, lens_xyz):
    """
    Builds the flange pose that puts the LENS exactly at
    lens_target looking along camera_axis (both world frame), with
    the image rolled by 'roll' radians about the camera axis.

    The flange position is derived from the orientation - the
    flange-to-lens offset rotates together with the roll - so every
    roll value keeps the same lens position, lens-object distance
    and aim; only the image orientation changes.
    """

    if abs(roll) > 1e-12:
        orientation = quaternion_multiply(
            quaternion_about_axis(roll, camera_axis),
            q_aim
        )
    else:
        orientation = q_aim

    rotation = quaternion_matrix(orientation)[:3, :3]

    position = np.asarray(lens_target) - rotation.dot(lens_xyz)

    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]

    return pose


def roll_candidates(step_deg, max_deg, previous_roll):
    """
    Roll angles (radians) to try at a waypoint: every step_deg
    within +/-max_deg, ordered by angular closeness to the roll
    used at the previous waypoint, so the image orientation drifts
    as little as possible between consecutive photos.
    """

    step = math.radians(step_deg)
    limit = math.radians(min(max_deg, 180.0)) + 1e-9

    rolls = [0.0]

    k = 1
    while k * step <= limit:
        rolls.append(k * step)
        if k * step < math.pi - 1e-9:
            # -180 deg duplicates +180 deg - keep only one.
            rolls.append(-k * step)
        k += 1

    def circular_distance(roll):
        delta = abs(roll - previous_roll) % (2.0 * math.pi)
        return min(delta, 2.0 * math.pi - delta)

    return sorted(rolls, key=circular_distance)


def create_arc_waypoints(
        start_pose,
        center,
        lens_start,
        arc_degrees,
        number_of_points,
        track_object,
        direction):
    """
    Generates one waypoint spec per arc stop so the LENS orbits the
    object (sphere center) at constant distance, sweeping an arc in
    the world Y-Z plane.

    +Y = right
    +Z = up

    Each spec is a dict:
      pose        - flange pose at zero image roll (the pose the
                    fixed-roll behavior uses);
      lens_target - world lens position at this stop;
      camera_axis - world unit vector the camera looks along
                    (lens -> object);
      q_aim       - flange orientation quaternion at zero roll.
    With track_object=False the last three are None.

    With track_object=True the whole start pose is rotated rigidly
    about the object center around the world X axis by
    -direction*theta: the lens keeps its exact starting distance
    and stays aimed at the object at every waypoint, whatever its
    offset from the flange. The camera starts looking at the object
    (straight down at the start pose) and descends along the sphere
    toward -Y (direction=-1, the default) or +Y (direction=+1); at
    90 degrees it looks at the object from the side, beyond that
    from underneath. The zero-roll pose is the baseline; the caller
    can re-roll it about camera_axis with make_flange_pose without
    disturbing the lens position or the aim.

    With track_object=False the orientation stays frozen: the LENS
    still orbits the center, and the flange keeps its constant
    world offset from the lens.
    """

    if number_of_points <= 0:
        raise ValueError(
            "The number of waypoints must be greater than zero."
        )

    flange_start = np.array([
        start_pose.position.x,
        start_pose.position.y,
        start_pose.position.z
    ])

    delta_flange = flange_start - center
    delta_lens = lens_start - center
    lens_offset_world = lens_start - flange_start

    axis_start = center - lens_start
    axis_start = axis_start / np.linalg.norm(axis_start)

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

        spec = {
            "lens_target": None,
            "camera_axis": None,
            "q_aim": None
        }

        if track_object:
            # Rotate the whole pose (flange position + orientation)
            # about the object center: the rigidly-attached lens
            # follows, staying at 'radius' and aimed at the object.
            position = center + rotate_about_x(
                delta_flange, -direction * theta
            )

            spec["lens_target"] = center + rotate_about_x(
                delta_lens, -direction * theta
            )
            spec["camera_axis"] = rotate_about_x(
                axis_start, -direction * theta
            )
            spec["q_aim"] = compute_aim_orientation(
                start_pose.orientation,
                theta,
                direction
            )
        else:
            # Frozen orientation: the lens orbits the center, and
            # the flange keeps its constant world offset from it.
            position = (
                center
                + rotate_about_x(delta_lens, -direction * theta)
                - lens_offset_world
            )

        waypoint = copy.deepcopy(start_pose)

        waypoint.position.x = position[0]
        waypoint.position.y = position[1]
        waypoint.position.z = position[2]

        if track_object:
            orientation = spec["q_aim"]

            waypoint.orientation.x = orientation[0]
            waypoint.orientation.y = orientation[1]
            waypoint.orientation.z = orientation[2]
            waypoint.orientation.w = orientation[3]

        spec["pose"] = waypoint

        waypoints.append(spec)

        rospy.loginfo(
            "Waypoint %d/%d - angle %.2f deg - "
            "position x %.6f - y %.6f - z %.6f",
            index,
            number_of_points,
            math.degrees(theta),
            waypoint.position.x,
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


def add_support_rod(scene, move_group, center, rod_radius,
                    object_height):
    """
    Adds the object's support rod - a thin vertical cylinder that
    holds the object 'object_height' above the ground - as a
    collision object, at full height (ground to object). Unlike the
    keep-out sphere, NO link is exempt from it.
    """

    bottom_z = center.z - object_height
    top_z = center.z
    rod_length = max(top_z - bottom_z, 0.01)

    rod_pose = PoseStamped()
    rod_pose.header.frame_id = move_group.get_planning_frame()
    rod_pose.pose.position.x = center.x
    rod_pose.pose.position.y = center.y
    rod_pose.pose.position.z = bottom_z + rod_length / 2.0
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
        "Support rod added: x=%.6f, y=%.6f, from z=%.3f (ground) "
        "up to z=%.3f (the object) - radius %.4f m",
        center.x,
        center.y,
        bottom_z,
        bottom_z + rod_length,
        rod_radius
    )


def load_stl_mesh(file_path, scale):
    """
    Loads a binary STL file into a shape_msgs/Mesh message, scaling
    the vertices (the holder is modeled in millimeters -> 0.001).
    Duplicate vertices are merged so the message stays compact.
    """

    file_path = os.path.abspath(os.path.expanduser(file_path))

    if not os.path.isfile(file_path):
        raise IOError(
            "Mesh file not found: {}".format(file_path)
        )

    file_size = os.path.getsize(file_path)

    mesh = Mesh()
    vertex_index = {}

    with open(file_path, "rb") as handle:

        handle.read(80)
        (triangle_count,) = struct.unpack("<I", handle.read(4))

        if file_size != 84 + triangle_count * 50:
            raise ValueError(
                "{} is not a binary STL file".format(file_path)
            )

        for _ in range(triangle_count):

            values = struct.unpack("<12fH", handle.read(50))

            indices = []

            for vertex in range(3):
                point = tuple(
                    round(values[3 + 3 * vertex + axis] * scale, 6)
                    for axis in range(3)
                )

                index = vertex_index.get(point)

                if index is None:
                    index = len(mesh.vertices)
                    vertex_index[point] = index
                    mesh.vertices.append(Point(*point))

                indices.append(index)

            if len(set(indices)) < 3:
                # Degenerate triangle (collapsed by the merge).
                continue

            mesh.triangles.append(
                MeshTriangle(vertex_indices=indices)
            )

    return mesh


def add_background_screen(scene, move_group, center, object_height,
                          mesh_file, scale, height_above_ground,
                          yaw_deg):
    """
    Adds the white background screen as a world collision obstacle.

    The mesh origin's vertical axis is the rod axis (the floor hole
    sits at the arc center), so the screen is placed at the
    object's x/y; the mesh bottom is placed height_above_ground
    above the ground (ground = object center - object_height). The
    yaw rotates the mesh about the world Z axis: at yaw 0 the wall
    bulges toward +Y, leaving the -Y side (the default camera side)
    open.

    Publishing on /collision_object avoids the pyassimp dependency
    of PlanningSceneInterface.add_mesh (same approach as the
    phone-holder attachment). Returns (bottom_z, top_z) of the
    screen in world coordinates.
    """

    mesh = load_stl_mesh(mesh_file, scale)

    z_values = [vertex.z for vertex in mesh.vertices]
    z_min = min(z_values)
    z_max = max(z_values)

    ground_z = center.z - object_height
    bottom_z = ground_z + height_above_ground

    mesh_pose = Pose()
    mesh_pose.position.x = center.x
    mesh_pose.position.y = center.y
    mesh_pose.position.z = bottom_z - z_min

    rotation = quaternion_about_axis(
        math.radians(yaw_deg),
        (0.0, 0.0, 1.0)
    )

    mesh_pose.orientation.x = rotation[0]
    mesh_pose.orientation.y = rotation[1]
    mesh_pose.orientation.z = rotation[2]
    mesh_pose.orientation.w = rotation[3]

    collision_object = CollisionObject()
    collision_object.header.frame_id = move_group.get_planning_frame()
    collision_object.id = SCREEN_OBJECT_NAME
    collision_object.meshes = [mesh]
    collision_object.mesh_poses = [mesh_pose]
    collision_object.operation = CollisionObject.ADD

    # Newer message versions add an object-level pose on top of the
    # mesh poses; it must be a valid identity, not all zeros.
    if hasattr(collision_object, "pose"):
        collision_object.pose.orientation.w = 1.0

    publisher = rospy.Publisher(
        "/collision_object",
        CollisionObject,
        queue_size=2,
        latch=True
    )

    rospy.sleep(0.5)

    scene.remove_world_object(SCREEN_OBJECT_NAME)
    rospy.sleep(0.5)

    publisher.publish(collision_object)
    rospy.sleep(1.0)

    try:
        confirmed = SCREEN_OBJECT_NAME in scene.get_known_object_names()
    except Exception:
        # Older moveit_commander versions - trust the latched
        # publication.
        confirmed = True

    if not confirmed:
        raise RuntimeError(
            "move_group did not confirm the background screen"
        )

    top_z = bottom_z + (z_max - z_min)

    rospy.loginfo(
        "Background screen added: %s (%d triangles), rod axis at "
        "x=%.6f, y=%.6f, bottom z=%.3f (%.1f cm above ground), "
        "top z=%.3f, yaw %.1f deg",
        mesh_file,
        len(mesh.triangles),
        center.x,
        center.y,
        bottom_z,
        height_above_ground * 100.0,
        top_z,
        yaw_deg
    )

    return bottom_z, top_z


def attach_phone_holder(scene, mesh_file, z_offset, yaw_deg):
    """
    Rigidly attaches the phone-holder mesh to the flange as an
    AttachedCollisionObject, so MoveIt collision-checks the real
    holder geometry against the support rod and the rest of the arm.

    Publishing on /attached_collision_object avoids the pyassimp
    dependency of PlanningSceneInterface.attach_mesh.
    """

    mesh = load_stl_mesh(mesh_file, HOLDER_MESH_SCALE)

    holder_pose = Pose()
    holder_pose.position.z = z_offset

    rotation = quaternion_about_axis(
        math.radians(yaw_deg),
        (0.0, 0.0, 1.0)
    )

    holder_pose.orientation.x = rotation[0]
    holder_pose.orientation.y = rotation[1]
    holder_pose.orientation.z = rotation[2]
    holder_pose.orientation.w = rotation[3]

    collision_object = CollisionObject()
    collision_object.header.frame_id = HOLDER_ATTACH_LINK
    collision_object.id = HOLDER_OBJECT_NAME
    collision_object.meshes = [mesh]
    collision_object.mesh_poses = [holder_pose]
    collision_object.operation = CollisionObject.ADD

    # Newer message versions add an object-level pose on top of the
    # mesh poses; it must be a valid identity, not all zeros.
    if hasattr(collision_object, "pose"):
        collision_object.pose.orientation.w = 1.0

    attached = AttachedCollisionObject()
    attached.link_name = HOLDER_ATTACH_LINK
    attached.object = collision_object
    attached.touch_links = HOLDER_TOUCH_LINKS

    publisher = rospy.Publisher(
        "/attached_collision_object",
        AttachedCollisionObject,
        queue_size=2,
        latch=True
    )

    rospy.sleep(0.5)

    # Detach any stale copy left over from a previous run (harmless
    # move_group warning if there is none) and remove the world
    # object the detach turns it into.
    detach = AttachedCollisionObject()
    detach.object.id = HOLDER_OBJECT_NAME
    detach.object.operation = CollisionObject.REMOVE

    publisher.publish(detach)
    rospy.sleep(0.5)

    scene.remove_world_object(HOLDER_OBJECT_NAME)
    rospy.sleep(0.5)

    publisher.publish(attached)
    rospy.sleep(1.0)

    try:
        confirmed = HOLDER_OBJECT_NAME in scene.get_attached_objects(
            [HOLDER_OBJECT_NAME]
        )
    except Exception:
        # Older moveit_commander versions cannot report attached
        # objects - trust the latched publication.
        confirmed = True

    if not confirmed:
        raise RuntimeError(
            "move_group did not confirm the phone holder attachment"
        )

    rospy.loginfo(
        "Phone holder attached to %s: %s (%d triangles), "
        "z offset %.4f m, yaw %.1f deg",
        HOLDER_ATTACH_LINK,
        mesh_file,
        len(mesh.triangles),
        z_offset,
        yaw_deg
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


def log_current_pose(handle, writer, move_group, waypoint, angle_deg,
                     lens_xyz):
    """
    Appends the current joint values, end-effector pose and world
    lens position to the output file.
    """

    joints = move_group.get_current_joint_values()
    pose = move_group.get_current_pose().pose

    rotation = quaternion_matrix([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ])[:3, :3]

    lens_world = np.array([
        pose.position.x,
        pose.position.y,
        pose.position.z
    ]) + rotation.dot(lens_xyz)

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
            pose.orientation.w,
            lens_world[0],
            lens_world[1],
            lens_world[2])]
    )

    handle.flush()


def ensure_acm_entry(acm, name):
    """
    Returns the index of 'name' in the Allowed Collision Matrix,
    adding a new all-disabled entry if it is missing. Bodies that
    are not robot links - the keep-out sphere, or the attached
    phone holder - have no entry until one is created.
    """

    if name not in acm.entry_names:

        acm.entry_names.append(name)

        for entry in acm.entry_values:
            entry.enabled.append(False)

        acm.entry_values.append(
            AllowedCollisionEntry(
                enabled=[False] * len(acm.entry_names)
            )
        )

    return acm.entry_names.index(name)


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

    keepout_index = ensure_acm_entry(acm, KEEPOUT_OBJECT_NAME)

    for link in ignored_links:

        link_index = ensure_acm_entry(acm, link)

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
    which bodies are in contact with the keep-out sphere, the
    support rod or the background screen.

    Returns (valid, sphere_links, rod_links, screen_links,
    other_pairs).
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
    rod_links = set()
    screen_links = set()
    other_pairs = []

    for contact in response.contacts:

        bodies = {
            contact.contact_body_1,
            contact.contact_body_2
        }

        if KEEPOUT_OBJECT_NAME in bodies:
            bodies.discard(KEEPOUT_OBJECT_NAME)
            sphere_links.update(bodies)
        elif ROD_OBJECT_NAME in bodies:
            bodies.discard(ROD_OBJECT_NAME)
            rod_links.update(bodies)
        elif SCREEN_OBJECT_NAME in bodies:
            bodies.discard(SCREEN_OBJECT_NAME)
            screen_links.update(bodies)
        else:
            other_pairs.append(
                "{} <-> {}".format(
                    contact.contact_body_1,
                    contact.contact_body_2
                )
            )

    return (response.valid, sorted(sphere_links),
            sorted(rod_links), sorted(screen_links), other_pairs)


def ensure_state_clear(robot):
    """
    Verifies that the current state is not considered in collision
    with the keep-out sphere, the support rod or the background
    screen. Links touching the sphere are exempted (the camera
    mount has to sit this close to the object); a collision with
    the rod or the screen is an error - both are always checked
    against every link.
    """

    for _ in range(5):

        valid, sphere_links, rod_links, screen_links, other_pairs = (
            find_keepout_contacts(robot)
        )

        if valid:
            rospy.loginfo(
                "Current state is collision-free with the "
                "keep-out sphere, support rod and background "
                "screen in place."
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

        if rod_links:
            rospy.logerr(
                "Links in collision with the support rod: %s. "
                "Adjust the start pose or the radius.",
                ", ".join(rod_links)
            )
            return False

        if screen_links:
            rospy.logerr(
                "Links in collision with the background screen: "
                "%s. Adjust the start pose, or check "
                "_screen_height / _screen_yaw_deg.",
                ", ".join(screen_links)
            )
            return False

        rospy.logerr(
            "Current state is in collision for reasons unrelated "
            "to the keep-out sphere or the rod: %s",
            "; ".join(other_pairs) if other_pairs else "unknown"
        )
        return False

    valid, _, _, _, _ = find_keepout_contacts(robot)
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

    try:
        success = move_group.go(wait=True)
    except Exception as error:
        rospy.logerr(
            "Error while moving to the initial joint position: %s",
            str(error)
        )
        success = False

    move_group.stop()
    move_group.clear_pose_targets()

    if not success:
        # The Franka controller sometimes reports failure (goal
        # tolerance) even though the arm arrived - check the actual
        # joint error before giving up.
        current = move_group.get_current_joint_values()

        max_error = max(
            abs(value - target)
            for value, target in zip(current, joint_position)
        )

        rospy.loginfo(
            "Worst joint error after the reported failure: "
            "%.4f rad",
            max_error
        )

        if max_error <= JOINT_REACHED_TOLERANCE:
            rospy.logwarn(
                "Controller reported failure but the initial joint "
                "position is within tolerance. Continuing."
            )
            success = True

    if not success:
        rospy.logerr(
            "Unable to reach the initial joint position."
        )
        return False

    rospy.loginfo("Initial joint position reached.")

    return True


def move_to_pose(move_group, robot, candidates, index, total_points):
    """
    Moves toward a single Cartesian waypoint, given as a list of
    (roll, pose) candidates that all put the lens at the same place
    aiming at the object, differing only in the image roll (a
    single entry when the roll is fixed).

    Tries a straight-line Cartesian segment for each candidate in
    order (reliable for the small distances between consecutive
    waypoints) and executes the first fully-feasible one; if none
    is, falls back to regular pose-target planning on the candidate
    whose segment got the furthest. If the controller reports
    failure but the end-effector is within tolerance of the target,
    the waypoint is considered reached.

    Returns the roll (radians) of the pose that was reached, or
    None on failure.
    """

    rospy.loginfo(
        "Planning waypoint %d/%d (%d roll candidate%s)...",
        index,
        total_points,
        len(candidates),
        "" if len(candidates) == 1 else "s"
    )

    move_group.set_start_state_to_current_state()

    success = False

    best_roll = candidates[0][0]
    best_pose = candidates[0][1]
    best_fraction = -1.0

    for roll, pose in candidates:

        (plan, fraction) = plan_cartesian_segment(move_group, pose)

        if fraction > best_fraction:
            best_roll = roll
            best_pose = pose
            best_fraction = fraction

        if fraction < CARTESIAN_MIN_FRACTION:
            continue

        best_roll = roll
        best_pose = pose

        if len(candidates) > 1:
            rospy.loginfo(
                "Waypoint %d: Cartesian segment feasible with "
                "image roll %.1f deg.",
                index,
                math.degrees(roll)
            )

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
        break

    else:
        rospy.logwarn(
            "No Cartesian segment for waypoint %d is fully "
            "feasible (best %.0f%% at image roll %.1f deg). "
            "Falling back to pose-target planning.",
            index,
            best_fraction * 100.0,
            math.degrees(best_roll)
        )

        if best_fraction <= 0.0:
            rospy.logwarn(
                "0%% feasible usually means the current state is "
                "considered in collision - e.g. the keep-out sphere "
                "touching a link that is not in "
                "_keepout_ignored_links. Try _object_radius:=0 to "
                "check."
            )

    if not success and target_reached(move_group, best_pose):
        rospy.logwarn(
            "Controller reported failure but waypoint %d is within "
            "tolerance. Continuing.",
            index
        )
        success = True

    if not success:

        rospy.loginfo(
            "Pose-target planning for waypoint %d/%d (image roll "
            "%.1f deg)...",
            index,
            total_points,
            math.degrees(best_roll)
        )

        move_group.set_start_state_to_current_state()
        move_group.set_pose_target(best_pose)

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

    if not success and target_reached(move_group, best_pose):
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
        return None

    rospy.loginfo(
        "Waypoint %d/%d reached.",
        index,
        total_points
    )

    return best_roll


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


def wait_for_enter(message):
    """
    Blocks until Enter is pressed in this terminal. Ctrl-C still
    aborts the script as usual.
    """

    rospy.loginfo(message)

    try:
        input(">>> Press Enter to continue... ")
    except EOFError:
        rospy.logwarn(
            "stdin is closed - cannot wait for input, continuing."
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

    # true = wait for Enter at every waypoint before moving to the
    # next one; false = pause _wait_between_points seconds instead.
    # The Enter confirmation at the initial pose is unconditional.
    confirm_each_pose = rospy.get_param(
        "~confirm_each_pose",
        CONFIRM_EACH_POSE
    )

    # true = after the last waypoint (and the final photo pause)
    # the robot returns to the initial joint configuration with a
    # collision-aware plan; false = it stays at the last waypoint.
    return_to_start = rospy.get_param(
        "~return_to_start",
        RETURN_TO_START
    )

    radius = rospy.get_param(
        "~radius",
        RADIUS_METERS
    )

    arc_degrees = rospy.get_param(
        "~arc_degrees",
        ARC_DEGREES
    )

    # -1 = descend toward -Y (default), +1 = toward +Y. Any other
    # value is normalized to its sign.
    arc_direction = rospy.get_param(
        "~arc_direction",
        ARC_DIRECTION
    )

    arc_direction = -1.0 if float(arc_direction) < 0.0 else 1.0

    number_of_points = rospy.get_param(
        "~number_of_points",
        NUMBER_OF_POINTS
    )

    # The hand rotates along the arc so the camera always faces the
    # object (confirmed after testing: a fixed world orientation
    # makes the far waypoints physically unreachable).
    # _track_object:=false keeps the orientation frozen instead.
    track_object = rospy.get_param(
        "~track_object",
        True
    )

    # With _track_object:=true the image roll about the camera axis
    # is free by default: each waypoint tries the rolls nearest the
    # previous waypoint's roll first and moves to the first pose
    # the arm can reach. The roll never moves the lens or the aim
    # (the object stays centered in frame) and the CSV records the
    # orientation actually reached, which is what reconstruction
    # needs. _free_roll:=false restores the fixed zero-roll poses.
    free_roll = rospy.get_param(
        "~free_roll",
        FREE_ROLL
    )

    roll_step_deg = rospy.get_param(
        "~roll_step_deg",
        ROLL_STEP_DEG
    )

    roll_max_deg = rospy.get_param(
        "~roll_max_deg",
        ROLL_MAX_DEG
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

    # White background screen mesh (world obstacle, the floor hole
    # slides over the support rod). '' disables it. Like the holder
    # STL, the file must sit next to this script on the robot PC.
    default_screen_mesh = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        SCREEN_MESH_FILE
    )

    screen_mesh = rospy.get_param(
        "~screen_mesh",
        default_screen_mesh
    )

    screen_scale = rospy.get_param(
        "~screen_scale",
        SCREEN_MESH_SCALE
    )

    # Height of the screen's bottom above the ground.
    screen_height = rospy.get_param(
        "~screen_height",
        SCREEN_HEIGHT_ABOVE_GROUND
    )

    # "auto" = derived from _arc_direction so the opening faces the
    # camera side; a number forces that yaw (deg, about world Z).
    screen_yaw_param = rospy.get_param(
        "~screen_yaw_deg",
        SCREEN_YAW_DEG
    )

    if isinstance(screen_yaw_param, str):
        if screen_yaw_param.strip().lower() != "auto":
            try:
                screen_yaw_deg = float(screen_yaw_param)
            except ValueError:
                rospy.logerr(
                    "_screen_yaw_deg must be a number or 'auto', "
                    "got: %s",
                    screen_yaw_param
                )

                moveit_commander.roscpp_shutdown()
                return 1
        else:
            screen_yaw_deg = 0.0 if arc_direction < 0.0 else 180.0
    else:
        screen_yaw_deg = float(screen_yaw_param)

    camera_offset = rospy.get_param(
        "~camera_offset",
        CAMERA_OFFSET_METERS
    )

    # Lens transform in the flange frame. The default is the
    # ultra-wide lens of the iPhone 15 Pro measured from
    # Mount+phone.stl; _lens_xyz:='' falls back to the legacy
    # behavior (lens on the flange z axis at _camera_offset).
    lens_xyz_text = rospy.get_param(
        "~lens_xyz",
        LENS_XYZ_LINK8
    )

    lens_axis_text = rospy.get_param(
        "~lens_axis",
        LENS_AXIS_LINK8
    )

    try:
        if lens_xyz_text:
            lens_xyz = np.array(
                parse_vector3(lens_xyz_text, "_lens_xyz")
            )
            lens_axis = np.array(
                parse_vector3(lens_axis_text, "_lens_axis")
            )
        else:
            lens_xyz = np.array([0.0, 0.0, camera_offset])
            lens_axis = np.array([0.0, 0.0, 1.0])
    except ValueError as error:
        rospy.logerr(
            "Invalid lens parameter: %s",
            str(error)
        )

        moveit_commander.roscpp_shutdown()
        return 1

    # Phone-holder mesh, attached to the flange as collision
    # geometry. An empty value skips the attachment (only sensible
    # while the Franka gripper is still mounted instead).
    default_holder_mesh = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        HOLDER_MESH_FILE
    )

    holder_mesh = rospy.get_param(
        "~holder_mesh",
        default_holder_mesh
    )

    holder_z_offset = rospy.get_param(
        "~holder_z_offset",
        HOLDER_Z_OFFSET
    )

    holder_yaw_deg = rospy.get_param(
        "~holder_yaw_deg",
        HOLDER_YAW_DEG
    )

    output_file = rospy.get_param(
        "~output_file",
        ""
    )

    if not output_file:
        # By default the poses are saved next to this script.
        output_file = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "arc_poses_{}.csv".format(
                datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            )
        )

    rospy.loginfo("Joint file: %s", joint_file)
    rospy.loginfo("Execution enabled: %s", execute_motion)
    rospy.loginfo("Radius: %.4f m", radius)
    rospy.loginfo("Arc: %.2f degrees", arc_degrees)
    rospy.loginfo(
        "Arc direction: toward %s",
        "-Y" if arc_direction < 0.0 else "+Y"
    )
    rospy.loginfo("Number of waypoints: %d", number_of_points)
    rospy.loginfo("Camera tracks the object: %s", track_object)
    if track_object:
        if free_roll:
            rospy.loginfo(
                "Image roll: free (step %.0f deg, up to +/-%.0f "
                "deg about the camera axis)",
                roll_step_deg,
                roll_max_deg
            )
        else:
            rospy.loginfo(
                "Image roll: fixed (zero roll at every waypoint)"
            )
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
        "Background screen: %s (bottom %.3f m above ground, "
        "yaw %.1f deg%s, scale %.3f)",
        screen_mesh if screen_mesh else "disabled",
        screen_height,
        screen_yaw_deg,
        " - auto from arc direction"
        if isinstance(screen_yaw_param, str)
        and screen_yaw_param.strip().lower() == "auto" else "",
        screen_scale
    )
    rospy.loginfo(
        "Lens in the flange frame: position (%.4f, %.4f, %.4f) m, "
        "camera axis (%.3f, %.3f, %.3f)",
        lens_xyz[0], lens_xyz[1], lens_xyz[2],
        lens_axis[0], lens_axis[1], lens_axis[2]
    )
    rospy.loginfo(
        "Phone holder mesh: %s (z offset %.4f m, yaw %.1f deg)",
        holder_mesh if holder_mesh else "disabled",
        holder_z_offset,
        holder_yaw_deg
    )
    rospy.loginfo(
        "Output file for joint values and poses: %s",
        output_file
    )
    if confirm_each_pose:
        rospy.loginfo(
            "Waypoint advance: manual - Enter required at every "
            "pose."
        )
    else:
        rospy.loginfo(
            "Wait between waypoints: %.2f seconds",
            wait_between_points
        )
    rospy.loginfo(
        "Return to the initial pose after the arc: %s",
        return_to_start
    )

    if arc_degrees > 90.0:
        rospy.logwarn(
            "Arc of %.1f degrees goes below the object's height: "
            "the hand will work close to the support rod on the "
            "final waypoints (intended for bottom views - MoveIt "
            "will reject any waypoint that would touch the rod).",
            arc_degrees
        )

    if radius <= 0.0:
        rospy.logerr(
            "The radius must be greater than zero, got %.4f m.",
            radius
        )

        moveit_commander.roscpp_shutdown()
        return 1

    if free_roll and roll_step_deg <= 0.0:
        rospy.logerr(
            "_roll_step_deg must be greater than zero with "
            "_free_roll:=true, got %.2f.",
            roll_step_deg
        )

        moveit_commander.roscpp_shutdown()
        return 1

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

    # Attach the phone holder before anything is planned - even the
    # initial joint move must respect its geometry. Done in dry-run
    # mode too, so RViz shows the attached mesh and the alignment
    # can be checked against the real holder before executing.
    if holder_mesh:
        try:
            attach_phone_holder(
                scene,
                holder_mesh,
                holder_z_offset,
                holder_yaw_deg
            )
        except Exception as error:
            rospy.logerr(
                "Unable to attach the phone holder: %s. Planning "
                "would ignore the holder geometry - fix "
                "_holder_mesh, or pass _holder_mesh:='' to skip "
                "the attachment deliberately.",
                str(error)
            )

            moveit_commander.roscpp_shutdown()
            return 1
    else:
        rospy.logwarn(
            "Phone holder attachment disabled (_holder_mesh:='')."
        )

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

    # Obstacles left over from a previous run persist in the
    # move_group planning scene across script restarts - remove
    # them before planning the move to the initial pose.
    scene.remove_world_object(KEEPOUT_OBJECT_NAME)
    scene.remove_world_object(ROD_OBJECT_NAME)
    scene.remove_world_object(SCREEN_OBJECT_NAME)
    rospy.sleep(0.5)

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
    log_current_pose(
        log_handle, log_writer, move_group, 0, 0.0, lens_xyz
    )

    lens_start, camera_axis_world, center_np = (
        compute_camera_geometry(
            start_pose,
            lens_xyz,
            lens_axis,
            radius
        )
    )

    sphere_center = Point(
        center_np[0],
        center_np[1],
        center_np[2]
    )

    rospy.loginfo(
        "Lens starts at x=%.6f, y=%.6f, z=%.6f looking along "
        "(%.3f, %.3f, %.3f)",
        lens_start[0], lens_start[1], lens_start[2],
        camera_axis_world[0],
        camera_axis_world[1],
        camera_axis_world[2]
    )

    rospy.loginfo(
        "Object (sphere center): x=%.6f, y=%.6f, z=%.6f - "
        "lens-object distance %.4f m",
        sphere_center.x,
        sphere_center.y,
        sphere_center.z,
        radius
    )

    # The arc math assumes the camera starts looking straight down
    # at the object (the start is the top of the sphere). With the
    # 45-degree phone cradle this constrains the start joints: the
    # flange must be pitched so the camera axis is vertical.
    aim_error_deg = math.degrees(
        math.acos(max(-1.0, min(1.0, -camera_axis_world[2])))
    )

    if aim_error_deg > 5.0:
        rospy.logwarn(
            "The camera axis is %.1f degrees away from straight "
            "down at the start pose. The object is placed along "
            "the camera axis (still centered in frame), but the "
            "arc no longer starts at the top of the sphere - "
            "adjust the start joints so the camera looks down.",
            aim_error_deg
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

    if screen_mesh:
        try:
            _, screen_top_z = add_background_screen(
                scene,
                move_group,
                sphere_center,
                object_height,
                screen_mesh,
                screen_scale,
                screen_height,
                screen_yaw_deg
            )
        except Exception as error:
            rospy.logerr(
                "Unable to add the background screen: %s. Planning "
                "would ignore the real screen - fix _screen_mesh, "
                "or pass _screen_mesh:='' to skip it deliberately.",
                str(error)
            )

            log_handle.close()
            moveit_commander.roscpp_shutdown()
            return 1

        if sphere_center.z + radius < screen_top_z:
            rospy.logwarn(
                "The screen rim (z=%.3f) is above the starting "
                "lens height (z=%.3f): the phone works inside the "
                "screen enclosure on the upper waypoints. MoveIt "
                "will veto any pose where the holder would touch "
                "the wall (11 cm from the rod).",
                screen_top_z,
                sphere_center.z + radius
            )
    else:
        rospy.logwarn(
            "Background screen disabled (_screen_mesh:='')."
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

    if rod_radius > 0.0 or object_radius > 0.0:
        try:
            if not ensure_state_clear(robot):
                rospy.logerr(
                    "The current state is still considered in "
                    "collision - planning will fail. Use "
                    "_object_radius:=0 / _rod_radius:=0 to disable "
                    "the obstacles."
                )
        except Exception as error:
            rospy.logerr(
                "Unable to verify the state validity: %s",
                str(error)
            )

    try:
        waypoints = create_arc_waypoints(
            start_pose=start_pose,
            center=center_np,
            lens_start=lens_start,
            arc_degrees=arc_degrees,
            number_of_points=number_of_points,
            track_object=track_object,
            direction=arc_direction
        )
    except Exception as error:
        rospy.logerr(
            "Error while generating the waypoints: %s",
            str(error)
        )

        moveit_commander.roscpp_shutdown()
        return 1

    total_points = len(waypoints)

    # Unconditional pause at the initial pose: the object (and its
    # rod) can now be placed under the lens - the exact target
    # position is in the "Object (sphere center)" log line above.
    wait_for_enter(
        "Robot at the initial pose, %d waypoints ready. Place the "
        "object at the logged sphere-center position (%.1f cm "
        "below the lens), clear the workspace, then confirm to "
        "start the arc." % (total_points, radius * 100.0)
    )

    last_roll = 0.0

    for index, spec in enumerate(waypoints, start=1):

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

        if track_object and free_roll:
            # Same lens position and aim for every candidate; only
            # the image roll differs, nearest the previous
            # waypoint's roll first.
            candidates = [
                (
                    roll,
                    make_flange_pose(
                        spec["lens_target"],
                        spec["camera_axis"],
                        spec["q_aim"],
                        roll,
                        lens_xyz
                    )
                )
                for roll in roll_candidates(
                    roll_step_deg,
                    roll_max_deg,
                    last_roll
                )
            ]
        else:
            candidates = [(0.0, spec["pose"])]

        reached_roll = move_to_pose(
            move_group=move_group,
            robot=robot,
            candidates=candidates,
            index=index,
            total_points=total_points
        )

        if reached_roll is None:
            rospy.logerr(
                "Trajectory stopped at waypoint %d. Poses recorded "
                "so far are kept in %s",
                index,
                output_file
            )

            log_handle.close()
            moveit_commander.roscpp_shutdown()
            return 1

        last_roll = reached_roll

        print_current_pose(
            move_group,
            "Waypoint {} reached".format(index)
        )

        log_current_pose(
            log_handle,
            log_writer,
            move_group,
            index,
            arc_degrees * float(index) / float(total_points),
            lens_xyz
        )

        if index < total_points:
            if confirm_each_pose:
                wait_for_enter(
                    "Waypoint %d/%d done - take the photo, then "
                    "confirm to move to the next pose." % (
                        index,
                        total_points
                    )
                )
            else:
                rospy.loginfo(
                    "Waiting %.2f seconds before the next "
                    "waypoint...",
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

    if return_to_start and not rospy.is_shutdown():

        # Final photo moment: the arm is still at the last
        # waypoint here.
        if confirm_each_pose:
            wait_for_enter(
                "Last waypoint done - take the final photo, then "
                "confirm to return to the initial pose."
            )
        else:
            rospy.loginfo(
                "Waiting %.2f seconds before returning to the "
                "initial pose...",
                wait_between_points
            )

            rospy.sleep(wait_between_points)

        rospy.loginfo(
            "Returning to the initial pose (the obstacles stay in "
            "the planning scene, so the plan avoids them)..."
        )

        if not move_to_joint_position(
                move_group,
                initial_joint_position):
            rospy.logerr(
                "Unable to plan a collision-free return to the "
                "initial pose - the arm stays at the last "
                "waypoint."
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
