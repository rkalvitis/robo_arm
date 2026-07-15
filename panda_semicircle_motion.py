#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Movimento ad arco per Franka Emika Panda.

Lo script:

1. legge una configurazione articolare iniziale da un file CSV;
2. sposta il robot nella configurazione iniziale;
3. legge la posa cartesiana raggiunta;
4. genera 8 waypoint nel piano Y-Z;
5. mantiene X e orientamento costanti;
6. si muove prima verso +Z e poi verso +Y;
7. attende 2 secondi tra un waypoint e il successivo.

Esempio joint_start.csv:

0.013337267496607689,0.13310515648967364,0.12014143002274653,-1.703551451575958,0.007806931131451653,1.0049740870915318,0.9248341724704006
"""

from __future__ import print_function

import copy
import math
import os
import sys

import numpy as np
import rospy
import moveit_commander


PLANNING_GROUP = "panda_arm"

RADIUS_METERS = 0.03
ARC_DEGREES = 60.0
NUMBER_OF_POINTS = 8

WAIT_BETWEEN_POINTS = 2.0
WAIT_AFTER_INITIAL_POSITION = 2.0

VELOCITY_SCALE = 0.05
ACCELERATION_SCALE = 0.05

JOINT_TOLERANCE = 0.002
POSITION_TOLERANCE = 0.002
ORIENTATION_TOLERANCE = 0.02

PLANNING_TIME = 10.0
PLANNING_ATTEMPTS = 10


def load_joint_position(file_path):
    """
    Legge esattamente 7 valori joint da un file CSV.
    """

    file_path = os.path.abspath(os.path.expanduser(file_path))

    if not os.path.isfile(file_path):
        raise IOError(
            "File delle joint non trovato: {}".format(file_path)
        )

    values = np.loadtxt(file_path, delimiter=",")
    values = np.asarray(values, dtype=float).reshape(-1)

    if values.size != 7:
        raise ValueError(
            "Il file deve contenere esattamente 7 valori joint. "
            "Valori trovati: {}".format(values.size)
        )

    if not np.all(np.isfinite(values)):
        raise ValueError(
            "Il file contiene valori non validi, NaN o infiniti."
        )

    return values.tolist()


def create_arc_waypoints(
        start_pose,
        radius,
        arc_degrees,
        number_of_points):
    """
    Genera un arco nel piano Y-Z.

    +Y = destra
    +Z = alto

    La traiettoria parte dalla posa corrente e inizialmente si muove
    soprattutto verso +Z. Successivamente si sposta anche verso +Y.

    Formula:

        delta_y = r * (1 - cos(theta))
        delta_z = r * sin(theta)

    theta varia da 0 a 60 gradi.

    L'orientamento dell'end-effector rimane invariato.
    """

    if radius <= 0.0:
        raise ValueError("Il raggio deve essere maggiore di zero.")

    if number_of_points <= 0:
        raise ValueError(
            "Il numero di waypoint deve essere maggiore di zero."
        )

    waypoints = []

    total_angle = math.radians(arc_degrees)

    rospy.loginfo(
        "Posa iniziale: x=%.6f, y=%.6f, z=%.6f",
        start_pose.position.x,
        start_pose.position.y,
        start_pose.position.z
    )

    for index in range(1, number_of_points + 1):

        theta = total_angle * float(index) / float(number_of_points)

        delta_y = radius * (1.0 - math.cos(theta))
        delta_z = radius * math.sin(theta)

        waypoint = copy.deepcopy(start_pose)

        waypoint.position.x = start_pose.position.x
        waypoint.position.y = start_pose.position.y + delta_y
        waypoint.position.z = start_pose.position.z + delta_z

        waypoints.append(waypoint)

        rospy.loginfo(
            "Waypoint %d/%d - angolo %.2f deg - "
            "delta Y %.6f m - delta Z %.6f m - "
            "posizione Y %.6f - posizione Z %.6f",
            index,
            number_of_points,
            math.degrees(theta),
            delta_y,
            delta_z,
            waypoint.position.y,
            waypoint.position.z
        )

    return waypoints


def move_to_joint_position(move_group, joint_position):
    """
    Muove il robot verso la configurazione articolare specificata.
    """

    rospy.loginfo("Movimento verso la posizione articolare iniziale...")

    move_group.set_joint_value_target(joint_position)

    success = move_group.go(wait=True)

    move_group.stop()
    move_group.clear_pose_targets()

    if not success:
        rospy.logerr(
            "Impossibile raggiungere la posizione articolare iniziale."
        )
        return False

    rospy.loginfo("Posizione articolare iniziale raggiunta.")

    return True


def move_to_pose(move_group, target_pose, index, total_points):
    """
    Pianifica ed esegue il movimento verso un singolo waypoint cartesiano.
    """

    rospy.loginfo(
        "Pianificazione waypoint %d/%d...",
        index,
        total_points
    )

    move_group.set_start_state_to_current_state()
    move_group.set_pose_target(target_pose)

    try:
        success = move_group.go(wait=True)
    except Exception as error:
        rospy.logerr(
            "Errore durante il movimento verso il waypoint %d: %s",
            index,
            str(error)
        )

        move_group.stop()
        move_group.clear_pose_targets()

        return False

    move_group.stop()
    move_group.clear_pose_targets()

    if not success:
        rospy.logerr(
            "Impossibile raggiungere il waypoint %d/%d.",
            index,
            total_points
        )
        return False

    rospy.loginfo(
        "Waypoint %d/%d raggiunto.",
        index,
        total_points
    )

    return True


def print_current_pose(move_group, label):
    """
    Stampa la posa cartesiana corrente.
    """

    pose = move_group.get_current_pose().pose

    rospy.loginfo(
        "%s - posizione: x=%.6f, y=%.6f, z=%.6f",
        label,
        pose.position.x,
        pose.position.y,
        pose.position.z
    )

    rospy.loginfo(
        "%s - orientamento: x=%.6f, y=%.6f, z=%.6f, w=%.6f",
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

    rospy.loginfo("File joint: %s", joint_file)
    rospy.loginfo("Esecuzione abilitata: %s", execute_motion)
    rospy.loginfo("Raggio: %.4f m", radius)
    rospy.loginfo("Arco: %.2f gradi", arc_degrees)
    rospy.loginfo("Numero waypoint: %d", number_of_points)
    rospy.loginfo(
        "Attesa tra i waypoint: %.2f secondi",
        wait_between_points
    )

    try:
        initial_joint_position = load_joint_position(joint_file)
    except Exception as error:
        rospy.logerr(
            "Errore nella lettura del file delle joint: %s",
            str(error)
        )

        moveit_commander.roscpp_shutdown()
        return 1

    rospy.loginfo(
        "Configurazione iniziale: %s",
        ["{:.9f}".format(value)
         for value in initial_joint_position]
    )

    try:
        move_group = moveit_commander.MoveGroupCommander(
            PLANNING_GROUP
        )
    except Exception as error:
        rospy.logerr(
            "Impossibile inizializzare MoveGroupCommander: %s",
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
            "Il planning group '%s' contiene %d joint invece di 7.",
            PLANNING_GROUP,
            len(current_joint_values)
        )

        moveit_commander.roscpp_shutdown()
        return 1

    if not execute_motion:
        rospy.logwarn(
            "Modalita di sicurezza attiva: il robot non verra mosso."
        )

        rospy.logwarn(
            "Per eseguire realmente il movimento aggiungere "
            "_execute:=true."
        )

        moveit_commander.roscpp_shutdown()
        return 0

    rospy.logwarn(
        "Controllare che l'area di lavoro sia libera e "
        "che il pulsante di arresto sia disponibile."
    )

    if not move_to_joint_position(
            move_group,
            initial_joint_position):
        moveit_commander.roscpp_shutdown()
        return 1

    rospy.loginfo(
        "Attesa di %.2f secondi nella posizione iniziale...",
        WAIT_AFTER_INITIAL_POSITION
    )

    rospy.sleep(WAIT_AFTER_INITIAL_POSITION)

    print_current_pose(
        move_group,
        "Posa iniziale raggiunta"
    )

    start_pose = copy.deepcopy(
        move_group.get_current_pose().pose
    )

    try:
        waypoints = create_arc_waypoints(
            start_pose=start_pose,
            radius=radius,
            arc_degrees=arc_degrees,
            number_of_points=number_of_points
        )
    except Exception as error:
        rospy.logerr(
            "Errore nella generazione dei waypoint: %s",
            str(error)
        )

        moveit_commander.roscpp_shutdown()
        return 1

    total_points = len(waypoints)

    for index, waypoint in enumerate(waypoints, start=1):

        if rospy.is_shutdown():
            rospy.logwarn(
                "ROS arrestato. Interruzione del movimento."
            )
            break

        rospy.loginfo(
            "Movimento verso waypoint %d/%d...",
            index,
            total_points
        )

        success = move_to_pose(
            move_group=move_group,
            target_pose=waypoint,
            index=index,
            total_points=total_points
        )

        if not success:
            rospy.logerr(
                "Traiettoria interrotta al waypoint %d.",
                index
            )

            moveit_commander.roscpp_shutdown()
            return 1

        print_current_pose(
            move_group,
            "Waypoint {} raggiunto".format(index)
        )

        if index < total_points:
            rospy.loginfo(
                "Attesa di %.2f secondi prima del prossimo waypoint...",
                wait_between_points
            )

            rospy.sleep(wait_between_points)

    rospy.loginfo(
        "Tutti i waypoint sono stati completati."
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
