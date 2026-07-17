#! /home/leon/anaconda3/envs/cecilia/bin/python


# ROS node to start the service for controlling the robot
# Author: Hongtao Wu
# Date: Mar 05

import rospy
from panda_moveit_ctrl.panda_robot_service import PandaRobotService

if __name__ == "__main__":
    rospy.init_node("panda_robot_service")

    vel = 0.4
    acc = 0.4
    # The custom phone holder replaced the Franka gripper: no
    # gripper interface to load, and the flange is the end-effector
    # frame (franka_control.launch must run with
    # load_gripper:=false as well).
    load_gripper = False
    ee = "panda_link8"
    PRS = PandaRobotService(vel, acc, ee=ee, load_gripper=load_gripper)

    
    rospy.spin()