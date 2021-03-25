#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

## Initiating robot commander to get robot information like kinematic model and joint states

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

#'arm_left', 'arm_left_torso', 'arm_right', 'arm_right_torso', 'both_arms_torso', 'torso'

arm_left=moveit_commander.MoveGroupCommander("arm_left")
arm_left_torso=moveit_commander.MoveGroupCommander("arm_left_torso")
arm_right=moveit_commander.MoveGroupCommander("arm_right")
arm_right_torso=moveit_commander.MoveGroupCommander("arm_right_torso")
both_arms_torso=moveit_commander.MoveGroupCommander("both_arms_torso")
torso=moveit_commander.MoveGroupCommander("torso")


display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

eef_link = arm_left.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

planning_frame_arm_left= arm_left.get_planning_frame()
print "============ Planning frame: %s" % planning_frame_arm_left

print("============ Printing arm_left joints")
print(arm_left.get_joints())
print("")

planning_frame_arm_left_torso= arm_left_torso.get_planning_frame()
print "============ Planning frame: %s" % planning_frame_arm_left_torso

print("============ Printing arm_left_torso joints")
print(arm_left_torso.get_joints())
print("")


planning_frame_arm_right= arm_right.get_planning_frame()
print "============ Planning frame: %s" % planning_frame_arm_right

print("============ Printing arm_right joints")
print(arm_right.get_joints())
print("")

planning_frame_arm_right_torso= arm_right_torso.get_planning_frame()
print "============ Planning frame: %s" % planning_frame_arm_right_torso

print("============ Printing arm_right_torso joints")
print(arm_right_torso.get_joints())
print("")

planning_frame_both_arms_torso= both_arms_torso.get_planning_frame()
print "============ Planning frame: %s" % planning_frame_both_arms_torso

print("============ Printing both_arms_torso joints")
print(both_arms_torso.get_joints())
print("")

planning_frame_torso= torso.get_planning_frame()
print "============ Planning frame: %s" % planning_frame_torso

print("============ Printing torso joints")
print(torso.get_joints())
print("")


