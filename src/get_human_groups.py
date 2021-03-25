#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
#from math import pi
from std_msgs.msg import String
#from moveit_commander.conversions import pose_to_list


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)


## Initiating robot commander to get robot information like kinematic model and joint states

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

#'arm_left', 'arm_left_torso', 'arm_right', 'arm_right_torso', 'both_arms_torso', 'torso'

#arm_left=moveit_commander.MoveGroupCommander("arm_left")
#arm_left_torso=moveit_commander.MoveGroupCommander("arm_left_torso")
#arm_right=moveit_commander.MoveGroupCommander("arm_right")
#arm_right_torso=moveit_commander.MoveGroupCommander("arm_right_torso")
#both_arms_torso=moveit_commander.MoveGroupCommander("both_arms_torso")
#torso=moveit_commander.MoveGroupCommander("torso")


display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

group_names = robot.get_group_names()
print("============ Available Planning Groups:", group_names)

# right_arm   right_hand

right_arm=moveit_commander.MoveGroupCommander("right_arm")
right_hand=moveit_commander.MoveGroupCommander("right_hand")

#cartesian = rospy.get_param('~cartesian', False)

# Allow replanning to increase the odds of a solution
#right_arm.allow_replanning(True)#

# Set the right arm reference frame
#right_arm.set_pose_reference_frame('right_foot')
# Set the right hand reference frame
#right_hand.set_pose_reference_frame('right_foot')

#right_arm.set_planner_id('RRTConnectkConfigDefault')

# Allow some leeway in position(meters) and orientation (radians)
#right_arm.set_goal_position_tolerance(0.0)
#right_arm.set_goal_orientation_tolerance(0.0) 

#eef_link = right_hand.get_end_effector_link()
#print("============ End effector link: %s" % eef_link)

#gripper_rhand=moveit_commander.MoveGroupCommander.set_end_effector_link("right_wrist")

#right_hand.get_current_pose()

# ****************** Pre defined Positions *********************************
#right_arm.set_named_target("elbow_bend")
#plan1 = right_arm.go()

#right_hand.set_named_target("grip_close")
#plan2 = right_hand.go()

#right_arm.set_named_target("sideways")
#plan1 = right_arm.go()

#right_arm.set_named_target("elbow_bend")
#plan1 = right_arm.go()

#right_hand.set_named_target("grip_open")
#plan2 = right_hand.go()

#right_arm.set_named_target("stand")
#plan1 = right_arm.go()

# ****************** Pre defined Positions *********************************


#pose_goal=geometry_msgs.msg.Pose()
#print(pose_goal)
#print("")

#pose_goal=geometry_msgs.msg.Pose()
#pose_goal.position.z = 1.2
#right_arm.go(pose_goal,wait=True)

#current_pose = right_arm.get_current_pose().pose
#print(current_pose)


#print("============ Printing right_arm joints")
#print(right_arm.get_joints())
#print("")

#print "============ Printing robot state"
#print robot.get_current_state()
#print ""

#joint_goal = right_arm.get_current_joint_values()
#joint_goal[0] = 0
#joint_goal[1] = 1.2
#joint_goal[2] = -1.2

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
#right_arm.go(joint_goal, wait=True)


#current_pose2 = right_arm.get_current_pose().pose
#print(current_pose2)
# Calling ``stop()`` ensures that there is no residual movement
#right_arm.stop()

#right_arm.set_named_target("non_sing_pose")
#plan1 = right_arm.go()

##Planning to a Pose Goal
#pose_goal=geometry_msgs.msg.Pose()
#pose_goal.orientation.w = 0.5
#pose_goal.position.x = 0.27
#pose_goal.position.y = 0.15
#pose_goal.position.z = 1.08
#right_arm.set_pose_target(pose_goal)
#right_arm.set_planning_time(10)
#right_arm.go(wait=True)
#right_arm.stop()
#right_arm.clear_pose_targets()

right_arm.set_named_target("elbow_bend")
#plan1 = right_arm.go()

##Compute the plan and execute it
#plan1=right_arm.plan()
#right_arm.go(wait=True)

#pose_target= geometry_msgs.msg.Pose()
#pose_target.orientation.w = 0.5
#pose_target.orientation.x = -0.15
#pose_target.orientation.y = -0.10
#pose_target.orientation.z = -0.55
#pose_target.position.x = 0.3
#pose_target.position.y = 0.34
#pose_target.position.z = 1.25
#right_arm.set_pose_target(pose_target)
#print(new_pose)
#right_arm.set_planning_time(10)

#plan1 = right_arm.plan()
#right_arm.go(wait=True)


#right_arm.stop()
#right_arm.clear_pose_targets()

#right_arm.set_named_target("stand")
#plan1 = right_arm.go()


#planning_frame_arm_left= arm_left.get_planning_frame()
#print "============ Planning frame: %s" % planning_frame_arm_left

#print("============ Printing arm_left joints")
#print(arm_left.get_joints())
#print("")

#planning_frame_arm_left_torso= arm_left_torso.get_planning_frame()
#print "============ Planning frame: %s" % planning_frame_arm_left_torso

#print("============ Printing arm_left_torso joints")
#print(arm_left_torso.get_joints())
#print("")


#planning_frame_arm_right= arm_right.get_planning_frame()
#print "============ Planning frame: %s" % planning_frame_arm_right

#print("============ Printing arm_right joints")
#print(arm_right.get_joints())
#print("")

#planning_frame_arm_right_torso= arm_right_torso.get_planning_frame()
#print "============ Planning frame: %s" % planning_frame_arm_right_torso

#print("============ Printing arm_right_torso joints")
#print(arm_right_torso.get_joints())
#print("")

#planning_frame_both_arms_torso= both_arms_torso.get_planning_frame()
#print "============ Planning frame: %s" % planning_frame_both_arms_torso

#print("============ Printing both_arms_torso joints")
#print(both_arms_torso.get_joints())
#print("")

#planning_frame_torso= torso.get_planning_frame()
#print "============ Planning frame: %s" % planning_frame_torso

#print("============ Printing torso joints")
#print(torso.get_joints())
#print("")


