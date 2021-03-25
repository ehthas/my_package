#!/usr/bin/env python

import sys
import copy
import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from control_msgs.msg import GripperCommand

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


#right_arm.set_goal_position_tolerance(0.1)
#right_arm.set_goal_orientation_tolerance(0.01)

planning_frame_right_arm= right_arm.get_planning_frame()
print "============ Planning frame: %s" % planning_frame_right_arm

planning_frame_right_hand= right_hand.get_planning_frame()
print "============ Planning frame: %s" % planning_frame_right_hand


right_arm.set_end_effector_link("right_wrist")
end_effector_link = right_arm.get_end_effector_link()

reference_frame="world"
right_arm.set_pose_reference_frame(reference_frame)

right_arm.allow_replanning(True)

right_arm.set_goal_position_tolerance(0.01)
right_arm.set_goal_orientation_tolerance(0.1)

# Start the arm in the "resting" pose stored in the SRDF file
#right_arm.set_named_target("stand")

#right_arm.go()

#rospy.sleep(2)

#oldpose=right_arm.get_current_pose().pose
#print "OLD POSE: ", oldpose

#right_arm.set_named_target("non_sing")

#right_arm.go()

#rospy.sleep(2)

#target_pose = geometry_msgs.msg.Pose()

#target_pose.header.frame_id = "reference_frame"

#target_pose.header.stamp = rospy.Time.now()

#target_pose.position.x = 0.30

#target_pose.position.y = 0.14

#target_pose.position.z = 1.07

#target_pose.orientation = oldpose.orientation

#target_pose.orientation.x = -0.5

#target_pose.orientation.y = -0.5

#target_pose.orientation.z = -0.49

#target_pose.orientation.w = -0.49 


# Set the target pose for the end-effector

#target_pose = geometry_msgs.msg.Pose()

#target_pose.header.frame_id = "reference_frame"

#target_pose.header.stamp = rospy.Time.now()

#target_pose.position.x = 0.05

#target_pose.position.y = 0.09

#target_pose.position.z = 0.86

#target_pose.orientation.x = -0.5

#target_pose.orientation.y = -0.5

#target_pose.orientation.z = 0.49

#target_pose.orientation.w = 0.49 

# Set the start state to the current state

#right_arm.set_start_state_to_current_state()

# Set the goal pose of the end effector to the stored pose 60
#right_arm.set_pose_target(target_pose, end_effector_link)
#right_arm.set_pose_target(target_pose)

# Plan a trajectory to the target pose

#traj = right_arm.plan()
# Execute the planned trajectory

#right_arm.execute(traj)

#end_effector_pose=right_arm.get_current_pose(end_effector_link)
#print (end_effector_pose)
#pose1=right_arm.get_current_pose()
#print (pose1)

# Pause for a second
#rospy.sleep(1)


#gripper = robot.get_group("right_arm")
#print(gripper.get_end_effector_link())
#gripper.set_end_effector_link("right_wrist")
#print(gripper.get_end_effector_link())

#gripper_pose=end_effector_link.get_current_pose().pose
#print "GRIPPER POSE: ", gripper_pose

#eef_link = "hand"
#eef_link1 = right_hand.set_end_effector_link(right_wrist)
#eef_link1 = right_hand.get_end_effector_link()
#print("============ End effector link: %s" % eef_link1)

#gripper_rhand=moveit_commander.MoveGroupCommander.set_end_effector_link("right_wrist")

#end_effector_pose=move_group.get_end_effector_link_pose().pose
#print "END_EFFECTOR_POSE: ", end_effector_pose


# ****************** Pre defined Positions *********************************
#right_arm.set_named_target("elbow_bend")
#plan1 = right_arm.go()

#right_hand.set_named_target("grip_close")
#plan2 = right_hand.go()

#right_arm.set_named_target("sideways")
#plan1 = right_arm.go()

#right_arm.set_named_target("non_sing")
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

#current_pose = robot.get_current_pose().pose
#print(current_pose)
#print("")


#print("============ Printing right_arm joints")
#print(right_arm.get_joints())
#print("")

#print "============ Printing robot state"
#print robot.get_current_state()
#print ""
# **********************************************************************
right_arm.set_named_target("non_sing")
plan1 = right_arm.go()

#eef_link.set_position_target(0.3,0.12,1.0) 

#right_arm.set_random_target()  
#, "right_hand");

#joint_goal = right_arm.get_current_joint_values()
#joint_goal[0] = 0
#joint_goal[1] = 1.2
#joint_goal[2] = -1.2
#joint_goal[3] = 0

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
#right_arm.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
#right_arm.stop()

#right_arm.set_named_target("non_sing")
#plan1 = right_arm.go()

#oldpose=right_arm.get_current_pose().pose
#print "OLD POSE: ", oldpose

#pose_goal=geometry_msgs.msg.Pose()
#pose_goal.orientation = oldpose.orientation
#pose_goal.position.x = oldpose.position.x + 0.20
#pose_goal.position.y = oldpose.position.y + 0.05
#pose_goal.position.z = oldpose.position.z + 0.04


##Planning to a Pose Goal
#pose_goal=geometry_msgs.msg.Pose()
#pose_goal.position.x = 0.47
#pose_goal.position.y = 0.14
#pose_goal.position.z = 1.12
#pose_goal.orientation.x = 0.0
#pose_goal.orientation.y = 0.5
#pose_goal.orientation.z = 0.0
#pose_goal.orientation.w = 0.5


#pose_goal= geometry_msgs.msg.Pose()
#pose_goal.orientation = oldpose.orientation
#pose_goal.orientation.w = 0.81
#pose_goal.orientation.x = -0.15
#pose_goal.orientation.y = -0.10
#pose_goal.orientation.z = -0.55
#pose_goal.position.x = 0.05
#pose_goal.position.y = 0.34
#pose_goal.position.z = 1.25
#pose_goal.orientation.x = -0.15
#pose_goal.orientation.y = -0.10
#pose_goal.orientation.z = -0.55
#pose_goal.orientation.w = 0.81


#right_arm.set_pose_target(pose_goal)
#right_arm.set_planning_time(10)
#right_arm.go(wait=True)
#right_arm.stop()
#right_arm.clear_pose_targets()

#newpose=right_arm.get_current_pose().pose
#print "NEW POSE: ", newpose

#right_arm.set_named_target("stand")
#plan1 = right_arm.go()

# *************************************************************

pose_target= geometry_msgs.msg.Pose()
pose_target.position.x = 0.25
pose_target.position.y = 0.09
pose_target.position.z = 1.08
#pose_target.orientation.x = 0
#pose_target.orientation.y = 0
#pose_target.orientation.z = 0
pose_target.orientation.w = 0.5
right_arm.set_pose_target(pose_target)
#print(new_pose)
#right_arm.set_planning_time(10)

plan1 = right_arm.plan()
right_arm.go(wait=True)


right_arm.set_named_target("stand")
plan1 = right_arm.go()

##Compute the plan and execute it
#plan1=right_arm.plan()
#right_arm.go(wait=True)

#pose_goal = geometry_msgs.msg.Pose()
#pose_goal.orientation.w = 1.0
#pose_target.orientation.x = -0.5
#pose_target.orientation.y = 0.5
#pose_target.orientation.z = -0.5
#pose_goal.position.x = 0.27
#pose_goal.position.y = 0.09
#pose_goal.position.z = 1.08
#right_arm.set_pose_target(pose_goal)
#right_arm.set_planning_time(10)
#plan=right_arm.go(wait=True)
#right_arm.stop()
#right_arm.clear_pose_targets()

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


