#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

## Initiating robot commander to get robot information like kinematic model and joint states

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
right_arm=moveit_commander.MoveGroupCommander("right_arm")
right_hand=moveit_commander.MoveGroupCommander("right_hand")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

group_names = robot.get_group_names()
print("============ Available Planning Groups:", group_names)

#getRPY(right_arm)

# right_arm   right_hand

right_arm.set_named_target("non_sing")
right_arm.go()

#pose_target = geometry_msgs.msg.Pose()
#pose_target.orientation.w = 1.0
#pose_target.position.x = 0.47
#pose_target.position.y = 0.14
#pose_target.position.z = 1.08
#right_arm.set_pose_target(pose_target)

#rospy.loginfo("Planning Trajectory to Pose 1")
#plan1 = right_arm.plan()
#rospy.loginfo("DONE Planning Trajectory to Pose 1")
#ospy.loginfo("Executing Trajectory to Pose 1")
#right_arm.go(wait=True)
#rospy.loginfo("Done Executing Trajectory to Pose 1")

stand_pose=right_arm.get_current_pose().pose
print "STAND POSE: ", stand_pose

orientation_list = [stand_pose.orientation.x, stand_pose.orientation.y, stand_pose.orientation.z, stand_pose.orientation.w] 
(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
print (orientation_list)
print (roll, pitch, yaw)


pose_target = geometry_msgs.msg.Pose()
#roll=-1.57
pitch=-0.08
#yaw=0.0
#pose_goal.orientation = stand_pose.orientation
pose_target.position.x = 0.25
pose_target.position.y = 0.14
pose_target.position.z = 1.10
quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
pose_target.orientation.x = quaternion[0]
pose_target.orientation.y = quaternion[1]
pose_target.orientation.z = quaternion[2]
pose_target.orientation.w = quaternion[3]
new_orientation_list = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w] 
right_arm.set_pose_target(pose_target)
print (new_orientation_list)

rospy.loginfo("Planning Trajectory to Pose 2")
plan1 = right_arm.plan()
rospy.loginfo("DONE Planning Trajectory to Pose 2")
rospy.loginfo("Executing Trajectory to Pose 2")
right_arm.go(wait=True)
rospy.loginfo("Done Executing Trajectory to Pose 2")

rospy.sleep(2)


joint_goal = right_arm.get_current_joint_values()
joint_goal[5] = -0.06
right_arm.go(joint_goal, wait=True)
#joint_goal[2] = -1.2
#joint_goal[3] = 0
#plan1 = right_arm.plan()
#right_arm.go(wait=True)

#right_arm.set_named_target("stand")
#right_arm.go()

near_pose=right_arm.get_current_pose().pose
#print "NEAR POSE: ", near_pose


pose_goal= geometry_msgs.msg.Pose()
pose_goal.orientation = near_pose.orientation
pose_goal.position.x = 0.34
pose_goal.position.y = 0.14
pose_goal.position.z = 1.10
right_arm.set_pose_target(pose_goal)
right_arm.go(wait=True)

pose_goal= geometry_msgs.msg.Pose()
pose_goal.orientation = near_pose.orientation
pose_goal.position.x = 0.25
pose_goal.position.y = 0.14
pose_goal.position.z = 1.10
right_arm.set_pose_target(pose_goal)
right_arm.go(wait=True)


pose_goal= geometry_msgs.msg.Pose()
pose_goal.orientation = near_pose.orientation
pose_goal.position.x = 0.0
pose_goal.position.y = 0.45
pose_goal.position.z = 1.10
right_arm.set_pose_target(pose_goal)
right_arm.go(wait=True)
