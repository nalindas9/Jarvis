#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import intera_interface
import utils
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


# Global variables
action = "None"

def sub_callback(msg):
	global action
	if msg.data == "pick":
		action = "pick"
	elif msg.data == "pour":
		action = "pour"
	elif msg.data == "place":
		action = "place" 
        
def main():
	# Initialize moveit commandder and rospy node
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_group_python_interface_tutorial',
		              anonymous=True)
    # ROS Subscriber to actions topic from BT
	rospy.Subscriber("action", String, sub_callback)
	# Instantiate robot commmander object
	robot = moveit_commander.RobotCommander()
	# Instantiate planning scene interface object
	scene = moveit_commander.PlanningSceneInterface()


	group_name = "right_arm"
	#Instantiate move group commander object with specified group name
	group = moveit_commander.MoveGroupCommander(group_name)

	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
		                                             moveit_msgs.msg.DisplayTrajectory,
		                                             queue_size=20)
	
	group.set_goal_position_tolerance(0.001);
	group.set_goal_orientation_tolerance(0.001)
	group.set_planning_time(10.0);
	group.set_num_planning_attempts(4);
	# We can get the name of the reference frame for this robot:
	planning_frame = group.get_planning_frame()
	print "============ Reference frame: %s" % planning_frame

	# We can also print the name of the end-effector link for this group:
	eef_link = group.get_end_effector_link()
	print "============ End effector: %s" % eef_link

	# We can get a list of all the groups in the robot:
	group_names = robot.get_group_names()
	print "============ Robot Groups:", robot.get_group_names()

	# Sometimes for debugging it is useful to print the entire state of the
	# robot:
	print "============ Printing robot state"
	print robot.get_current_state()
	print ""

	gripper = intera_interface.Gripper()
	
	rate = rospy.Rate(10) # 10 hz
	picked, poured, placed = False, False, False
	while 1:
		if action == "pick" and picked == False:
			rospy.loginfo("Action pick recieved!")
			picked, plan = utils.pick(group, gripper)
		elif action == "pour" and poured == False:
			rospy.loginfo("Action pour recieved!")
		elif action == "place" and placed == False:
			rospy.loginfo("Action place recieved!")
		rate.sleep()

	"""
	# Specify the pose goal here
	pose_goal = geometry_msgs.msg.Pose()
	pose_goal.orientation.w = 0
	pose_goal.orientation.x = 1
	pose_goal.orientation.y = 0
	pose_goal.orientation.z = 0
	pose_goal.position.x = 0.5
	pose_goal.position.y = -0.4
	pose_goal.position.z = 0.3
	group.set_pose_target(pose_goal)
	gripper.close()
	plan = group.go(wait=True)
	# Calling `stop()` ensures that there is no residual movement
	group.stop()
	# It is always good to clear your targets after planning with poses.
	# Note: there is no equivalent function for clear_joint_value_targets()
	group.clear_pose_targets()
	print('Executed Plan 1 Succesfully!')
	rospy.sleep(2)
	
	# Specify the pose goal here
	pose_goal1 = geometry_msgs.msg.Pose()
	pose_goal1.orientation.w = 0
	pose_goal1.orientation.x = 1
	pose_goal1.orientation.y = 0
	pose_goal1.orientation.z = 0
	pose_goal1.position.x = 0.6
	pose_goal1.position.y = -0.29
	pose_goal1.position.z = -0.03
	group.set_pose_target(pose_goal1)
	plan = group.go(wait=True)
	# Calling `stop()` ensures that there is no residual movement
	group.stop()
	# It is always good to clear your targets after planning with poses.
	# Note: there is no equivalent function for clear_joint_value_targets()
	group.clear_pose_targets()
	print('Executed Plan 2 Succesfully!')
    """
    

	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(plan)
	# Publish
	display_trajectory_publisher.publish(display_trajectory);
	print('Planning ended successfully!')
	rospy.spin()

if __name__ == '__main__':
	main()

