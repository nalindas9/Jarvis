import rospy
import geometry_msgs.msg

# Function for pick action
def pick(group, gripper):
	p1 = geometry_msgs.msg.Pose()
	p1.orientation.w = 0.472527;
	p1.orientation.x = -0.513829;
	p1.orientation.y = -0.5175;
	p1.orientation.z = -0.494866;
	p1.position.x = -0.0236007;
	p1.position.y = 0.906033;
	p1.position.z = 0.168587;
	group.set_pose_target(p1)
	plan = group.go(wait=True)
	gripper.close()
	group.stop()
	group.clear_pose_targets()
	
	rospy.sleep(1)
	
	p2 = geometry_msgs.msg.Pose()
	p2.orientation.w = group.get_current_pose().pose.orientation.w;
	p2.orientation.x = group.get_current_pose().pose.orientation.x;
	p2.orientation.y = group.get_current_pose().pose.orientation.y;
	p2.orientation.z = group.get_current_pose().pose.orientation.z;
	p2.position.x = group.get_current_pose().pose.position.x;
	p2.position.y = group.get_current_pose().pose.position.y;
	p2.position.z = group.get_current_pose().pose.position.z - 0.25;
	group.set_pose_target(p2)
	plan = group.go(wait=True)
	gripper.open()
	group.stop()
	group.clear_pose_targets()
	rospy.loginfo("Object picked up!")
	return True, plan
