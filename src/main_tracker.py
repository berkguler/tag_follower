 #!/usr/bin/env python

import rospy
import tf
import actionlib
import time

from geometry_msgs.msg import Twist, Point, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from teleop import process as teleop_process

goal_found = False

class TagClass:
	def __init__(self,ID,x,y,z,r,p,yaw,w):
		self.ID = ID
		self.x = x
		self.y = y
		self.z = z
		self.r = r
		self.p = p
		self.yaw = yaw
		self.w = w
		self.item_list = [self.r, self.p, self.yaw, self.w]

goal = MoveBaseGoal()

def getBox(msg):
	global goal
	global goal_found
	global marker_list
	marker_list = []
	readed_markers = []
	goal = MoveBaseGoal()
	rospy.loginfo("ariyorum")
	if len(msg.markers) >= 1:
		# tag = msg.markers[0].pose.pose.position
		for i in range(0,len(msg.markers)):
			
			print("%d - %.2f - %.2f - %.2f // %.2f - %.2f - %.2f - %.2f"%(msg.markers[i].id,msg.markers[i].pose.pose.position.x,msg.markers[i].pose.pose.position.y,msg.markers[i].pose.pose.position.z,msg.markers[i].pose.pose.orientation.x,msg.markers[i].pose.pose.orientation.y,msg.markers[i].pose.pose.orientation.z,msg.markers[i].pose.pose.orientation.w))
			readed_markers.append(msg.markers[i].id)
			marker_list.append(TagClass(msg.markers[i].id,msg.markers[i].pose.pose.position.x,msg.markers[i].pose.pose.position.y,msg.markers[i].pose.pose.position.z,msg.markers[i].pose.pose.orientation.x,msg.markers[i].pose.pose.orientation.y,msg.markers[i].pose.pose.orientation.z,msg.markers[i].pose.pose.orientation.w))

			if 1 in readed_markers:
				goal_found = True
				target_index = readed_markers.index(1)
				goal.target_pose.header.frame_id = "map"
				goal.target_pose.header.stamp = rospy.Time.now()
				goal.target_pose.pose.position.x = marker_list[target_index].x
				goal.target_pose.pose.position.y = marker_list[target_index].y
				goal.target_pose.pose.position.z = marker_list[target_index].z
				goal.target_pose.pose.orientation.x = marker_list[target_index].yaw
				goal.target_pose.pose.orientation.y = -1*marker_list[target_index].p
				goal.target_pose.pose.orientation.z = marker_list[target_index].r
				goal.target_pose.pose.orientation.w = marker_list[target_index].x
				print(goal)
			else: 
				goal = MoveBaseGoal()
	else:
		goal = MoveBaseGoal()



def movebase_client(target_goal):
	# Create an action client called "move_base" with action definition file "MoveBaseAction"
	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	client.wait_for_server()
	
	client.send_goal(target_goal)
	print("Goal sended")
	wait = client.wait_for_result()

	
	if not wait:
		rospy.logerr("Action server not available!")
		rospy.signal_shutdown("Action server not available!")
	else:
		return client.get_result()


if __name__ == '__main__':
	try:
		rospy.init_node("tag_follower")
		msg = rospy.wait_for_message("/ar_pose_marker",AlvarMarkers,timeout=None)
		rospy.loginfo("bunu goruyon mu gotelek")
		teleop_process
		getBox(msg)
		if(goal_found):
			result = movebase_client(goal)
			
			if result:
				rospy.loginfo("Goal Execution Done!")
		else:
			rospy.loginfo("yarrak")
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation test finished.")
