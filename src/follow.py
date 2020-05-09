#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from ar_track_alvar_msgs.msg import AlvarMarkers



class TagClass:
	def __init__(self,ID,x,y,z):
		self.ID = ID
		self.x = x
		self.y = y
		self.z = z
		


def getBox(msg):
	global target
	global tag
	global marker_list
	marker_list = []
	readed_markers = []

	if len(msg.markers) >= 1:
		# tag = msg.markers[0].pose.pose.position
		for i in range(0,len(msg.markers)):
			if msg.markers[i].id not in readed_markers:
				print("%d bir tene var abey"%msg.markers[i].id)
				readed_markers.append(msg.markers[i].id)
				marker_list.append(TagClass(msg.markers[i].id,msg.markers[i].pose.pose.position.x,msg.markers[i].pose.pose.position.y,msg.markers[i].pose.pose.position.z))
			else:
				index_ = readed_markers.index(msg.markers[i].id)
				if marker_list[index_].ID == msg.markers[i].id:
					print("%d iki tene var abey"%msg.markers[i].id)
					print("%.2f + %.2f "%(marker_list[index_].x, msg.markers[i].pose.pose.position.x))
					marker_list[index_].x = (marker_list[index_].x + msg.markers[i].pose.pose.position.x)/2
					marker_list[index_].y = (marker_list[index_].x + msg.markers[i].pose.pose.position.y)/2
					marker_list[index_].z = (marker_list[index_].x + msg.markers[i].pose.pose.position.z)/2
					print("Son durum %.2f "%(marker_list[index_].x))
			
			if 1 in readed_markers:
				target_index = readed_markers.index(1)
				tag.x = marker_list[target_index].x
				tag.y = marker_list[target_index].y
				tag.z = marker_list[target_index].z
			else: 
				tag = Point()
	else:
		tag.x = 0
		tag.y = 0
		tag.z = 0.5
	for every_marker in marker_list:
		print("%d + %.2f "%(every_marker.ID, every_marker.x))







if __name__ == '__main__':

	rospy.init_node ("tag_follower")
	marker = rospy.Subscriber("/ar_pose_marker",AlvarMarkers, getBox)
	move = rospy.Publisher("/husky_velocity_controller/cmd_vel",Twist, queue_size = 1 )
	target = -1
	marker_list = []
	rate = rospy.Rate(10)
	speed = Twist()
	tag = Point()

	while not rospy.is_shutdown():
		print("------------")
		speed.linear.x = (tag.x-1.5)*0.05
		speed.angular.z = tag.z*0.5

		if (tag.x-1.5) <= 0.2 and (tag.x-1.5)>=-0.2:
			target = -1
			print("Target is spotted")

		move.publish(speed)
		rate.sleep()




