#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from ar_track_alvar_msgs.msg import AlvarMarkers

tag = Point()

def getBox(msg):

	global tag
	if len(msg.markers) == 1:
		tag = msg.markers[0].pose.pose.position
		print(tag.z)
	else:
		tag = Point()

rospy.init_node ("tag_follower")

marker = rospy.Subscriber("/ar_pose_marker",AlvarMarkers, getBox)
move = rospy.Publisher("/husky_velocity_controller/cmd_vel",Twist, queue_size = 1 )

rate = rospy.Rate(10)
speed = Twist()

while not rospy.is_shutdown():

	speed.linear.x = (tag.x-1.5)*0.5
	speed.angular.z = tag.z*1.25

	print(speed)

	move.publish(speed)

	rate.sleep()
