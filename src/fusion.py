 #!/usr/bin/env python
import rospy
import tf
import actionlib
import time

import math
import numpy as np

from geometry_msgs.msg import Twist, Point, Quaternion  
from ar_track_alvar_msgs.msg import AlvarMarkers
from datmo.msg import TrackArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan

from tf import TransformListener


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

class TagMatcher:

    def __init__ (self):
        self.tf = TransformListener()
        rospy.loginfo("TagMatcher Initalized!!!!!!")

	def getMessages(self):
		self.L_msg = rospy.wait_for_message("/datmo/box_kf",TrackArray,timeout=None)
		self.T_msg = rospy.wait_for_message("/ar_pose_marker",AlvarMarkers,timeout=None)

	def Fusion(lidar_message,tag_message):
		# Rotate Tag
		
		# 2 /map
		# read L_msg
		# compare

	def getTagMessage(self,msg):
		if len(msg.markers) >= 1:
			for i in range(0,len(msg.markers)):
				

    # def getBoxfromLidar(self,msg):
    #     for i in range(0,len(msg.tracks)):
    #        if  msg.tracks[i].odom.pose.pose.position.x > 4.0 and msg.tracks[i].odom.pose.pose.position.x < 6.0:
    #            print msg.tracks[i]

	def Rotation(self,translation,orientation):
		self.rotation_matrix =  np.array([1,0,0,0,0,-1,0,1,0]).reshape(3,3) #-90 degree about the x axis
		self.converted_trans = np.dot(self.rotation_matrix,np.asarray(translation).reshape(3,1))
		self.converted_euler = np.dot(self.rotation_matrix,np.asarray(tf.transformations.euler_from_quaternion(orientation)).reshape(3,1))
		self.converted_quaternion = tf.transformations.quaternion_from_euler(self.converted_euler[0], self.converted_euler[1],self.converted_euler[2])
		return self.converted_trans, self.converted_quaternion

	def moveARtag2Base(self,tagframe):
		return np.dot(self.getTagFrame(tagframe)[2],self.getHuskyPose()[2])

    def getHuskyPose(self):	
		try:
			(trans,rot) = self.tf.lookupTransform('/map', '/base_footprint', rospy.Time(0))
			homo_matrix = tf.transformations.quaternion_matrix(rot)
			homo_matrix[0][3] = trans[0]
			homo_matrix[1][3] = trans[1]
			homo_matrix[2][3] = trans[2]
			return (trans,rot,homo_matrix)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print("Error Occured! - Detection of Husky Pose wrt map frame")		

    def getTagFrame(self,tagFrame):	
		try:
			(trans,rot) = self.tf.lookupTransform('/base_footprint', tagFrame, rospy.Time(0))
			new_trans, new_rot = self.Rotation(trans,rot)
			homo_matrix = tf.transformations.quaternion_matrix(new_rot)
			homo_matrix[0][3] = new_trans[0]
			homo_matrix[1][3] = new_trans[1]
			homo_matrix[2][3] = new_trans[2]
			return (new_trans,new_rot,homo_matrix)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print("Error Occured! - Detection of Tag Pose wrt base_footprint frame")	


if __name__ == '__main__':
    rospy.init_node("fusion")
    matcher = TagMatcher()
 
