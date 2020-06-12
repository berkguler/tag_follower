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



class TagMatcher:

	def __init__ (self):
		self.tf = TransformListener()
		rospy.loginfo("TagMatcher Initalized!!!!!!")

	def getMessages(self):
		L_msg = rospy.wait_for_message("/datmo/box_kf",TrackArray,timeout=None)
		T_msg = rospy.wait_for_message("/ar_pose_marker",AlvarMarkers,timeout=None)
		return L_msg, T_msg

	def Fusion(self):
		L_msg, T_msg = self.getMessages()
		if len(T_msg.markers) >= 1:
			for i in range(0,len(T_msg.markers)):
				for k in range(0, len(L_msg.tracks)):
					# Tag
					self.tag_frame_name = "/ar_marker_" + str(T_msg.markers[i].id)
					self.tag_trans, self.tag_orient, self.tag_matrix = self.getTagFrame(self.tag_frame_name)
					# Box
					box_x = L_msg.tracks[k].odom.pose.pose.position.x
					box_y = L_msg.tracks[k].odom.pose.pose.position.y
					box_orientation = (
						L_msg.tracks[k].odom.pose.pose.orientation.x,
						L_msg.tracks[k].odom.pose.pose.orientation.y,
						L_msg.tracks[k].odom.pose.pose.orientation.z,
						L_msg.tracks[k].odom.pose.pose.orientation.w
					)
					box_euler = tf.transformations.euler_from_quaternion(box_orientation)
					circle = math.sqrt(math.pow(self.tag_trans[0] - box_x,2)+math.pow(self.tag_trans[1] - box_y,2))
					if circle <= 1 :
						print("-!-!--!-!---!--!---!--!--!-!--!-!--\nID: %d\nCamera:\t%.2f\t%.2f\nLidar:\t%.2f\t%.2f\nCircle: %.2f\t%.2f"%(T_msg.markers[i].id,self.tag_trans[0],self.tag_trans[1],box_x,box_y,circle,box_euler[2]))
					else:
						
						print("Olmadi") # -----------------------\nID: %d\nCamera:\t%.2f\t%.2f\nLidar:\t%.2f\t%.2f\nCircle: %.2f"%(T_msg.markers[i].id,self.tag_trans[0],self.tag_trans[1],box_x,box_y,circle))


	def Rotation(self,translation,orientation):
		self.rotation_matrix =  np.array([1,0,0,0,0,-1,0,1,0]).reshape(3,3) #-90 degree about the x axis
		self.converted_trans = np.dot(self.rotation_matrix,np.asarray(translation).reshape(3,1))
		self.converted_euler = np.dot(self.rotation_matrix,np.asarray(tf.transformations.euler_from_quaternion(orientation)).reshape(3,1))
		self.converted_quaternion = tf.transformations.quaternion_from_euler(self.converted_euler[0], self.converted_euler[1],self.converted_euler[2])
		return self.converted_trans, self.converted_quaternion

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
		
	def moveARtag2Base(self,tagframe):
		return np.dot(self.getTagFrame(tagframe)[2],self.getHuskyPose()[2])		


if __name__ == '__main__':
	rospy.init_node("fusion")
	r = rospy.Rate(1)
	matcher = TagMatcher()
	while not rospy.is_shutdown():
		matcher.Fusion()
		r.sleep()
 
