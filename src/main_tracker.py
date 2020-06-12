 #!/usr/bin/env python
import fusion

import rospy
import tf
import actionlib
import time

import math
import numpy as np

from geometry_msgs.msg import Twist, Point, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan

from tf import TransformListener

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

class TagFollower:

	def __init__ (self):
		self.goal = MoveBaseGoal()
		self.readed_markers = []
		self.readed_markers_tf = []
		self.marker_list= []
		self.tf = TransformListener()
		rospy.loginfo("tagFollower Class initalized")

	def tagScan(self):
		self.tag_msg = rospy.wait_for_message("/ar_pose_marker",AlvarMarkers,timeout=None)
		self.getBox(self.tag_msg)

	def getBox(self,msg):
		if len(msg.markers) >= 1:
			""" Is there a readed tags?"""
			for i in range(0,len(msg.markers)):
				""" Each readed tags """
				print("%d - %.2f - %.2f - %.2f // %.2f - %.2f - %.2f - %.2f"%(msg.markers[i].id,msg.markers[i].pose.pose.position.x,msg.markers[i].pose.pose.position.y,msg.markers[i].pose.pose.position.z,msg.markers[i].pose.pose.orientation.x,msg.markers[i].pose.pose.orientation.y,msg.markers[i].pose.pose.orientation.z,msg.markers[i].pose.pose.orientation.w))
				if True or not msg.markers[i].id  in self.readed_markers: # true to bypass condition
					""" Has it been readed? """
					self.readed_markers.append(msg.markers[i].id) # add readed_markers list
					self.tag_frame = "/ar_marker_" + str(msg.markers[i].id) # create the frame
					self.old_quaternion = (
						msg.markers[i].pose.pose.orientation.x,
						msg.markers[i].pose.pose.orientation.y,
						msg.markers[i].pose.pose.orientation.z,
						msg.markers[i].pose.pose.orientation.w
					)
					self.old_offset = (
						msg.markers[i].pose.pose.position.x,
						msg.markers[i].pose.pose.position.y,
						msg.markers[i].pose.pose.position.z,
					)

					self.new_translation, self.new_quaternion =  self.RotateTags(self.old_offset,self.old_quaternion,'x') # -90 degree rotation around x axis

					self.marker_list.append(TagClass(msg.markers[i].id,self.new_translation[0],self.new_translation[1],self.new_translation[2],self.new_quaternion[0],self.new_quaternion[1],self.new_quaternion[2],self.new_quaternion[3]))
					self.readed_markers_tf.append(self.moveARtag2Base(self.tag_frame))
					self.getTagFrame(self.tag_frame)

	def RotateTags(self,translation,orientation,axis):
		if axis == 'y':
			self.rotation_matrix =  np.array([0,0,-1,0,1,0,1,0,0]).reshape(3,3) #-90 degree about the y axis
		elif axis == 'x':
			self.rotation_matrix =  np.array([1,0,0,0,0,-1,0,1,0]).reshape(3,3) #-90 degree about the x axis

		self.converted_trans = np.dot(self.rotation_matrix,np.asarray(translation).reshape(3,1))
		self.converted_euler = np.dot(self.rotation_matrix,np.asarray(tf.transformations.euler_from_quaternion(orientation)).reshape(3,1))
		self.converted_quaternion = tf.transformations.quaternion_from_euler(self.converted_euler[0], self.converted_euler[1],self.converted_euler[2])
		print("Rotation: \n")
		print("Translation %s\n------------------------"%axis)
		print(translation)
		print(self.converted_trans)
		print("Orientation %s\n------------------------"%axis)
		print(tf.transformations.euler_from_quaternion(orientation))
		print(self.converted_euler)
		return self.converted_trans, self.converted_quaternion
		

	def getHuskyPose(self):	
		try:
			(trans,rot) = self.tf.lookupTransform('/map', '/base_footprint', rospy.Time(0))
			print("\nHusky Pose Detected!")
			homo_matrix = tf.transformations.quaternion_matrix(rot)
			homo_matrix[0][3] = trans[0]
			homo_matrix[1][3] = trans[1]
			homo_matrix[2][3] = trans[2]
			print(homo_matrix)
			print(tf.transformations.euler_from_quaternion(rot))
			print("-------------------above Husky pose --------------------------------- \n")
			return (trans,rot,homo_matrix)
			
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print("Olmadi birader")		


	def getTagFrame(self,tagFrame):	
		try:
			(trans,rot) = self.tf.lookupTransform('/base_footprint', tagFrame, rospy.Time(0))
			print("\n%s  tag frame Detected!"%tagFrame)
			print(trans)
			print(tf.transformations.euler_from_quaternion(rot))

			new_trans, new_rot = self.RotateTags(trans,rot,'x')
			homo_matrix = tf.transformations.quaternion_matrix(new_rot)
			homo_matrix[0][3] = new_trans[0]
			homo_matrix[1][3] = new_trans[1]
			homo_matrix[2][3] = new_trans[2]
			return (new_trans,new_rot,homo_matrix)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print("Olmadi birader")		


	def moveARtag2Base(self,tagframe):
		return np.dot(self.getTagFrame(tagframe)[2],self.getHuskyPose()[2])


	def gotoBox(self):
		print("Available markers:\n" + str(self.readed_markers))
		if len(self.readed_markers) >= 1:
			rospy.loginfo("You can start the process")
			self.target_tag = int(raw_input("enter the ID of the target box: "))
			if self.target_tag in self.readed_markers:
			 	self.target_index = self.readed_markers.index(self.target_tag)
				self.target_homo_matrix = self.readed_markers_tf[self.target_index]
			 	self.goal.target_pose.header.frame_id = "map"
			 	self.goal.target_pose.header.stamp = rospy.Time.now()
				self.target_orientation = tf.transformations.quaternion_from_matrix(self.target_homo_matrix)
			 	self.goal.target_pose.pose.position.x = self.target_homo_matrix[0][3]
			 	self.goal.target_pose.pose.position.y = self.target_homo_matrix[1][3]
			  	self.goal.target_pose.pose.position.z = self.target_homo_matrix[2][3]
				self.goal.target_pose.pose.orientation.x = self.target_orientation[0]
			 	self.goal.target_pose.pose.orientation.y = self.target_orientation[1]
			 	self.goal.target_pose.pose.orientation.z = self.target_orientation[2]
			 	self.goal.target_pose.pose.orientation.w = self.target_orientation[3]
			 	print(self.goal)
				self.result = movebase_client(self.goal)
				if self.result:
					rospy.loginfo("Goal Execution Done!")
				else:
					rospy.loginfo("There is nothing to follow")
	
	def search(self):
		self.waypoints = [(0,2,math.pi),(0,-2,math.pi),(0,2,0),(1,-2, 0),(0,0,0)]
		for point in self.waypoints:
			self.goal.target_pose.header.frame_id = "map"
			self.goal.target_pose.header.stamp = rospy.Time.now()
			self.goal.target_pose.pose.position.x = point[0] 
			self.goal.target_pose.pose.position.y = point[1]
			self.quat = quaternion_from_euler(0,0,point[2])
			self.goal.target_pose.pose.orientation.x = self.quat[0]
			self.goal.target_pose.pose.orientation.y = self.quat[1]
			self.goal.target_pose.pose.orientation.z = self.quat[2]
			self.goal.target_pose.pose.orientation.w = self.quat[3]
			
			print(self.goal)
			self.result = movebase_client(self.goal)
			if self.result:
				rospy.loginfo("Goal Execution Done!")
			else:
				rospy.loginfo("There is nothing to follow")
			self.tagScan()
		print(self.readed_markers)
		  	

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

def random_walk(x,y):

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.orientation.w = 1
	result = movebase_client(goal)
	if result:
			rospy.loginfo("Goal Execution Done!")

def getLaser(laser_msg):
	# values at 0 degree
	print laser_msg.ranges[0]
	# values at 90 degree
	print laser_msg.ranges[357]
	print laser_msg.ranges[358]
	print laser_msg.ranges[359]
	print laser_msg.ranges[360]
	print laser_msg.ranges[361]
	print laser_msg.ranges[362]
	print laser_msg.ranges[363]
	# values at 180 degree
	print laser_msg.ranges[719]


if __name__ == '__main__':
	try:
		rospy.init_node("tag_follower")
		move = rospy.Publisher("/husky_velocity_controller/cmd_vel",Twist, queue_size = 10 )
		tagMonster = TagFollower()
		while not rospy.is_shutdown():
			choice = int(raw_input("What do you want?\n(-1) - break \n(1) - scan tags \n(2) - go-to-goal\n(3) - follow tag \n(4) - getHuskyPose \n(5) - read laser \n(6) - scan tag \n:"))
			if choice == 1:
				tagMonster.search()
			elif choice == 2:
				x = float(raw_input("x: "))
				y = float(raw_input("y: "))
				random_walk(x,y)
				tagMonster.tagScan()
			elif choice == 3:
				tagMonster.gotoBox()
			elif choice == 4:
				tagMonster.getHuskyPose()
			elif choice == 5:
				laser_msg = rospy.wait_for_message("/scan",LaserScan)
				getLaser(laser_msg)
			elif choice == 6:
				tagMonster.tagScan()
			elif choice == -1:
				break

	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation test finished.")
