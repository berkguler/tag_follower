#!/usr/bin/env python

import rospy
 
from geometry_msgs.msg import Twist, Point
from pynput.keyboard import Key,Listener


l_increment = 0.5
a_increment = 0.1

def on_press(key):	
	global velocity
	global a_increment
	global l_increment

	if key == Key.down:
		velocity.x = velocity.x - l_increment
	elif key == Key.up:
		velocity.x = velocity.x + l_increment
	elif key == Key.left:
		velocity.z = velocity.z + a_increment
	elif key == Key.right:	
		velocity.z = velocity.z - a_increment
	elif key == Key.ctrl_r:
		velocity = Point()
	elif key == Key.shift_r: 
		l_increment = 0.5
		a_increment = 0.1
	elif key == Key.shift_l:
		l_increment= l_increment*1.1
		a_increment = a_increment*1.1
	elif key == Key.ctrl:
		l_increment= l_increment*0.9
		a_increment = a_increment*0.9



def on_release(key):
    global velocity
    
    # stop on PAUSE
    if key == Key.esc:
        print("quit on PAUSE")
        return False

if __name__=='__main__':
	rospy.init_node ("tag_follower")
	move = rospy.Publisher("/husky_velocity_controller/cmd_vel",Twist, queue_size = 10 )
	rate = rospy.Rate(100)
	speed = Twist()
	velocity = Point()
	listener = Listener(on_press=on_press, on_release=on_release, suppress=False)
	listener.start()	
	#print("A Hybrid Mobile Manipulator Teleop Interface\n------------------------------------------------------------\n\t\tUp Arrow\n\t\tLinear Speed Up\nLeft Arrow\t\tDown Arrow\t\tRight Arrow\nTurn Left\t\tLinear Speed Down\t\tTurn Right\n\nCtrl_r\t\tStop\nShift_l\t\increase incrementation\nCtrl_l\t\treduce incrementation\nShift_r\t\tReset incrementation\n\n")
	while listener.running and not rospy.is_shutdown():
		rospy.loginfo(speed)
		speed.linear.x = velocity.x
		speed.angular.z = velocity.z
		move.publish(speed)
		rate.sleep()
