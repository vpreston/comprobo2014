#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Control the Neato with your keyboard!
-------------------------------------
g h  j  k l
  n  m  ,

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
anything else : stop

CTRL-C to quit
"""

#create a dictionary of key meanings
keyMeaning = {
	'g':(0,1), #twist right
	'h':(1,1), #turn right
	'j':(1,0), #forward
	'k':(1,-1), #turn left
	'l':(0,-1), #twist left
	'n':(-1,-1), #circle left back
	'm':(-1,0), #backward
	',':(-1,1), #circle right back
}   

speedKeys = {
	'q':(1.1,1.1), #new max speeds
	'z':(.9,.9), #new min speeds
	'w':(1.1,1), #forward faster
	'x':(.9,1), #forward slower
	'e':(1,1.1), #angular faster
	'c':(1,.9), #angular slower
}

def getch():
	"""Return the next character typed on the keyboard"""
	fd = sys.stdin.fileno()
	old_settings = termios.tcgetattr(fd)
	try:
		tty.setraw(sys.stdin.fileno())
		ch = sys.stdin.read(1)
	finally:
		termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
		return ch


def get_velocity(speed_init, turn_rate):
	""" provide the speed and turning goodness """
	return "currently:\tspeed %s\tturn %s " % (speed_init, turn_rate)

#set initial variables up
speed = 0.5
turn = 1

if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)

	pub = rospy.Publisher('cmd_vel', Twist) #know what your publishing to the bot
	rospy.init_node('teleop_twist_keyboard') #this is the node that your code is in

	x = 0
	th = 0
	status = 0

	try:
		print msg #print howto the terminal
		print get_velocity(speed, turn) #print robot stats to terminal
		while(1): #while things are going swell
			key = getch() #get the key pressed
			if key in keyMeaning.keys(): #if that key matches whats in the key Dictionary
				x = keyMeaning[key][0] #linear thing
				th = keyMeaning[key][1] #angular thing
			elif key in speedKeys.keys(): #if its a speed key
				speed = speed * speedKeys[key][0] #calculate new speed
				turn = turn * speedKeys[key][1] #claculate new turn rate

				print get_velocity(speed, turn) #print new status of bot
				if (status == 14): #if what is displayed in the terminal is gone, reprint the message
					print msg
				status = (status + 1) % 15
			else:
				x = 0 #stop!
				th = 0 #stop!
				if (key == '\x03'): #terminate program
					break
			twist = Twist() #make twist happy
			twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = 0
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
			pub.publisher(twist)
	except:
		print e #notify of an exception

	finally:
		twist = Twist() #make twist happy
		twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
		pub.publisher(twist)

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



