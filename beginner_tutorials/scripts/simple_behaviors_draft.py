#!/usr/bin/env python
#This is writing a new controller, this is the publishing script
#beginner_tutorials is the node of interest here


import rospy
from std_msgs.msg import String

#find this out using rosmsg, you find that twist is a subset of geometry, and Vector 3 is the publishing data
from geometry_msgs.msg import Twist, Vector3
import sys, tty, termios
from sensor_msgs.msg import LaserScan

mean_distance = 0
people_distance = 1
people_angle = 0

def front_scan_recieved(msg):
    """Process data from laser scanner, msg is of type sensor_msgs/LaserScan"""
    #print "front scan received"
    valid_ranges = []
    global mean_distance
    for i in range(5):
        if msg.ranges[i] > 0 and msg.ranges[i] < 8:
            valid_ranges.append(msg.ranges[i])
        if len(valid_ranges) > 0:
            mean_distance = sum(valid_ranges)/float(len(valid_ranges))
        else:
            mean_distance = 0


def wall_approach():
    """The publisher for a Neato Controller"""
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) #we're publishing to cmd_vel with a type Twist
    sub = rospy.Subscriber('scan', LaserScan, front_scan_recieved)
    rospy.init_node('wall_approach', anonymous=True)
    r = rospy.Rate(10) # 10hz
    global mean_distance
    while not rospy.is_shutdown():
        if mean_distance > 0.98 and mean_distance < 1.02:
            msg = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))
        else:
            prop = 0.2*(mean_distance - 1.0)
            msg = Twist(Vector3(prop,0.0,0.0), Vector3(0.0,0.0,0.0))
        #msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 2.0))
        #msg = Twist(angular=Vector3(z-2.0))
        pub.publish(msg)
        r.sleep()

def people_scan_recieved(msg_people):
    """Process data from laser scanner, msg is of type sensor_msgs/LaserScan"""
    print "scan received"
    valid_ranges = {}
    people_angles = []
    global people_distance
    global people_angle
    for i in range(359):
        if msg_people.ranges[i] > 0 and msg_people.ranges[i] < 2:
            valid_ranges[msg_people.ranges[i]] = i
        if len(valid_ranges) > 0:
            people_distance = min(valid_ranges.keys())
            people_angle = valid_ranges[people_distance]
            print people_distance
            print people_angle
        else:
            people_distance = -1.0
            people_angle = 0

def people_follow():
    """The publisher for a Neato Controller"""
    pub_people = rospy.Publisher('cmd_vel', Twist, queue_size=10) #we're publishing to cmd_vel with a type Twist
    sub_people = rospy.Subscriber('scan', LaserScan, people_scan_recieved)
    rospy.init_node('people_follow', anonymous=True)
    r = rospy.Rate(10) # 10hz
    global people_distance
    global people_angle
    prop_turn = 0.0005*(people_angle)
    while not rospy.is_shutdown():
        if people_distance > 0.9 and people_distance < 1.1:
            if people_angle < 10 or people_angle > 350:
                msg_people = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))
            else:
                msg_people = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,prop_turn))
        else:
            prop_dist = 0.2*(people_distance - 1.0)
            if people_angle < 10 or people_angle > 350:
                msg_people = Twist(Vector3(prop_dist,0.0,0.0), Vector3(0.0,0.0,0.0))
            else:
                msg_people = Twist(Vector3(prop_dist,0.0,0.0), Vector3(0.0,0.0,prop_turn))
        #msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 2.0))
        #msg = Twist(angular=Vector3(z-2.0))
        pub_people.publish(msg_people)
        r.sleep()

def get_keycmds():
    """Return the next character typed on the keyboard"""
    fd = sys.stdin.fileno()
    old_setttings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno(1))
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRIAN)
    return ch 

        
if __name__ == '__main__':
    try:
        people_follow()
    except rospy.ROSInterruptException: pass
