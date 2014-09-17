#!/usr/bin/env python
#This is writing a new controller, this is the publishing script
#beginner_tutorials is the node of interest here


import rospy
import math
from std_msgs.msg import String

#find this out using rosmsg, you find that twist is a subset of geometry, and Vector 3 is the publishing data
from geometry_msgs.msg import Twist, Vector3
import sys, tty, termios
from sensor_msgs.msg import LaserScan

mean_distance = 0
people_distance = 1
people_angle = 0
behavior = ''
lead_left_avg = 0
lead_right_avg = 0
trailing_left_avg = 0
trailing_right_avg = 0

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
    global people_distance
    global people_angle
    for i in range(359):
        if msg_people.ranges[i] > 0 and msg_people.ranges[i] < 2.5:
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
    while not rospy.is_shutdown():
        prop_turn = 0.0015*(people_angle)
        if people_distance > 0.5 and people_distance < 0.7:
            if people_angle < 10 or people_angle > 350:
                msg_people = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))
            else:
                if people_angle <=180:
                    msg_people = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,prop_turn))
                else:
                    msg_people = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,-prop_turn))
        else:
            prop_dist = 0.2*(people_distance - 0.6)
            if people_angle < 10 or people_angle > 350:
                msg_people = Twist(Vector3(prop_dist,0.0,0.0), Vector3(0.0,0.0,0.0))
            else:
                if people_angle <=180:
                    msg_people = Twist(Vector3(prop_dist,0.0,0.0), Vector3(0.0,0.0,prop_dist/(0.5*people_distance)))
                else:
                    msg_people = Twist(Vector3(prop_dist,0.0,0.0), Vector3(0.0,0.0,-prop_dist/(0.5*people_distance)))
        #msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 2.0))
        #msg = Twist(angular=Vector3(z-2.0))
        pub_people.publish(msg_people)
        r.sleep()

def wall_scan_received(msg_wall):
    """Process data from laser scanner, msg is of type sensor_msgs/LaserScan"""
    print "scan received"
    lead_left_distance = []
    lead_right_distance = []
    trailing_left_distance = []
    trailing_right_distance = []
    global behavior
    global lead_left_avg
    global lead_right_avg
    global trailing_left_avg
    global trailing_right_avg
    for i in range(11):
        if msg_wall.ranges[i+40] > 0 and msg_wall.ranges[i+40] < 2:
            lead_left_distance.append(msg_wall.ranges[i+40])
        if msg_wall.ranges[i+130] > 0 and msg_wall.ranges[i+130] < 2:
            trailing_left_distance.append(msg_wall.ranges[i+130])
        if msg_wall.ranges[i+310] > 0 and msg_wall.ranges[i+310] < 2:
            lead_right_distance.append(msg_wall.ranges[i+310])
        if msg_wall.ranges[i+220] > 0 and msg_wall.ranges[i+220] < 2:
            trailing_right_distance.append(msg_wall.ranges[i+220])
    if len(lead_left_distance)+len(trailing_left_distance) > len(trailing_right_distance)+len(lead_right_distance):
        print 'its left!'
        lead_left_avg = sum(lead_left_distance)/float(len(lead_left_distance)+0.1)
        trailing_left_avg = sum(trailing_left_distance)/float(len(trailing_left_distance)+0.1)
        behavior = 'left'
    else:
        print 'its right'
        lead_right_avg = sum(lead_right_distance)/float(len(lead_right_distance)+0.1)
        trailing_right_avg = sum(trailing_right_distance)/float(len(trailing_right_distance)+0.1)
        behavior = 'right'       

def wall_follow():
    """The publisher for a Neato Controller"""
    pub_wall = rospy.Publisher('cmd_vel', Twist, queue_size=10) #we're publishing to cmd_vel with a type Twist
    sub_wall = rospy.Subscriber('scan', LaserScan, wall_scan_received)
    rospy.init_node('wall_follow', anonymous=True)
    r = rospy.Rate(10) # 10hz
    global behavior
    global lead_left_avg
    global lead_right_avg
    global trailing_left_avg
    global trailing_right_avg
    while not rospy.is_shutdown():
        if behavior == 'right':
            right_prop_dist = 0.3*(lead_right_avg - trailing_right_avg)
            if lead_right_avg < 1.0- 0.1 and lead_right_avg != 0:
                msg_wall = Twist(Vector3(0.1,0.0,0.0), Vector3(0.0,0.0,0.1/(0.5*(lead_right_avg)))) 
            elif lead_right_avg > 1.0 +0.1:
                msg_wall = Twist(Vector3(0.1,0.0,0.0), Vector3(0.0,0.0,-0.1/(0.5*(lead_right_avg)))) 
            else:
                if lead_right_avg - 0.1 < trailing_right_avg and lead_right_avg + 0.1 > trailing_right_avg:
                    msg_wall = Twist(Vector3(0.1,0.0,0.0), Vector3(0.0,0.0,0.0))
                else:
                    msg_wall = Twist(Vector3(0.1,0.0,0.0), Vector3(0.0,0.0,right_prop_dist))
        elif behavior == 'left':
            left_prop_dist = 0.3*(lead_left_avg - trailing_left_avg)
            if lead_left_avg < 1.0- 0.1 and lead_left_avg != 0:
                print 'correcting'
                msg_wall = Twist(Vector3(0.1,0.0,0.0), Vector3(0.0,0.0,-0.1/(0.5*(lead_left_avg))))
            elif lead_left_avg > 1.0 +0.1:
                print 'correcting'
                msg_wall = Twist(Vector3(0.1,0.0,0.0), Vector3(0.0,0.0,0.1/(0.5*(lead_left_avg)))) 
            else:
                print 'maintaining'
                if lead_left_avg - 0.1 < trailing_left_avg and lead_left_avg + 0.1 > trailing_left_avg:
                    msg_wall = Twist(Vector3(0.1,0.0,0.0), Vector3(0.0,0.0,0.0))
                else:
                    msg_wall = Twist(Vector3(0.1,0.0,0.0), Vector3(0.0,0.0,-left_prop_dist))                
        else:
            msg_wall = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0)) 
        pub_wall.publish(msg_wall)
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
        wall_follow()
    except rospy.ROSInterruptException: pass
