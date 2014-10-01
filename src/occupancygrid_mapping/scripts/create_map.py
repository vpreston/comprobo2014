#!/usr/bin/env python
## Simple mapping algorithm based on occupancy grids

import rospy
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Pose
from sensor_msgs.msg import LaserScan
from tf import TransformListener
from tf.transformations import euler_from_quaternion
from math import cos, sin, pi, sqrt, ceil
from matplotlib.pyplot import imshow
import cv2
import numpy as np

class OccupancyGridMapper:
	def __init__(self):
		cv2.namedWindow("map")
		rospy.init_node("occupancy_grid_mapper")
		self.origin = [-10, -10]
		self.seq = 0
		self.resolution = .1
		self.n = 200
		self.p_occ = 0.5*np.ones((self.n,self.n))
		rospy.Subscriber("scan", LaserScan, self.process_scan, queue_size=1)
		self.pub = rospy.Publisher("map",OccupancyGrid)
		self.tf_listener = TransformListener()

	def is_in_map(self,x_ind,y_ind):
		return not(x_ind < self.origin[0] or
				   x_ind > self.origin[0] + self.n*self.resolution or
				   y_ind < self.origin[1] or
				   y_ind > self.origin[1] + self.n*self.resolution)

	def process_scan(self, msg):
		if len(msg.ranges) != 360:
			return
		# get pose according to the odometry
		p = PoseStamped(header=Header(stamp=msg.header.stamp,frame_id="base_link"), pose=Pose())
		self.odom_pose = self.tf_listener.transformPose("odom", p)
		# convert the odom pose to the tuple (x,y,theta)
		self.odom_pose = OccupancyGridMapper.convert_pose_to_xy_and_theta(self.odom_pose.pose)
		for i in range(360):
			if msg.ranges[i] > 0.0 and msg.ranges[i] < 5.0:
				# draw a picture for this!!
				map_x = self.odom_pose[0] + msg.ranges[i]*cos(i*pi/180.0+self.odom_pose[2])
				map_y = self.odom_pose[1] + msg.ranges[i]*sin(i*pi/180.0+self.odom_pose[2])

				x_detect = int((map_x - self.origin[0])/self.resolution)
				y_detect = int((map_y - self.origin[1])/self.resolution)

				u = (map_x - self.odom_pose[0], map_y - self.odom_pose[1])
				magnitude = sqrt(u[0]**2 + u[1]**2)
				n_steps = max([1,int(ceil(magnitude/self.resolution))])
				u_step = (u[0]/(n_steps-1), u[1]/(n_steps-1))
				marked = set()
				for i in range(n_steps):
					curr_x = self.odom_pose[0] + i*u_step[0]
					curr_y = self.odom_pose[1] + i*u_step[1]
					if not(self.is_in_map(curr_x,curr_y)):
						break
					if (curr_x < self.origin[0] or
						curr_x > self.origin[0] + self.n*self.resolution or
						curr_y < self.origin[1] or
						curr_y > self.origin[1] + self.n*self.resolution):
						break
					x_ind = int((curr_x - self.origin[0])/self.resolution)
					y_ind = int((curr_y - self.origin[1])/self.resolution)
					if x_ind == x_detect and y_ind == y_detect:
						break
					if not((x_ind,y_ind) in marked):
						p_occ_prop = self.p_occ[x_ind,y_ind]*0.1
						p_notocc_prop = (1-self.p_occ[x_ind,y_ind])*0.9
						self.p_occ[x_ind,y_ind] = p_occ_prop / (p_occ_prop+p_notocc_prop)
					else:
						marked.add((x_ind,y_ind))
				if self.is_in_map(map_x, map_y):
					p_occ_prop = self.p_occ[x_detect,y_detect]*0.8
					p_notocc_prop = (1-self.p_occ[x_detect,y_detect])*.05
					self.p_occ[x_detect,y_detect] = p_occ_prop / (p_occ_prop+p_notocc_prop)

		self.seq += 1
		if self.seq % 10 == 0:
			# make occupancy grid
			map = OccupancyGrid()
			map.header.seq = self.seq
			self.seq += 1
			map.header.stamp = msg.header.stamp
			map.header.frame_id = "map"
			map.info.origin.position.x = self.origin[0]
			map.info.origin.position.y = self.origin[1]
			map.info.width = self.n
			map.info.height = self.n
			map.info.resolution = self.resolution
			map.data = [0]*self.n**2
			for i in range(self.n):
				for j in range(self.n):
					idx = i+self.n*j
					if self.p_occ[i,j] < 0.1:
						map.data[idx] = 0
					elif self.p_occ[i,j] > 0.9:
						map.data[idx] = 100
					else:
						map.data[idx] = -1
			self.pub.publish(map)

		# create the image from the probabilities
		im = np.zeros((self.p_occ.shape[0],self.p_occ.shape[1],3))
		for i in range(im.shape[0]):
			for j in range(im.shape[1]):
				if self.p_occ[i,j] < 0.1:
					im[i,j,:] = 1.0
				elif self.p_occ[i,j] > 0.9:
					im[i,j,:] = 0.0
				else:
					im[i,j,:] = 0.5

		x_odom_index = int((self.odom_pose[0] - self.origin[0])/self.resolution)
		y_odom_index = int((self.odom_pose[1] - self.origin[1])/self.resolution)

		cv2.circle(im,(y_odom_index, x_odom_index),2,(255,0,0))
		cv2.imshow("map",cv2.resize(im,(500,500)))
		cv2.waitKey(20)

	@staticmethod
	def convert_pose_to_xy_and_theta(pose):
		""" Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
		orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
		angles = euler_from_quaternion(orientation_tuple)
		return (pose.position.x, pose.position.y, angles[2])

	def run(self):
		r = rospy.Rate(10)
		while not(rospy.is_shutdown()):
			r.sleep()

if __name__ == '__main__':
    try:
        node = OccupancyGridMapper()
        node.run()
    except rospy.ROSInterruptException: pass
