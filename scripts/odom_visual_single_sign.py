#!/usr/bin/env python
import rospy
import sys
import math
import cv2

import matplotlib.pyplot as plt

from scipy.misc import imread
from math import cos, sin

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, rotation_matrix, quaternion_from_matrix, euler_from_quaternion




class final_localization(object):

	print ("\nOdom+Visual Localization Node started:\n")
	print ("Subscribes:/robot_geometry")
	print ("	   /odom")
	print ("Publishes: /odom_visual_robot_geometry")



	def __init__(self):

		rospy.init_node('final_localization', anonymous=True)

		self.odom_visual_robot_position = rospy.Publisher("/odom_visual_robot_geometry",TwistStamped,queue_size=3)

		self.localizaer_pos = rospy.Subscriber("/robot_geometry",TwistStamped,self.callback_localizer,queue_size=3)
		self.odom_pos = rospy.Subscriber("/odom", Odometry, self.callback_odom,queue_size=3)

		self.odom_visual_pos_x = 0
		self.odom_visual_pos_y = 0

		self.localizer_pos_x = 0
		self.localizer_pos_y = 0
		self.yaw = 0
		self.v_odom = 0



	def callback_localizer(self,data):
		self.localizer_pos_x = data.twist.linear.x
		self.localizer_pos_y = data.twist.linear.y



	def callback_odom(self,data):

		self.odom_pos_x = data.pose.pose.position.x
		self.odom_pos_y = data.pose.pose.position.y

		self.w_odom = data.twist.twist.angular.z
		self.v_odom = data.twist.twist.linear.x * math.pow(10, 7)

		(roll, pitch, self.yaw) = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])


#			print (self.odom_visual_pos_x)

	def run(self):
#		r = rospy.Rate(200)
		curr_time = rospy.Time.now()
		last_time = curr_time

		while not rospy.is_shutdown():
			curr_time = rospy.Time.now()
			dt = (curr_time - last_time).to_sec()

			last_time = curr_time

	
			if (self.localizer_pos_x != 0):
				self.odom_visual_pos_x = self.localizer_pos_x
				self.odom_visual_pos_y = self.localizer_pos_y
			else:
				self.odom_visual_pos_x += (self.v_odom * dt * sin(self.yaw) * 100) / math.pow(10, 7)
				self.odom_visual_pos_y += (self.v_odom * dt * cos(self.yaw) * 100) / math.pow(10, 7)

			msg_robot=TwistStamped()
			msg_robot.header.frame_id="Final Position"
			msg_robot.twist.linear.x = self.odom_visual_pos_x
			msg_robot.twist.linear.y = self.odom_visual_pos_y
			msg_robot.twist.linear.z = 0
			msg_robot.twist.angular.x = 0
			msg_robot.twist.angular.y = 0
			msg_robot.twist.angular.z = 0
			self.odom_visual_robot_position.publish(msg_robot)
			print self.odom_visual_pos_x 
			print self.odom_visual_pos_y

		try:
			rospy.spin()
	#		plt.show(block=True)

		except rospy.exceptions.ROSTimeMovedBackwardsException:
			print("Shutting down")



if __name__ == '__main__':
	node = final_localization()
	node.run()
