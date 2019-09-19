#!/usr/bin/env python
import rospy
import sys
import math
import cv2
import matplotlib.pyplot as plt

from scipy.misc import imread
from geometry_msgs.msg import TwistStamped

class localization:

	print ("\nLocalization Node started:\n")
	print ("Subscribes:/object_geometry_blue")
	print ("	   /object_geometry_red")
	print ("Publishes: /robot_geometry")

	def __init__(self):

		# Initiate publisher
		self.robot_position = rospy.Publisher("/robot_geometry",TwistStamped,queue_size=3)

		# Subscribers
		self.yaya_sign_pos = rospy.Subscriber("/park_geometry", TwistStamped, self.callback_park,queue_size=3)


	# Getting data from /dur_geometry topics.
	def callback_park(self,data):
		uzaklik_park_y = data.twist.linear.z * 100
		if data.twist.linear.x > 0:
			uzaklik_park_x = data.twist.linear.x * 100
		else:
			uzaklik_park_x = data.twist.linear.x * -100

		# Traffic signs static coordinates.
		x_park = 84
		y_park = 84


		try:
			msg_robot=TwistStamped()
			msg_robot.header.frame_id="Turtlebot"
			msg_robot.twist.linear.x = 0
			msg_robot.twist.linear.y = 0
			msg_robot.twist.linear.z = 0
			msg_robot.twist.angular.x = 0
			msg_robot.twist.angular.y = 0
			msg_robot.twist.angular.z = 0
			if (data.twist.linear.z != 0):
				x_robot = x_park - uzaklik_park_x - 4
				y_robot = y_park - uzaklik_park_y + 15


				print ("x = ", x_robot)
				print ("y = ", y_robot)

				# Publish the coordinates of robot to /robot_geometry topic.
				msg_robot=TwistStamped()
				msg_robot.header.frame_id="Turtlebot"
				msg_robot.twist.linear.x = x_robot
				msg_robot.twist.linear.y = y_robot
				msg_robot.twist.linear.z = 0
				msg_robot.twist.angular.x = 0
				msg_robot.twist.angular.y = 0
				msg_robot.twist.angular.z = 0
			self.robot_position.publish(msg_robot)

		except:
			print ("None")

def main(args):

	# Class starts.
	loc = localization()

	# Initialize the node.
	rospy.init_node('localization', anonymous=True)

	# Continue the node until plotting window is closed.
	try:
		rospy.spin()


	except KeyboardInterrupt:
		print("Shutting down")

	# Close all remained open windows.
	cv2.destroyAllWindows()

# Python starting main function.
if __name__ == '__main__':
	main(sys.argv)
