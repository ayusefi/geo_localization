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
		self.park_sign_pos = rospy.Subscriber("/park_geometry",TwistStamped,self.callback_blue,queue_size=3)
		self.yaya_sign_pos = rospy.Subscriber("/yaya_geometry", TwistStamped, self.callback_red,queue_size=3)




		# robot's y position initial value. It was added after applying threshold because python needed it to give a value for y_old in line 85
		self.y = None

	# Getting data from /sollama_geometry topic.
	def callback_blue(self,data):
		self.uzaklik_blue = data.twist.linear.z

	# Getting data from /dur_geometry topics.
	def callback_red(self,data):
		uzaklik_red = data.twist.linear.z

		# Traffic signs static coordinates.
		x1 = 84
		y1 = 36

		x2 = 20
		y2 = 10

		try:


			# Getting distance info from two subscribed topics.
			r1 = self.uzaklik_blue * 100
			r2 = uzaklik_red * 100

			# Constants ( localization mathematical formula starts from here.)
			n = math.pow(r1, 2) - math.pow(r2, 2)
			a = math.pow(x1, 2) - math.pow(x2, 2) + math.pow(y1, 2) - math.pow(y2, 2)
			b = 2 * x1 - 2 * x2
			c = 2 * y1 - 2 * y2
			d = a - n 
			e = math.pow(b, 2) * math.pow(r1, 2) - math.pow(b, 2) * math.pow(x1, 2) + 2 * x1 * b * d - math.pow(d, 2) - math.pow(b, 2) * math.pow(y1, 2)
			f = 2 * x1 * b * c - 2 * d * c - 2 * y1 * math.pow(b, 2)
			g = math.pow(b, 2) + math.pow(c, 2)

			delta = (math.pow(f, 2) + 4 * g * e) 

			kok_delta = math.sqrt(delta)

			y_old = self.y

			# The delta sign in polynom calculation formula depends on the distance of r1 and r2.
			if r1 > r2:
				self.y = (-f - kok_delta) / (2 * g)
				x = (a - n - c * self.y) / b
			else:
				self.y = (-f + kok_delta) / (2 * g)
				x = (a - n - c * self.y) / b
			
			# To remove the noise define a threshold for y ( What is the best threshold???)
			y_difference = self.y - y_old
			if abs(y_difference) > 5:
				self.y = y_old

			print ("x = ", x)
			print ("y = ", self.y)

			# Publish the coordinates of robot to /robot_geometry topic.
			msg_robot=TwistStamped()
			msg_robot.header.frame_id="Turtlebot"
			msg_robot.twist.linear.x = x
			msg_robot.twist.linear.y = self.y
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
