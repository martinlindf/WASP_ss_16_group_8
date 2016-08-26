#!/usr/bin/env python
'''
This source will act as support to finish your project and does not follow best
coding practices.
'''
#Import Python Packages, ROS messages
from __future__ import print_function
from __future__ import division
import roslib
import sys
import rospy

from std_msgs.msg import String
import numpy as np

#import the custom message we created to store objects
from wasp_custom_msgs.msg import object_loc
from math import hypot


#This is the main class for object detection, it has some initializations about nodes
#Call back functions etc
class object_detection:
	def __init__(self):
		#Create Rospy subscriber

		rospy.init_node('obj_loc_listener', anonymous=True)
		self.object_location = rospy.Subscriber("/object_location", object_loc, self.callback)
		
	def callback(self, data):
		print(data)
		rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.ID)


#Main function for the node
def main(args):
	ic = object_detection()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting Down object_detection Node')
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
