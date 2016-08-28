#!/usr/bin/env python

#Import Python Packages, ROS messages
from __future__ import print_function
from __future__ import division

import sys
import rospy
import json

import numpy as np

from wasp_custom_msgs.msg import object_loc


class object_detection_listener:
	def __init__(self):
		rospy.init_node('obj_loc_listener', anonymous=True)
		self.object_location = rospy.Subscriber("/object_location", object_loc, self.callback_loc)

	def callback_loc(self, data):
			rospy.loginfo(" Shape with ID %s found and saved", data.ID)
			new_data = dict()
			new_data["Robot"] = "Turtle"
			new_data["ID"] = data.ID
			new_data["point.x"] = data.point.x
			new_data["point.y"] = data.point.y
			new_data["point.z"] = data.point.z
			try:
				with open('object_loc.json', 'a') as fp:
					json.dump(new_data, fp, sort_keys=True, indent=4)
			except:
				print("Could not write/append to loc file!!")
				pass
			# rospy.loginfo(rospy.get_caller_id() + " Shape ID: %s", data.ID)	


#Main function for the node
def main(args):
	object_detection_listener()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting Down object_detection Node')

if __name__ == '__main__':
	main(sys.argv)
