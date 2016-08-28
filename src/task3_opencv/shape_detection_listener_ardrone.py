#!/usr/bin/env python

#Import Python Packages, ROS messages
from __future__ import print_function
from __future__ import division

import sys
import rospy
import json

import numpy as np

from wasp_custom_msgs.msg import object_loc
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import PoseStamped


class object_detection_listener:
	def __init__(self):
		self.xcoord_glob = 0
		self.ycoord_glob = 0
		self.orientation_glob = 0
		self.map_xcoord = 0
		self.map_ycoord = 0

		rospy.init_node('obj_loc_listener_ardrone', anonymous=True)
		self.object_location = rospy.Subscriber("/object_location_ardrone", object_loc, self.callback_loc)
		self.object_location = rospy.Subscriber("/current_pose", PoseStamped, self.callback_turtlebot_location)
		self.navdata = rospy.Subscriber("/ardrone/navdata", Navdata, self.callback_nav)

	def callback_turtlebot_location(self, data):
		self.map_xcoord = data.pose.position.x
		self.map_ycoord = data.pose.position.y

	def callback_loc(self, data):
		if self.orientation_glob > 150.0 and self.orientation_glob < 210.0:
			rospy.loginfo(" Shape with ID %s found and saved", data.ID)
			new_data = dict()
			new_data["Robot"] = "AR.Drone"
			new_data["ID"] = data.ID
			new_data["point.x"] = data.point.x + self.xcoord_glob + self.map_xcoord
			new_data["point.y"] = data.point.y + self.ycoord_glob + self.map_ycoord
			new_data["point.z"] = data.point.z
			try:
				with open('object_loc_ardrone.json', 'a') as fp:
					json.dump(new_data, fp, sort_keys=True, indent=4)
			except:
				print("Could not write/append to loc file!!")
				pass
			# rospy.loginfo(rospy.get_caller_id() + " Shape ID: %s", data.ID)

	def callback_nav(self, data):
		alpha = 32 * np.pi/180.0
		try:
			
			orientation = data.tags_orientation[0] * np.pi/180.0
			#print("Orientation = " +  str(orientation * 180.0/np.pi))
			self.orientation_glob = orientation * 180.0/np.pi
			tags_xc = data.tags_xc[0]
			tags_yc = data.tags_yc[0]
			height = data.tags_distance[0]
			xcoord = -1*np.tan(alpha) * height * (tags_xc-500)/500
			ycoord = np.tan(alpha) * height * (500-tags_yc)/500
			xcoord = xcoord * np.cos(orientation - 180)
			ycoord = ycoord * np.sin(orientation - 180)
			self.xcoord_glob = xcoord
			self.ycoord_glob = ycoord
			#print("x = " + str(xcoord))
			#print("y = " + str(ycoord))
		except:
			pass		


#Main function for the node
def main(args):
	object_detection_listener()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting Down object_detection Node')

if __name__ == '__main__':
	main(sys.argv)
