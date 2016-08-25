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
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
#import the custom message we created to store objects
from wasp_custom_msgs.msg import object_loc
import tf
from math import hypot

#Define Constants

#Focal Length of the Asus Prime sensor camera
focal_leng = 570.34222

#This may change during the competetion, need to be calibrated
square_side_lenth = 0.115 #in mts
triangle_side_length = 0.112 #in mts
star_side_length = 0.049

#This function finds the lengths of all the sides and estimates the longest.
def Longest_Length(approxcontour):
	#add the first element in the end to complete the loop
	approxcontour = np.concatenate((approxcontour,[approxcontour[0]]))
	#The below lines find the length between two adjacent points
	#and append them in  an array
	ptdiff = lambda (p1,p2): (p1[0]-p2[0], p1[1]-p2[1])
	diffs = map(ptdiff, zip(approxcontour,approxcontour[1:]))
	dist = []
	for d in diffs:
		dist.append(hypot(*d))
	#find maximum of lenghts found
	LongestSide = max(dist)
	return LongestSide

#This is the main class for object detection, it has some initializations about nodes
#Call back functions etc
class object_detection:
	def __init__(self):
		#Create Rospy Publisher and subscriber
		self.object_location_pub = rospy.Publisher("/object_location", object_loc, queue_size =1)
		#original images is huge and creates lot of latency, therefore subscribe to compressed image

		#self.image_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.callback)
		self.image_sub = rospy.Subscriber("/ardrone/image_raw", Image, self.callback)
		#Cv Bridge is used to convert images from ROS messages to numpy array for openCV and vice versa
		self.bridge = CvBridge()
		#Obejct to transform listener which will be used to transform the points from one coordinate system to other.
		self.tl = tf.TransformListener()

	def _get_shapes(self, cv_image, contours, msg_id):
		
		contour_color = {110: (0,0,255), 120: (0, 255,0), 130: (255,0,0)}
		
		for x in range (len(contours)):
			tmp_msg_id = msg_id
			contourarea = cv2.contourArea(contours[x]) #get area of contour
			if contourarea > 600: #Discard contours with a small area as this may just be noise
				#The below 2 functions help you to approximate the contour to a nearest polygon
				arclength = cv2.arcLength(contours[x], True)
				approxcontour = cv2.approxPolyDP(contours[x], 0.02 * arclength, True)
				#Find the coordinates of the polygon with respect to he camera frame in pixels
				rect_cordi = cv2.minAreaRect(contours[x])
				obj_x = int(rect_cordi[0][0])
				obj_y = int(rect_cordi[0][1])

				#Check for Square
				if len(approxcontour) == 8: # Star
					cv2.drawContours(cv_image,[approxcontour],0,contour_color[msg_id],2)
					approxcontour = approxcontour.reshape((len(approxcontour),2))
					LongestSide = Longest_Length(approxcontour)
					Distance = (focal_leng*star_side_length)/LongestSide
					tmp_msg_id = tmp_msg_id + 1
					
				elif len(approxcontour) == 4: # Square
					cv2.drawContours(cv_image,[approxcontour],0,contour_color[msg_id],2)
					approxcontour = approxcontour.reshape((len(approxcontour),2))
					LongestSide = Longest_Length(approxcontour)
					Distance = (focal_leng*square_side_lenth)/LongestSide #focal length x Actual Border width / size of Border in pixels
					tmp_msg_id = tmp_msg_id + 2
				
				elif len(approxcontour) == 3: # Triangle
					cv2.drawContours(cv_image,[approxcontour],0,contour_color[msg_id],2)
					approxcontour = approxcontour.reshape((len(approxcontour),2))
					LongestSide = Longest_Length(approxcontour)
					Distance = (focal_leng*triangle_side_length)/LongestSide
					tmp_msg_id = tmp_msg_id + 3
				else:
					continue

				'''
				elif len(approxcontour) >= 10 and len(approxcontour) <= 11: # Circle-Star
					cv2.drawContours(cv_image,[approxcontour],0,contour_color[msg_id],2)
					approxcontour = approxcontour.reshape((len(approxcontour),2))
					LongestSide = Longest_Length(approxcontour)
					Distance = (focal_leng*square_side_lenth)/LongestSide
					tmp_msg_id = tmp_msg_id + 4
				'''

				#Calculate Cordinates wrt to Camera, convert to Map
				#Coordinates and publish message for storing
				#319.5, 239.5 = image centre
				obj_cam_x = ((obj_x - 319.5)*Distance)/focal_leng
				obj_cam_y = ((obj_y - 239.5)*Distance)/focal_leng

				#convert the x,y in camera frame to a geometric stamped point
				P = PointStamped()
				P.header.stamp = rospy.Time.now() - rospy.Time(23)
				#print ('time: ', data.header.stamp)
				P.header.frame_id = 'camera_rgb_optical_frame'
				P.point.x = obj_cam_x
				P.point.y = obj_cam_y
				P.point.z = Distance

				#Transform Point into map coordinates
				#trans_pt = self.tl.transformPoint('/map', P)

				#fill in the publisher object to publish
				obj_info_pub = object_loc()
				obj_info_pub.ID = tmp_msg_id #ID need to be changed
				
				#print("obj_info_pub.ID = " + str(tmp_msg_id))
				
				
				#obj_info_pub.point.x = trans_pt.point.x
				#obj_info_pub.point.y = trans_pt.point.y
				#obj_info_pub.point.z = trans_pt.point.z
				
				obj_info_pub.point.x = obj_cam_x
				obj_info_pub.point.y = obj_cam_y
				obj_info_pub.point.z = Distance
				#publish the message
				self.object_location_pub.publish(obj_info_pub)

	#Callback function for subscribed image
	def callback(self,data):
		#The below two functions conver the compressed image to opencv Image
		#'''
		#np_arr = np.fromstring(data.data, np.uint8)
		#cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR) # TURTLE COMPRESSED
		#'''
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") # DRONE RAW
		#Create copy of captured image
		img_cpy = cv_image.copy()
		#Color to HSV and Gray Scale conversion
		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

		#Red_Thresholds
		lower_red1 = np.array([0, 82, 82])
		upper_red1 = np.array([24, 255,255])
		lower_red2 = np.array([162,132,147])
		upper_red2 = np.array([188,255,255])
		#Blue Thresholds
		lower_blue = np.array([100,84,66])
		upper_blue = np.array([112,255,255])
		#Green Thresholds
		lower_green = np.array([53,42,83])
		upper_green = np.array([88,255,255])

		# Threshold the HSV image to get only single color portions
		# mask1a = cv2.inRange(hsv, lower_red1, upper_red1) # TA BORT
		mask1b = cv2.inRange(hsv, lower_red2, upper_red2)
		mask2 = cv2.inRange(hsv, lower_green, upper_green)
		mask3 = cv2.inRange(hsv, lower_blue, upper_blue)
		
		masks = [mask1b, mask2, mask3]
		color_id = [110, 110, 120, 130]
		
		#cv2.imshow("Green Mask",mask2)
		#cv2.imshow("Blue Mask",mask3)
		#cv2.imshow("Red Mask 1",mask1a)
		#cv2.imshow("Red Mask 2",mask1b)
				
		i = 0
		for mask in masks:
			#Find contours(borders) for the shapes in the image
			contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
			#Pass through each contour and check if it has required properties to classify into required object
			self._get_shapes(cv_image, contours, color_id[i])
			i = i + 1

		#Display the captured image
		cv2.imshow("Image",cv_image)
		#cv2.imshow("HSV", hsv)
		cv2.waitKey(1)


#Main function for the node
def main(args):
	rospy.init_node('object_detection', anonymous = False)
	ic = object_detection()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting Down object_detection Node')
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
