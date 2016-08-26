#!/usr/bin/env python
'''
This node simply publishes the current believed pose of the Turtlebot
'''
#Import Python Packages, ROS messages

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
import time

# Used to store the mos recent pose of the robot
latest_received_pose = None

def publish_current_pose():
	# Publish the current believed pose of the robot

	if latest_received_pose == None:

		# We don't know where the robot is
		print "Nothing to publish"

	else:

		# Let's send the latest robot position
		print "Publishing latest pose"

		position_to_publish = PoseStamped()
		position_to_publish.pose = latest_received_pose.pose.pose

		pose_publisher.publish(position_to_publish)

		quaternion = (
			position_to_publish.pose.orientation.x,
			position_to_publish.pose.orientation.y,
			position_to_publish.pose.orientation.z,
			position_to_publish.pose.orientation.w)

		euler_angles = quaternion_to_euler( quaternion )

	return

def quaternion_to_euler(quaternion):

	euler = tf.transformations.euler_from_quaternion(quaternion)

	# The angle we are interested in
	yaw_angle = euler[2]
	yaw_angle_degrees = yaw_angle*(180./3.14)
	
	return euler

def pose_received_callback(data):
	# When we receive a robot pose we update our belief of the robot pose

	global latest_received_pose
	
	latest_received_pose = data

	return

# Intializes everything
def start():

	# Create publisher
	global pose_publisher

	#Initialize current node with some name
	rospy.init_node('position_publisher')


	pose_publisher = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)

	rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_received_callback)

	rate = rospy.Rate(10) # 5hz

	while not rospy.is_shutdown():

		publish_current_pose()

		rate.sleep()

# Main function
if __name__ == '__main__':
	start()
