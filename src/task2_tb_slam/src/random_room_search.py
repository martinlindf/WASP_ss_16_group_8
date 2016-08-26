#!/usr/bin/env python
'''
This node published random goal positions for move_base to follow
'''
#Import Python Packages, ROS messages

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
#from move_base_msgs import MoveBaseActionGoal

from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
import tf
import time

import random


# List of goal points
goal_points_list = []

latest_goal_id = None
latest_goal_time = time.time()

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

def create_goal_points_list():

	global goal_points_list

	goal_points_list.append([-0.57, -0.24, 0])
	goal_points_list.append([-0.46, 0.13, 0])
	goal_points_list.append([0.79, -0.08, 0])
	goal_points_list.append([0.77, -0.89, 0])

	return

def get_goal_to_publish():

	global latest_goal_time
	global latest_goal_id
	global goal_points_list

	if latest_goal_id == None:

		latest_goal_id = 1
		latest_goal_time = time.time()

	else:

		if time.time() - latest_goal_time > 20.:

			latest_goal_id += 1
			latest_goal_time = time.time()
			
		else:

			print "Too little time passed since last goal sent"

			return None


	# publish_current_pose()
	goal_to_send = MoveBaseActionGoal()
	pose_stamped = PoseStamped()
	pose = Pose()

	idx = random.randrange(len(goal_points_list))

	pose.position.x = goal_points_list[idx][0]
	pose.position.y = goal_points_list[idx][1]


	roll = 0.
	pitch = 0.
	yaw = 0.
	quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

	pose.orientation.x = quaternion[0]
	pose.orientation.y = quaternion[1]
	pose.orientation.z = quaternion[2]
	pose.orientation.w = quaternion[3]

	pose_stamped.pose = pose
	goal = MoveBaseGoal()
	goal.target_pose = pose_stamped
	goal.target_pose.header.frame_id = 'map'
	goal_to_send.goal = goal


	

	# goal_id = GoalID()
	# goal_id.id = "test1"

	# test.goal_id = goal_id

	return goal_to_send


# Intializes everything
def start():

	# Create publisher
	global goal_publisher

	create_goal_points_list()

	#Initialize current node with some name
	rospy.init_node('random_room_searcher')


	pose_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)

	#move_base/goal (move_base_msgs/MoveBaseActionGoal) 

	rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_received_callback)

	rate = rospy.Rate(1./5.) # 5hz

	while not rospy.is_shutdown():

		goal_to_publish = get_goal_to_publish()

		if goal_to_publish != None:

			pose_publisher.publish(goal_to_publish)

			print "published a goal:"
			print goal_to_publish.goal.target_pose.pose.position

		rate.sleep()

# Main function
if __name__ == '__main__':
	start()
