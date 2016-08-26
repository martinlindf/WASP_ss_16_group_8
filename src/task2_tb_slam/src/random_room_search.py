#!/usr/bin/env python
'''
This node publishes random goal positions for move_base to follow
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
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalStatus

import tf
import time

import random


# List of goal points
goal_points_list = []

latest_goal_id = None
latest_goal_time = time.time()

goal_publisher = None


# Used to store the mos recent pose of the robot
latest_received_pose = None


def create_goal_points_list():
	# These are the goal points for the dummy environment we have
	global goal_points_list

	goal_points_list.append([-0.57, -0.24, 0])
	goal_points_list.append([-0.46, 0.13, 0])
	goal_points_list.append([0.79, -0.08, 0])
	goal_points_list.append([0.77, -0.89, 0])

	return

def goal_status_callback(data):
	# Whenever we get a status message, we will process it and 
	# try to check if all the missions are already completed
	global latest_goal_id

	print "=================="
	print "Received a Goal Status List message"

	for idx in range(len(data.status_list)):

		temp_goal_status = data.status_list[idx]

		print "idx = ", idx
		#print "status = ", temp_goal_status.status
		temp_status = temp_goal_status.status
		#print "text = ", temp_goal_status.text
		#print "goal id = ", temp_goal_status.goal_id.id
		temp_goal_id = int(temp_goal_status.goal_id.id)

		if temp_goal_id < latest_goal_id:

			# The status of an old mission is published for a while
			# even after it is finished, we need to ignore it
			print "Old goal id, ignore"

		else:

			if temp_status > 2 and temp_status < 6:

				# Here we check the status of the latest mission,
				# if it is a recent mission and with a status that 
				# indicates that it is not in progress anymore, we simply
				# call a new mission

				global latest_goal_id
				print "Something happened, trigger new"
				try_to_publish_goal()
				global latest_goal_time
				latest_goal_time = time.time() - 21.

	return

def get_goal_to_publish():
	# Get a random goal and go there
	# We could try to improve this random search, and have some memory

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

			# Print should not happen anymore
			print "Too little time passed since last goal sent"

			return None


	goal_to_send = MoveBaseActionGoal()

	current_goal_id = GoalID()
	current_goal_id.id = str(latest_goal_id)
	goal_to_send.goal_id = current_goal_id;

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

	return goal_to_send

def try_to_publish_goal():

	global goal_publisher

	goal_to_publish = get_goal_to_publish()

	if goal_to_publish != None:

		goal_publisher.publish(goal_to_publish)

		print "published a goal:"
		print goal_to_publish.goal.target_pose.pose.position


	return

# Intializes everything
def start():

	# Create publisher
	global goal_publisher

	create_goal_points_list()

	#Initialize current node with some name
	rospy.init_node('random_room_searcher')


	goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)

	#move_base/goal (move_base_msgs/MoveBaseActionGoal) 

	rospy.Subscriber("/move_base/status", GoalStatusArray, goal_status_callback)


	print "Sleeping for two seconds, to try to make sure the navigation goal is correctly received"
	time.sleep(2.0)

	rate = rospy.Rate(1./5.) # 5hz

	while not rospy.is_shutdown():

		
		rate.sleep()

# Main function
if __name__ == '__main__':
	start()
