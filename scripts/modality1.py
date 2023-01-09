#! /usr/bin/env python3
"""
.. module:: Modality1
	:platform: Unix
	:synopsis: Python code for interface of the modalities
	
.. moduleauthor:: Awais Tahir <awaistahir29@gmail.com>

This node implements the first modality for controlling the robot in the environment.
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import transformations
from std_srvs.srv import *

# Creating message 

msg = """ 
Modality 1
"""

# Action Client
goal_msg = MoveBaseGoal()				
achieved = False						
active_ = 0								
desired_position_x = 0					
desired_position_y = 0					


# Defining ActionClient(), the function starts the communication with wait_for_server()
def ActionClient():
	global goal_msg
	global client
	client.wait_for_server()
	goal_msg.target_pose.header.frame_id = 'map'			
	goal_msg.target_pose.header.stamp = rospy.Time.now()	
	goal_msg.target_pose.pose.orientation.w = 1				

# Defining done_cb() function, this function updates when status value changes
# and it's the core of the action, because it changes the behaviour of the robot
# the n variable 

def done_cb(status, result):
	global client
	global achieved

	if status == 2:
		print("Goal received a cancel request.")
		return
	if status == 3:
		print("Goal achieved!")
		achieved = True
		return
	if status == 4:
		print("Timeout expired. Goal aborted.")
		return
	if status == 5:
		print("The goal was not accepted.")
		return
	if status == 6:
		print("The goal received a cancel request when he didn't finish the task.")
		return
	if status == 8:
		print("The goal received a cancel request before it started executing. ")
		return

# No-parameter callback that gets called on transitions to Active.

def active_cb():
	k=1

# Callback that gets called whenever feedback for this goal is received. Takes one parameter: the feedback. 

def feedback_cb(feedback):
	k=1

# SetGoal() is used to set the goal data in the goal_msg which will be published.
# Sends a goal to the ActionServer, and also registers callbacks.
# If a previous goal is already active when this is called. We simply forget about 
# that goal and start tracking the new goal. No cancel requests are made.

def SetGoal(x, y):
	global goal_msg
	global client
	goal_msg.target_pose.pose.position.x = x
	goal_msg.target_pose.pose.position.y = y
	client.send_goal(goal_msg, done_cb, active_cb, feedback_cb)

# UpdatingVariables() is used to keep update the three parameters crucial to the script,
# active_, desired_position_x, desired_position_y.

def UpdatingVariables():
	global desired_position_x, desired_position_y, active_
	active_ = rospy.get_param('active')
	desired_position_x = rospy.get_param('des_pos_x')
	desired_position_y = rospy.get_param('des_pos_y')

# Defining main() function.

def main():
	global client
	global goal_msg
	global achieved

	client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)		# Action client.

	rospy.init_node('modality1') # Init node

	# Running the ActionClient() function to start the communication with the action.

	ActionClient()

	# Boolprint used in order to know if the previous state was printable.

	boolprint = 0
	print(msg) 

	while (1):
		
		# Updating the variables (including active_).

		UpdatingVariables()

		# If we are in Idle state but a goal was not achieved we need to cancel the goal.
		# If active_ is turned to 0 we can idle the process and wait until the
		# first modality is asked by the user. In any case, we want to cancel 
		# the goal asked by the user.

		if active_ == 0:
			
			# If the robot is idle forced by the user we do this:

			if boolprint == 0 and achieved == False:
				print("Modality 1 is currently in idle state. \n")
				client.cancel_goal()
				boolprint = 1

			# If the robot has achieved the goal.

			if achieved == True:
				boolprint = 1
				achieved = False

		# If active_ is turned to 1 we procede with the task of the process.

		elif active_ == 1:

			# If the prevoius state was Idle then we can set a new goal
			if boolprint == 1:
				print( "The robot is moving towards your desired target. ")
				SetGoal(desired_position_x, desired_position_y)	# Here we decide to set a new goal.
				boolprint = 0	# If this modality will be blocked, then we have to be put in idle.

			
if __name__ == '__main__':
    main()
