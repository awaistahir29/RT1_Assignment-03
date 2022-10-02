#!/usr/bin/env python3

import rospy
import os
import signal


# Declaring strings with some messages to display.

title = """ 
User Interface """ + """
Choose Among Three Modalities:"""

menu_msg = """
[1] """ + """Drive Automatically""" + """
[2] """ + """Drive using Keyboard. """ + """
[3] """ + """Drive using Keyboard Assisted by the Program.""" + """
[4] """ + """Quit Simulation.
"""

# Defining function switch(), this function will start the different
# modalities depending on what the user decides to choose. The variable
# boolprint is used to wait in the first modality the end of the task.

boolprint = False
def switch():
	global boolprint 
	print(menu_msg)

	if boolprint == True:
		print("Press [0] for canceling the target.")
		boolprint = False
	command = input('Instert a command \n')
	"""Command: the variable to assign the modality.
	"""

	# Setting all the modalities idle.
	
	if command == "0":
		rospy.set_param('active', 0)
		print("Idle.")
		active_=rospy.get_param("/active")
		print(active_)

	# Starting the first modality, then asking the user which position he wants to reach.
	# Once the position is written we set the position on the parameters, these will be
	# read by the first modality.

	elif command == "1":

		rospy.set_param('active', 0)
		print("Modality 1 is active.")
		active_=rospy.get_param("/active")
		print("Where do you want the robot to go?")
		des_x_input = float(input("Insert the desired x position: "))
		des_y_input = float(input("Insert the desired y position: " ))
		print("Now we try to reach the position x = " + str(des_x_input) + " , y = " + str(des_y_input))
		print("The robot is moving towards your desired target!" )		
		rospy.set_param('des_pos_x', des_x_input)
		rospy.set_param('des_pos_y', des_y_input)
		rospy.set_param('active', 1)
		boolprint=True

	# Starting the second modality.

	elif command == "2":
		rospy.set_param('active', 2)
		print("Modality 2 is active." )
		active_ = rospy.get_param("/active")
	
	# Starting the third modality.

	elif command == "3":
		rospy.set_param('active', 3)
		print("Modality 3 is active.")
		active_=rospy.get_param("/active")

	# If we want to quit the program, we press 4.

	elif command == "4":
		print("Exiting...")
		os.kill(os.getpid(), signal.SIGKILL)
	
	# If the user presses anything else, we want to quit the program.

	else:
		print("Wrong key!")

# What we want now, is to call the functions created and printing the starting message.

def main():
	print(title)
	while not rospy.is_shutdown():
		switch()

if __name__ == '__main__':
    main()
