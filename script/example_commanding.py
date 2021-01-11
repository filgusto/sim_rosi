#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from sim_rosi.msg import RosiMovement
from sim_rosi.msg import RosiMovementArray
from sim_rosi.msg import ManipulatorJoints

# example parameters
_omega_left = -1
_omega_right = 1
_arm_front_rotSpeed = 1
_arm_rear_rotSpeed = -1

def getCommand():

	tractions_sp = [_omega_right, _omega_right, -_omega_left, -_omega_left]
	arms_sp = [_arm_front_rotSpeed, _arm_rear_rotSpeed, _arm_front_rotSpeed, _arm_rear_rotSpeed]

	return  tractions_sp, arms_sp


# main function
if __name__=='__main__':

	# initialize node
	rospy.init_node('example_commanding', anonymous=True)

	# sends a message to the user
	rospy.loginfo('example_commanding node started')

	# registering to publishers
	pub_traction = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=1)
	pub_arms = rospy.Publisher('/rosi/command_arms_speed', RosiMovementArray, queue_size=1)

	# defining eternal loop rate frequency
	node_sleep_rate = rospy.Rate(10)

	# eternal loop (runs until second order)
	while not rospy.is_shutdown():			

		# retrieving current command setpoint
		traction_sp, arms_sp = getCommand()

		# retrieving current command setpoint
		traction_sp, arms_sp = getCommand()

		# creating a common header
		msg_header = Header()
		msg_header.stamp = rospy.Time.now()
		msg_header.frame_id = 'rosi_example'

		## -- Preparing traction message for publishing
		tractions_command_list = RosiMovementArray()
		tractions_command_list.header = msg_header
		for i in range(4):

			# setting the node id
			node_id_current = i+1

			# creating empty array
			traction_command = RosiMovement()		

			# inserting node id
			traction_command.nodeID = node_id_current

			# inserting the joint variable setpoint
			traction_command.joint_var = traction_sp[i]

			# appending a specific command to the list
			tractions_command_list.movement_array.append(traction_command)

		## -- Preparing arms message for publishing
		arms_command_list = RosiMovementArray()
		arms_command_list.header = msg_header
		for i in range(4):

			# setting the node id
			node_id_current = i+1

			# creating empty array
			arm_command = RosiMovement()

			# inserting node id
			arm_command.nodeID = node_id_current

			# inserting the joint variable setpoint
			arm_command.joint_var = arms_sp[i]

			# appending the specific command to the list
			arms_command_list.movement_array.append(arm_command)


		# -- Publishing both command messages
		pub_traction.publish(tractions_command_list)
		pub_arms.publish(arms_command_list)

		# sleep for a while
		node_sleep_rate.sleep()




