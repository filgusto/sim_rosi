#!/usr/bin/env python
import rospy
import numpy as np 
from std_msgs.msg import Header
from sim_rosi.msg import RosiMovement
from sim_rosi.msg import RosiMovementArray
from sim_rosi.msg import ManipulatorJoints

# example parameters
_omega_left = 1
_omega_right = -1
_arm_front_rotSpeed = 0
_arm_rear_rotSpeed = 0

class RosiCommanding():

	# constructor
	def __init__(self):

		# sends a message to the user
		rospy.loginfo('rosi_command_example node started')

		# registering to publishers
		self.pub_traction = rospy.Publisher('/rosi/cmd/speed_traction', RosiMovementArray, queue_size=1)
		self.pub_arms = rospy.Publisher('/rosi/cmd/speed_arms', RosiMovementArray, queue_size=1)

		# registering to subscribers
		self.sub_arms_positions = rospy.Subscriber('/rosi/sensor/pos_arms', RosiMovementArray, self.callback_arms_position)

		# defining eternal loop rate frequency
		node_sleep_rate = rospy.Rate(10)

		# eternal loop (runs until second order)
		while not rospy.is_shutdown():			

			# retrieving current command setpoint
			traction_sp, arms_sp = self.getCommand()

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


			# -- Publishing both messages
			self.pub_traction.publish(tractions_command_list)
			self.pub_arms.publish(arms_command_list)

			# sleep for a while
			node_sleep_rate.sleep()


	# function that should return command setpoints for ROSI
	@staticmethod
	def getCommand():

			tractions_sp = [_omega_right, _omega_right, -_omega_left, -_omega_left]
			arms_sp = [_arm_front_rotSpeed, _arm_rear_rotSpeed, _arm_front_rotSpeed, _arm_rear_rotSpeed]

			return  tractions_sp, arms_sp

	# callback for the subscriber
	def callback_arms_position(self, msg):
		#print(msg)
		pass


if __name__ == '__main__':

	# initialize node
	rospy.init_node('rosi_command_example', anonymous=True)

	# instantiate the class
	try:
		node_obj = RosiCommanding()
	except rospy.ROSInterruptException: pass

