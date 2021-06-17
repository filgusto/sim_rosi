#!/usr/bin/env python
import rospy
import numpy as np
from sim_rosi.msg import RosiMovement
from sim_rosi.msg import RosiMovementHeader
from sensor_msgs.msg import Joy

class RosiNodeClass():

	# class attributes
	max_translational_speed = 5 # in [m/s]
	max_rotational_speed = 10 # in [rad/s]
	max_arms_rotational_speed = 0.52 # in [rad/s]

	# how to obtain these values? see Mandow et al. COMPLETE THIS REFERENCE
	var_lambda = 0.965
	wheel_radius = 0.1324
	ycir = 0.531

	# class constructor
	def __init__(self):
		# sends a message to the user
		rospy.loginfo('rosi_joy node started')

		# initializing some attributes
		self.omega_left = 0
		self.omega_right = 0
		self.arm_front_rotSpeed = 0
		self.arm_rear_rotSpeed = 0

		# computing the kinematic A matrix
		self.kin_matrix_A = self.compute_kinematicAMatrix(self.var_lambda, self.wheel_radius, self.ycir)

		# registering to publishers
		self.pub_traction = rospy.Publisher('/rosi/cmd/speed_traction', RosiMovementHeader, queue_size=1)
		self.pub_arm = rospy.Publisher('/rosi/cmd/speed_arms', RosiMovementHeader, queue_size=1)

		# registering to subscribers
		self.sub_joy = rospy.Subscriber('/joy', Joy, self.callback_Joy)

		# defining the eternal loop frequency
		node_sleep_rate = rospy.Rate(10)

		rospy.spin()

	# ---- Support Methods --------
	# -- Method for compute the skid-steer A kinematic matrix
	@staticmethod
	def compute_kinematicAMatrix(var_lambda, wheel_radius, ycir):

		# kinematic A matrix 
		matrix_A = np.array([[var_lambda*wheel_radius/2, var_lambda*wheel_radius/2],
							[(var_lambda*wheel_radius)/(2*ycir), -(var_lambda*wheel_radius)/(2*ycir)]])

		return matrix_A


	# joystick callback function
	def callback_Joy(self, msg):

		# receives current rostime
		time_ros = rospy.get_rostime()

		# receiving and treating commands
		j_vel_lin = msg.axes[1]
		j_vel_ang = -1 * msg.axes[0]
		j_arm_cmd = msg.axes[4]
		j_arms_s = [1 if msg.axes[5] == -1 else 0, msg.buttons[5], 1 if msg.axes[2] == -1 else 0, msg.buttons[4]]

		''' Treating traction command'''

		# computing desired linear and angular velocities of the platform
		des_vel_linear_x = self.max_translational_speed * j_vel_lin
		des_vel_angular_z = self.max_rotational_speed * j_vel_ang

		# b matrix
		b = np.array([[des_vel_linear_x], [des_vel_angular_z]])

		# finds the joints control
		x = np.linalg.lstsq(self.kin_matrix_A, b, rcond=-1)[0]

		# query the sides velocities and converts to rad/s
		omega_right = np.ndarray.tolist(np.deg2rad(x[0]))[0]
		omega_left = np.ndarray.tolist(np.deg2rad(x[1]))[0]

		# mouting traction cmd message
		cmd_traction = RosiMovementHeader()
		cmd_traction.header.stamp = time_ros
		cmd_traction.header.frame_id = 'rosi'
		cmd_traction.nodeID = [1, 2, 3, 4]
		cmd_traction.joint_var = [-omega_right, -omega_right, omega_left, omega_left]
		self.pub_traction.publish(cmd_traction)


		''' Treating arms command'''

		# computing rotational arms speed
		arm_rot_speed = j_arm_cmd * self.max_arms_rotational_speed

		# mounting the np  command list
		cmd_list = np.array([j_arms_s, 4*[arm_rot_speed]])
		cmd_list = np.prod(cmd_list, axis=1)

		# PAREI AQUI PAREI AQUI

		print(cmd_list)






		# eternal loop (until second order)
'''
		while not rospy.is_shutdown():

			# msg to the usr
			rospy.loginfo('hello')

			# sleeps for a while
			node_sleep_rate.sleep()


			arm_command_list = RosiMovementArray()
			traction_command_list = RosiMovementArray()

			# mounting the lists
			for i in range(4):

				# ----- treating the traction commands
				traction_command = RosiMovement()

				# mount traction command list
				traction_command.nodeID = i+1

				# separates each traction side command
				if i < 2:
					traction_command.joint_var = self.omega_right
				else:
					traction_command.joint_var = self.omega_left

				# appending the command to the list
				traction_command_list.movement_array.append(traction_command)

				# ----- treating the arms commands		
				arm_command = RosiMovement()
		
				# mounting arm command list
				arm_command.nodeID = i+1
				
				# separates each arm side command
				if i == 0 or i == 2:
					arm_command.joint_var = self.arm_front_rotSpeed
				else:
					arm_command.joint_var = self.arm_rear_rotSpeed

				# appending the command to the list
				arm_command_list.movement_array.append(arm_command)

			# publishing
			self.pub_arm.publish(arm_command_list)		
			self.pub_traction.publish(traction_command_list)

			

		# infinite loop
		#while not rospy.is_shutdown():
			# pass

		# enter in rospy spin
		#rospy.spin()


	
		
		'''	
	


# instaciate the node
if __name__ == '__main__':

	# initialize the node
	rospy.init_node('rosi_joy', anonymous=True)

	# instantiate the class
	try:
		node_obj = RosiNodeClass()
	except rospy.ROSInterruptException: pass

