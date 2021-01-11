#!/usr/bin/python
import rospy
from sim_rosi.msg import RosiMovementArray

# subscriber callback
def callback_armsPosition(msg):

	print(msg)

# node main function
if __name__ == '__main__':

	# initialize node
	rospy.init_node('example_subscribing', anonymous=True)

	# sends a message to the user
	rospy.loginfo('example_subscribing node started.')

	# register to subscriber
	sub_arms_position = rospy.Subscriber('/rosi/arms_joints_position', RosiMovementArray, callback_armsPosition)

	# spins forever
	rospy.spin()