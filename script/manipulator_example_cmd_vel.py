#!/usr/bin/python
import rospy
import numpy as np
from std_msgs.msg import Header
from sim_rosi.msg import ManipulatorJoints

# define here which strategy will generate linear and angular velocities to joints
def getCommand():

    # mounting the joints position array
    arm_joints_target_vel = [0, 0, 0, 0 ,0 , -0.0349, 5*0.0349]

    return arm_joints_target_vel

# main script-
if __name__=='__main__':

    # initialize node
    rospy.init_node('manipulator_example_cmd_vel', anonymous=True)

    # sends a message to the user
    rospy.loginfo('manipulator_example_cmd_vel node started')

    # registering publisher
    pub_arm_cmd = rospy.Publisher('/manipulator/cmd/joints_vel_target', ManipulatorJoints, queue_size=1)

    # defining eternal loop rate frequency
    node_sleep_rate = rospy.Rate(5)

    # eternal loop
    while not rospy.is_shutdown():

        # retrieving command setpoint data
        msg_data_arr = getCommand()

        # creating pub msg header
        msg_header = Header()
        msg_header.stamp = rospy.Time.now()
        msg_header.frame_id = 'gen3'

        # creating pub msg
        msg_pub = ManipulatorJoints()
        msg_pub.header = msg_header
        msg_pub.joint_variable = msg_data_arr

        # publishing message
        pub_arm_cmd.publish(msg_pub)

        # sends a message to user
        rospy.loginfo(msg_data_arr)

        # sleeps node
        node_sleep_rate.sleep()




