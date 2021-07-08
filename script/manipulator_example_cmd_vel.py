# Zsh calling with input example:
#$ rosrun sim_rosi manipulator_example_cmd_vel.py '[0,0,0,0,0,0,0]'
#!/usr/bin/env python
import sys
import rospy
import numpy as np
from std_msgs.msg import Header
from sim_rosi.msg import ManipulatorJoints

# defining parameter
_rot_speed = -0.2 * 0.7854

# define here which strategy will generate linear and angular velocities to joints
def getCommand(rot_joints):
    # mounts a rotational speed target vel array based on input list selector
    #print(rot_joints)
    return [a*_rot_speed for a in rot_joints]

# node definition
def node_fcn(rot_joints):

    # initialize node
    rospy.init_node('manipulator_example_cmd_vel', anonymous=True)

    # sends a message to the user
    rospy.loginfo('manipulator_example_cmd_vel node started')

    # registering publisher
    pub_arm_cmd = rospy.Publisher('/manipulator/cmd/vel_target_joints', ManipulatorJoints, queue_size=1)

    # defining eternal loop rate frequency
    node_sleep_rate = rospy.Rate(2)

    # eternal loop
    while not rospy.is_shutdown():

        # retrieving command setpoint data
        msg_data_arr = getCommand(rot_joints)

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

# main script-
if __name__=='__main__':

    # calling function
    if len(sys.argv) < 2:
        rospy.loginfo('received no argument')

        # calling node
        node_fcn([0, 0, 0, 0, 0, 0, 0])
    else:
        rospy.loginfo('argument received')

        # converting input from str to list of numbers
        l_in = sys.argv[1]
        l_in = list(map(int, [l_in[1], l_in[3], l_in[5], l_in[7], l_in[9], l_in[11], l_in[13]]))

        # calling node
        node_fcn(l_in)





