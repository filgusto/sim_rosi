#!/usr/bin/python
import rospy
from sim_rosi.msg import HokuyoReading

class rosiKinectClass():

    # constructor
    def __init__(self):

        # sends a message to the user
        rospy.loginfo('rosi_example_kinect node started.')

        # class variables
        self.data_hokuyo = HokuyoReading()

        # subscribes to topics
        self.sub_hokuyo = rospy.Subscriber('/sensor/hokuyo', HokuyoReading, self.callback_hokuyo)

        # spins eternally
        node_sleep_rate = rospy.Rate(10)    # runs loop at 10hz

        # this loop runs eternally until ROSCORE closes
        while not rospy.is_shutdown():

            # ====================
            # here you may insert your custom code to treat kinect data
            # remark that node is constantly storing received information in the following variables
            # unncomment a line and run this node to see what happens

            rospy.loginfo(self.data_hokuyo)

            # ====================

            # sleeps and spins once
            node_sleep_rate.sleep()


    # velodyne image receiving callback
    def callback_hokuyo(self, msg):
        # stores received msg in a class variable
        self.data_hokuyo = msg


if __name__=='__main__':

    # initialize node
    rospy.init_node('rosi_example_kinect', anonymous=True)

    # instantiates node class
    try:
        class_obj = rosiKinectClass()
    except rospy.ROSInterruptException: pass
