#!/usr/bin/python
import rospy
from sensor_msgs.msg import PointCloud2

class rosiKinectClass():

    # constructor
    def __init__(self):

        # sends a message to the user
        rospy.loginfo('rosi_example_kinect node started.')

        # class variables
        self.data_velodyne = PointCloud2()

        # subscribes to topics
        self.sub_velodyne = rospy.Subscriber('/velodyne/points2', PointCloud2, self.callback_velodyne)

        # spins eternally
        node_sleep_rate = rospy.Rate(10)    # runs loop at 10hz

        # this loop runs eternally until ROSCORE closes
        while not rospy.is_shutdown():

            # ====================
            # here you may insert your custom code to treat kinect data
            # remark that node is constantly storing received information in the following variables
            # unncomment a line and run this node to see what happens

            rospy.loginfo(self.data_velodyne)

            # ====================

            # sleeps and spins once
            node_sleep_rate.sleep()


    # rgb image receiving callback
    def callback_velodyne(self, msg):
        # stores received msg in a class variable
        self.data_velodyne = msg


if __name__=='__main__':

    # initialize node
    rospy.init_node('rosi_example_kinect', anonymous=True)

    # instantiates node class
    try:
        class_obj = rosiKinectClass()
    except rospy.ROSInterruptException: pass
