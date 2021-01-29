#!/usr/bin/python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

class rosiKinectClass():

    # constructor
    def __init__(self):

        # sends a message to the user
        rospy.loginfo('rosi_example_kinect node started.')

        # class variables
        self.data_rgb = Image()
        self.data_depth = Image()
        self.data_info = CameraInfo()

        # subscribes to topics
        self.sub_rgb    = rospy.Subscriber('/sensor/kinect_rgb', Image, self.callback_rgb)
        self.sub_depth  = rospy.Subscriber('/sensor/kinect_depth', Image, self.callback_depth)
        self.sub_info   = rospy.Subscriber('/sensor/kinect_info', CameraInfo, self.callback_info)

        # spins eternaly
        node_sleep_rate = rospy.Rate(10)    # runs loop at 10hz

        # this loop runs eternally until ROSCORE closes
        while not rospy.is_shutdown():

            # ====================
            # here you may insert your custom code to treat kinect data
            # remark that node is constantly storing received information in the following variables
            # unncomment a line and run this node to see what happens

            #rospy.loginfo(self.data_rgb)
            #rospy.loginfo(self.data_depth)
            #rospy.loginfo(self.data_info)

            # ====================

            # sleeps and spins once
            node_sleep_rate.sleep()


    # rgb image receiving callback
    def callback_rgb(self, msg):
        # stores received msg in a class variable
        self.data_rgb = msg

    # depth image receiving callback
    def callback_depth(self, msg):
        # stores received msg in a class variable
        self.data_depth = msg

    # camera info receiving callback
    def callback_info(self, msg):
        # store received msg in a class variable
        self.data_info = msg

if __name__=='__main__':

    # initialize node
    rospy.init_node('rosi_example_kinect', anonymous=True)

    # instantiates node class
    try:
        class_obj = rosiKinectClass()
    except rospy.ROSInterruptException: pass
