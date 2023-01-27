#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
import numpy as np
from cv_bridge import CvBridge

class MyCameraNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyCameraNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.sub = rospy.Subscriber('~cam', String, self.callback)

    def callback(self, data):
        img=compressed_imgmsg_to_cv2(data.data, dst_format="jpeg")
        rospy.loginfo("Got frame of size %s", img.size)

if __name__ == '__main__':
    # create the node
    node = MyCameraNode(node_name='my_camera_node')
    # keep spinning
    rospy.spin()