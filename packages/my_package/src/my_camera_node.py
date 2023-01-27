#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import numpy as np

import cv2
from cv_bridge import CvBridge

class MyCameraNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyCameraNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.sub = rospy.Subscriber('~cam', CompressedImage, self.callback)
        self._bridge = CvBridge()

    def callback(self, data):
        img=self._bridge.compressed_imgmsg_to_cv2(data)
        rospy.loginfo("Read frame of size %s from camera_node", img.size)

if __name__ == '__main__':
    # create the node
    node = MyCameraNode(node_name='my_camera_node')
    # keep spinning
    rospy.spin()