#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from sensor_msgs.msg import CompressedImage

from typing import cast

import numpy as np

import cv2
from cv_bridge import CvBridge

class MyCameraNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyCameraNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.sub = rospy.Subscriber('~cam', CompressedImage, self.callback)
        self.pub = rospy.Publisher(
            "~image/compressed",
            CompressedImage,
            queue_size=1,
            dt_topic_type=TopicType.VISUALIZATION,
            dt_help="The stream of JPEG compressed images from the modified camera feed",
        )
        self.image = None
        self._bridge = CvBridge()

    def callback(self, data):
        if self.image is None:
            self.image = self._bridge.compressed_imgmsg_to_cv2(data)
            rospy.loginfo("Received first frame of size %s from camera_node", self.image.size)
        self.image = self._bridge.compressed_imgmsg_to_cv2(data)

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():
            if self.image is not None:
                # image = cast(np.ndarray, self.image)
                # image_msg = CompressedImage()
                # image_msg.header.stamp = rospy.Time.now()
                # image_msg.format = "jpeg"
                # image_msg.data = image.tobytes()
                img=cv2.flip(self.image,0)
                image_msg = self._bridge.cv2_to_compressed_imgmsg(img, dst_format="jpeg")
                rospy.loginfo("Publishing modified image.")
                self.pub.publish(image_msg)
                rate.sleep()


if __name__ == '__main__':
    # create the node
    node = MyCameraNode(node_name='my_camera_node')
    # keep spinning
    node.run()
    rospy.spin()