#!/usr/bin/env python3
import math

import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32, Int32
import rosbag


class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)

        # bag
        self.bag=rosbag.Bag("/data/bags/encoder.bag", "w")
        self.bag_flag=True

        # distance counter
        self.dist_l = 0
        self.dist_r = 0

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber("~tick_l", WheelEncoderStamped, self.cb_enc_l)
        self.sub_encoder_ticks_right = rospy.Subscriber("~tick_r",WheelEncoderStamped, self.cb_enc_r)
        self.sub_executed_commands = rospy.Subscriber("~cmd", WheelsCmdStamped, self.cb_executed_commands)

        # Publishers
        self.pub_integrated_distance_left = rospy.Publisher("dist_l",Float32)
        self.pub_integrated_distance_right = rospy.Publisher("dist_r",Float32)

        self.log("Initialized")

    def cb_enc_l(self,msg):
        if self.bag_flag:
            self.cb_encoder_data("l",msg)
    def cb_enc_r(self, msg):
        if self.bag_flag:
            self.cb_encoder_data("r",msg)
    def cb_encoder_data(self, wheel, msg):
        """ Update encoder distance information from ticks.
        """
        diff = np.pi * 0.066 * (msg.data / 135)
        mtick=Int32()
        mtick.data=msg.data
        mdiff=Float32()
        mdiff.data=diff
        mdist = Float32()
        if wheel=='l':
            self.dist_l+=diff
            mdist.data = self.dist_l
            self.pub_integrated_distance_left.publish(mdist)
        else:
            self.dist_r+=diff
            mdist.data = self.dist_r
            self.pub_integrated_distance_right.publish(mdist)

        if self.bag_flag:
            self.bag.write("tick_{}".format(wheel), mtick)
            self.bag.write("diff_{}".format(wheel), mdiff)
            self.bag.write("dist_{}".format(wheel), mdiff)

    def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """
        if self.bag_flag:
            self.bag.write("wheels_cmd_executed", msg)

    def on_shutdown(self):
        mdistl=Float32()
        mdistl.data=self.dist_l
        mdistr = Float32()
        mdistr.data = self.dist_r
        self.pub_integrated_distance_left.publish(mdistl)
        self.pub_integrated_distance_right.publish(mdistr)
        self.bag_flag=False
        self.bag.close()


if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")