#!/usr/bin/env python3
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

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber("~tick_l", WheelEncoderStamped, self.cb_enc_l)
        self.sub_encoder_ticks_right = rospy.Subscriber("~tick_r",WheelEncoderStamped, self.cb_enc_r)
        self.sub_executed_commands = rospy.Subscriber("~cmd", WheelsCmdStamped, self.cb_executed_commands)

        # Publishers
        self.pub_integrated_distance_left = rospy.Publisher("dist_l",Float32)
        self.pub_integrated_distance_right = rospy.Publisher("dist_r",Float32)

        self.log("Initialized")

    def cb_enc_l(self,msg):
        self.cb_encoder_data("l",msg)
    def cb_enc_r(self, msg):
        self.cb_encoder_data("r",msg)
    def cb_encoder_data(self, wheel, msg):
        """ Update encoder distance information from ticks.
        """
        if wheel=="l":
            ltick=Int32()
            ltick.data=msg.data
            self.bag.write("tick_l",ltick)
        else:
            rtick = Int32()
            rtick.data = msg.data
            self.bag.write("tick_r", rtick)

    def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """
        self.bag.write("wheels_cmd_executed", msg)

    def on_shutdown(self):
        self.bag.close()


if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")