#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from serial import Serial, serialutil

import dynamic_reconfigure.client

class Node:

    def __init__(self):
        twist_topic = rospy.get_param("~twist_topic")
        empty_topic = rospy.get_param("~empty_topic")
        reply_topic = rospy.get_param("~reply_topic")

        self.enable = rospy.get_param("~enable")

        self.send_deploy

        dev = rospy.get_param("~dev", "/dev/ttyACM0")
        baud = int(rospy.get_param("~baud", "115200"))

        rospy.loginfo("Nordic_send - opening serial : " + dev)
        # TODO COPY SPECIFICS

        if self.enable:
            self.serial = Serial(dev, timeout=1, baudrate=baud)
        # TODO may need to close open

        # only need a callback on reply

        self.sub_camera = rospy.Subscriber(twist_topic, Twist, self.callback)
        rospy.loginfo("Nordic_send - subscribed to topic : " + twist_topic)
        self.sub_hector = rospy.Subscriber(empty_topic, Empty, self.callback)
        rospy.loginfo("Nordic_send - subscribed to topic : " + empty_topic)
        self.sub_reply = rospy.Subscriber(empty_topic, Empty, self.callback)

    def run(self):
        rospy.spin()

    def callback_twist(self, data):
        if self.enable:
            self.write_serial(data)

    def write_serial(self, data):
        if self.enable:
            self.serial.write(data)
        
if __name__ == '__main__':
    rospy.init_node('nordic_send', anonymous=True)
    node = Node()
    node.run()

    

