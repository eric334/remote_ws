#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Empty
from serial import Serial, serialutil

import dynamic_reconfigure.client

class Node:

    def __init__(self):
        twist_topic = rospy.get_param("~twist_topic")
        empty_topic = rospy.get_param("~empty_topic")
        reply_topic = rospy.get_param("~reply_topic")

        self.enable = rospy.get_param("~enable")
        rospy.loginfo("Nordic_send - serial enabled : " + str(self.enable))

        self.twist = Twist()

        self.send_deploy = False

        dev = rospy.get_param("~dev", "/dev/ttyACM0")
        baud = int(rospy.get_param("~baud", "115200"))

        rospy.loginfo("Nordic_send - opening serial : " + dev)
        # TODO COPY SPECIFICS

        if self.enable:
            self.serial = Serial(dev, timeout=1, baudrate=baud)

        # only need a callback on reply

        self.sub_twist = rospy.Subscriber(twist_topic, Twist, self.callback_twist)
        rospy.loginfo("Nordic_send - subscribed to topic : " + twist_topic)
        self.sub_empty = rospy.Subscriber(empty_topic, Empty, self.callback_empty)
        rospy.loginfo("Nordic_send - subscribed to topic : " + empty_topic)
        self.sub_reply = rospy.Subscriber(reply_topic, Empty, self.callback_reply)

    def run(self):
        rospy.spin()

    def callback_reply(self, empty):
        control_twist = TwistStamped()
        control_twist.twist = self.twist
        control_twist.header.frame_id = "con"

        if self.send_deploy:
            control_twist.header.frame_id = "condep"
            self.send_deploy = False

        print(control_twist)
        
        if self.enable:
            write_serial(control_twist)


    def callback_twist(self, data):
        self.twist = data

    def callback_empty(self,empty):
        self.send_deploy = True

    def write_serial(self, data):
        self.serial.write(data)
        
if __name__ == '__main__':
    rospy.init_node('nordic_send', anonymous=True)
    node = Node()
    node.run()

    

