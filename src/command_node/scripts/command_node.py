#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy

class Node:

    def __init__(self):
        
        twist_topic = rospy.get_param("~twist_topic")
        empty_topic = rospy.get_param("~empty_topic")

        self.a_down_minimum_delta = int(rospy.get_param("~a_down_minimum_delta"))
        
        self.sub_joy = rospy.Subscriber('joy', Joy, self.callback_joy)
        rospy.loginfo("Command_node - subscribed topic : " + 'joy')
        
        self.pub_twist = rospy.Publisher(twist_topic, Twist, queue_size=1)
        rospy.loginfo("Command_node - published topic : " + twist_topic)
        self.pub_empty = rospy.Publisher(empty_topic, Empty, queue_size=1)
        rospy.loginfo("Command_node - published topic : " + empty_topic)

        self.a_down = False
        self.a_down_last = rospy.get_time()
    
    def callback_joy(self, data):
        twist = Twist()
        twist.linear.x = data.axes[1]
        twist.angular.z = data.axes[0]

        self.pub_twist.publish(twist)

        # handle button press
        if (data.buttons[0] == 1):
            if not self.a_down:
                now = rospy.get_time()
                if (now - self.a_down_last > self.a_down_minimum_delta):
                    self.a_down = True
                    self.a_down_last = now
                    self.pub_empty.publish(Empty())
        else:
            self.a_down = False

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('joy_node', anonymous=True)
    node = Node()
    node.run()
