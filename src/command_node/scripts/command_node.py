#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy

class Node:

    def __init__(self):
        
        twist_topic = rospy.get_param("~twist_topic")
        empty_topic = rospy.get_param("~empty_topic")

        self.update_rate = int(rospy.get_param("~update_rate"))
        self.a_down_minimum_delta = int(rospy.get_param("~a_down_minimum_delta"))
        
        self.sub_joy = rospy.Subscriber('joy', Joy, self.callback_joy)
        
        self.pub_twist = rospy.Publisher(twist_topic, Twist, queue_size=1)
        self.pub_empty = rospy.Publisher(empty_topic, Empty, queue_size=1)

        self.a_down = False
        self.a_down_last = 0

        rospy.spin()
    
    def callback_joy(self):

        # update twist
        twist = Twist()
        twist.linear.x = data.axes[1]
        twist.angular.z = data.axes[0]

        self.twist = twist

        # handle button press
        if (twist.buttons[0] == 1):
            if not self.a_down:
                if (rospy.get_rostime() - self.a_down_last > self.a_down_minimum_delta):
                    self.a_down = True
                    self.pub_empty.publish(Empty())
        else:
            self.a_down = False


    def run(self):
        # loop is neccesary to send regular updates
        rate = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():

            self.pub_twist.publish(self.twist)
            
            rate.sleep()


    def callback_joy(self, data):
        twist = Twist()
        twist.linear.x = data.axes[1]
        twist.angular.z = data.axes[0]

        if (twist.buttons[0] == 1):
            self.a_down = True
        else:
            self.a_down = False

        self.pub_twist.publish(twist)
        

if __name__ == '__main__':
    rospy.init_node('joy_node', anonymous=True)
    node = Node()
