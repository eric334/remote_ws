#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def main_handler(data):
    rospy.loginfo(data.data)


def main_recv():
    rospy.init_node('main_recv', anonymous=True)
    rospy.Subscriber('main_line', String, main_handler)
    
    rospy.spin()

if __name__ == '__main__':
    main_recv()