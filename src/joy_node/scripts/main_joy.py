#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class joyClass:

    def __init__(self):
        
        self._sub = rospy.Subscriber('joy', Joy, self._callback)
        self._pub = rospy.Publisher('main_line', Twist, queue_size=1)

        rospy.spin()

    def _callback(self, data):
        twist = Twist()
        twist.linear.x = data.axes[1]
        twist.angular.z = data.axes[0]

        self._pub.publish(twist)
        

if __name__ == '__main__':
    rospy.init_node('main_joy', anonymous=True)
    joyclass = joyClass()
