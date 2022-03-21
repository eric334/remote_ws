#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('main', anonymous=True)
    pub = rospy.Publisher('main_line', String, queue_size=1)
    
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        message = "hello world"
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass