#!/usr/bin/env python3
import rospy
import binascii
from sensor_msgs.msg import CompressedImage
from serial import Serial, serialutil
from rospy.msg import AnyMsg
from io import BytesIO
from image_tools import ImageTools
import sys

class Node:
    def __init__(self):
        self.image_view = ImageTools()
        self.image_view.window_name = rospy.get_param("~window_name")
        self.image_view.display_width = int(rospy.get_param("~display_width"))

        topic = rospy.get_param("~topic")

        self.sub_camera = rospy.Subscriber(topic, CompressedImage, self.callback)
        rospy.loginfo("Camera_view - subscribed topic : " + topic)

    def callback(self, CompressedImage):
        self.image_view.display_image(CompressedImage)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('camera_view', anonymous=True)
    node = Node()
    node.run()
