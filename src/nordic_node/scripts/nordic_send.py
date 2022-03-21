#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from serial import Serial, serialutil

import dynamic_reconfigure.client

class Node:

    def __init__(self):

        camera_topic = rospy.get_param("~camera_topic")
        hector_topic = rospy.get_param("~hector_topic")
        jpeg_quality = rospy.get_param("~jpeg_quality_level")

        rospy.loginfo("Nordic_send - topic : " + camera_topic)
        rospy.loginfo("Nordic_send - topic : " + hector_topic)

        self.set_compressedimage_quality(camera_topic, jpeg_quality)
        self.set_compressedimage_quality(hector_topic, jpeg_quality)

        dev = rospy.get_param("~dev", "/dev/ttyACM0")
        baud = int(rospy.get_param("~baud", "115200"))

        rospy.loginfo("Nordic_send - opening serial : " + dev)
        # TODO COPY SPECIFICS
        #self.serial = Serial(dev, timeout=1, baudrate=baud)

        self.sub_camera = rospy.Subscriber(camera_topic, CompressedImage, self.callback_camera)
        rospy.loginfo("Nordic_send - subscribed to topic : " + camera_topic)
        self.sub_hector = rospy.Subscriber(hector_topic, CompressedImage, self.callback_hector)
        rospy.loginfo("Nordic_send - subscribed to topic : " + hector_topic)

    def run(self):
        rospy.spin()

    # modify headers so they are identifiable on the other end
    def callback_hector(self, compressedImage):
        compressedImage.header.frame_id = "hec"
        self.write_serial(compressedImage)
    
    # modify headers so they are identifiable on the other end
    def callback_camera(self, compressedImage):
        compressedImage.header.frame_id = "cam"
        self.write_serial(compressedImage)

    def write_serial(self, data):
        #self.serial.write(data)
        return

    def set_compressedimage_quality(self, topic, quality):
            client = dynamic_reconfigure.client.Client(topic, timeout = 3)
            client.update_configuration({ 'format' : 'jpeg', 'jpeg_quality' : int(quality)})

if __name__ == '__main__':
    rospy.init_node('nordic_send', anonymous=True)
    node = Node()
    node.run()

    

