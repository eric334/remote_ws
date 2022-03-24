#!/usr/bin/env python
import rospy
from std_msgs.msg import CompressedImage
from serial import Serial, serialutil
import StringIO
import sys

# recieve data messages from nordic, get message type and publish
class Node:
    def __init__(self):
        camera_topic = rospy.get_param("~camera_topic")
        hector_topic = rospy.get_param("~hector_topic")

        dev = rospy.get_param("~dev", "/dev/ttyACM0")
        baud = int(rospy.get_param("~baud", "115200"))
        
        rospy.loginfo("Nordic_recv - opening serial : " + dev)
        # TODO COPY SPECIFICS ?
        self.serial = Serial(dev, timeout=1, baudrate=baud)

        self.pub_camera = rospy.Publisher(camera_topic, CompressedImage)
        rospy.loginfo("Nordic_recv - published topic : " + camera_topic)
        self.pub_hector = rospy.Publisher(hector_topic, CompressedImage)
        rospy.loginfo("Nordic_recv - published topic : " + hector_topic)

    def run(self):
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            bytesToRead = self.serial.inWaiting()
            data = self.serial.read(bytesToRead)

            message_type = str(data._type)

            if message_type == "sensor_msgs/CompressedImage":
                
                if data.header.frame_id = "cam":
                    self.pub_camera.publish(data)

                elif data.header.frame_id = "hec":
                    self.pub_hector.publish(data)

                else:
                    rospy.logdebug("Error: unrecognized frame id found: "+ data.header.frame_id)
            else:
                rospy.logdebug("Error: unrecognized message type found: "+message_type)
            
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('nordic_recv', anonymous=True)
    node = Node()
    node.run()
