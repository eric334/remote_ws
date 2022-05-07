#!/usr/bin/env python3
import rospy
import binascii
import traceback
from sensor_msgs.msg import CompressedImage
from serial import Serial, serialutil
from rospy.msg import AnyMsg
from io import BytesIO
from image_tools import ImageTools
import sys

# recieve data messages from nordic, get message type and publish
class Node:
    def __init__(self):
        self.camera_image = ImageTools()

        camera_topic = rospy.get_param("~camera_topic")

        dev = rospy.get_param("~dev", "/dev/ttyACM0")
        baud = int(rospy.get_param("~baud", "115200"))
        
        rospy.loginfo("Nordic_recv - opening serial : " + dev)
        # TODO COPY SPECIFICS ?
        self.serial = Serial(dev, timeout=1, baudrate=baud)
        # TODO may need to close open

        self.pub_camera = rospy.Publisher(camera_topic, CompressedImage, queue_size = 1)
        rospy.loginfo("Nordic_recv - published topic : " + camera_topic)

    def run(self):
        #rate = rospy.Rate(100)

        message = b''
        last_packet = 0

        compressedImage = CompressedImage()

        while not rospy.is_shutdown():
            bytesToRead = self.serial.inWaiting()
            data = self.serial.read(bytesToRead)

            data = data[:64]
            
            if data[0:5] == b'start':
                print(binascii.hexlify(data))
                print(type(data))
                print("last_packet bytes: " + str(binascii.hexlify(data[5:6])))
                last_packet = int.from_bytes(data[5:6], 'big')
                print ("last_packet: " + str(last_packet))
                message = b''
            elif data[0:3] == b'end':
                
                message = message[:len(message) - 64 + last_packet]
                print("Entire message: \n " + str(binascii.hexlify(message)))

                print("done message")
                try:
                    buffer = BytesIO(message)
                    compressedImage.deserialize(buffer.getvalue())
                except:
                    rospy.logerr("Deserialization of message failed, traceback: \n" + traceback.format_exc())

                print(compressedImage)

                if compressedImage.header.frame_id == "cam":
                    self.pub_camera.publish(compressedImage)

                if compressedImage.header.frame_id == "cam":
                    self.pub_hector.publish(compressedImage)

            elif data != b'':
                #print(len(data))
                #print(binascii.hexlify(data))
                message += data
            
            # rate.sleep()


if __name__ == '__main__':
    rospy.init_node('nordic_recv', anonymous=True)
    node = Node()
    node.run()
