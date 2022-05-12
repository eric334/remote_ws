#!/usr/bin/env python3
import rospy
import binascii
import traceback
from std_msgs.msg import Empty
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from serial import Serial, serialutil
from rospy.msg import AnyMsg
from io import BytesIO
import sys
import numpy as np

# recieve data messages from nordic, get message type and publish
class Node:
    def __init__(self):
        self.enable = rospy.get_param("~enable")
        self.enable_reply_ticks = rospy.get_param("~enable_reply_ticks")
        rospy.loginfo("Nordic_recv - serial enabled : " + str(self.enable))
        rospy.loginfo("Nordic_recv - reply ticks enabled : " + str(self.enable_reply_ticks))

        camera_topic = rospy.get_param("~camera_topic")
        tilemap_topic = rospy.get_param("~tilemap_topic")
        fullmap_topic = rospy.get_param("~fullmap_topic")
        reply_topic = rospy.get_param("~reply_topic")
        pir_string_topic = rospy.get_param("~pir_string_topic")
        pose_topic = rospy.get_param("~pose_topic")

        dev = rospy.get_param("~dev", "/dev/ttyACM0")
        baud = int(rospy.get_param("~baud", "115200"))

        self.last_send_time = None
        self.total_updates = 0
        
        if self.enable:
            rospy.loginfo("Nordic_recv - opening serial : " + dev)
            self.serial = Serial(dev, timeout=1, baudrate=baud)

        self.pub_reply = rospy.Publisher(reply_topic, Empty, queue_size = 1)
        rospy.loginfo("Nordic_recv - published topic : " + reply_topic)

        self.pub_camera = rospy.Publisher(camera_topic, CompressedImage, queue_size = 1)
        rospy.loginfo("Nordic_recv - published topic : " + camera_topic)
        
        self.pub_tilemap = rospy.Publisher(tilemap_topic, CompressedImage, queue_size = 1)
        rospy.loginfo("Nordic_recv - published topic : " + tilemap_topic)
        
        self.pub_fullmap = rospy.Publisher(fullmap_topic, CompressedImage, queue_size = 1)
        rospy.loginfo("Nordic_recv - published topic : " + tilemap_topic)
        
        self.pub_pir_string = rospy.Publisher(pir_string_topic, String, queue_size = 1)
        rospy.loginfo("Nordic_recv - published topic : " + pir_string_topic)
        
        self.pub_pose = rospy.Publisher(pose_topic, PoseStamped, queue_size = 1)
        rospy.loginfo("Nordic_recv - published topic : " + pose_topic)

    def run(self):
        # do nothing if disabled but stay alive
        if not self.enable:
            if self.enable_reply_ticks:
                rate = rospy.Rate(3) # hz
                while not rospy.is_shutdown():
                    self.pub_reply.publish(Empty())
                    rate.sleep()
            else:
                rospy.spin()
            return

        #rate = rospy.Rate(100)

        message = b''
        last_packet = 0

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
                #print("Entire message: \n " + str(binascii.hexlify(message)))

                compressedImage = CompressedImage()

                try:
                    buffer = BytesIO(message)
                    compressedImage.deserialize(buffer.getvalue())
                except:
                    rospy.logerr("Deserialization of message failed, traceback: \n" + traceback.format_exc())

                #print(compressedImage)

                if compressedImage.header.frame_id == "cam":
                    self.pub_camera.publish(compressedImage)
                elif compressedImage.header.frame_id == "tile":
                    # get pose data
                    self.publish_pose(compressedImage.header.seq, "tile")
                    self.pub_tilemap.publish(compressedImage)
                elif compressedImage.header.frame_id == "full":
                    # get pose data
                    self.publish_pose(compressedImage.header.seq, "full")
                    self.pub_fullmap.publish(compressedImage)
                else:
                    rospy.logerr("Error: unrecognized frame id found: "+ compressedImage.header.frame_id)

                # get node and pir string from end of end packet
                num_nodes = int.from_bytes(data[3:4], 'big')
                pir_string = String()
                pir_string.data = data[4:4+num_nodes].decode("utf-8")
                self.pub_pir_string.publish(pir_string)

                # initiate send back message
                self.pub_reply.publish(Empty())

            elif data != b'':
                #print(len(data))
                #print(binascii.hexlify(data))
                message += data
            
            # rate.sleep()

    def publish_pose(self, uint32, maptype):
        # get pose from uint32 pid num
        pose = PoseStamped()
        pose.header.frame_id = maptype
        pose.pose.position.x, pose.pose.position.y = uint32_to_two_uint16(uint32)
 
        self.pub_pose.publish(pose)

    def uint32_to_two_uint16(self, val):
    firstval = c_uint16(val >> 16).value
    return firstval, c_uint16(val ^ (firstval << 16)).value


if __name__ == '__main__':
    rospy.init_node('nordic_recv', anonymous=True)
    node = Node()
    node.run()

