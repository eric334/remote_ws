#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Empty
from serial import Serial, serialutil
from io import BytesIO
import struct
import binascii
import direct_client as direct_client_

class Node:

    def __init__(self):
        twist_topic = rospy.get_param("~twist_topic")
        empty_topic = rospy.get_param("~empty_topic")
        reply_topic = rospy.get_param("~reply_topic")
        
        control_info_topic = rospy.get_param("~control_info_topic")

        self.direct_client = rospy.get_param("~direct_client")
        self.enable = rospy.get_param("~enable")
        if self.direct_client:
            self.enable = False
            self.direct_client = direct_client_.Client(6001)
            
        rospy.loginfo("Nordic_send - serial enabled : " + str(self.enable))

        self.twist = Twist()

        self.send_deploy = False

        self.dev = rospy.get_param("~dev", "/dev/ttyACM0")
        self.baud = int(rospy.get_param("~baud", "115200"))

        rospy.loginfo("Nordic_send - opening serial : " + self.dev)
        # TODO COPY SPECIFICS

        if self.enable:
            self.serial = Serial(self.dev, timeout=1, baudrate=self.baud)

        # only need a callback on reply

        self.sub_twist = rospy.Subscriber(twist_topic, Twist, self.callback_twist)
        rospy.loginfo("Nordic_send - subscribed to topic : " + twist_topic)
        
        self.sub_empty = rospy.Subscriber(empty_topic, Empty, self.callback_empty)
        rospy.loginfo("Nordic_send - subscribed to topic : " + empty_topic)
        
        self.sub_reply = rospy.Subscriber(reply_topic, Empty, self.callback_reply)
        rospy.loginfo("Nordic_send - subscribed to topic : " + reply_topic)
        
        self.pub_control = rospy.Publisher(control_info_topic, TwistStamped, queue_size=1)
        rospy.loginfo("Nordic_send - published topic : " + control_info_topic)

    def run(self):
        rospy.spin()

        if self.direct_client:
            self.direct_client.close_socket()

    def callback_reply(self, empty):
        rospy.loginfo("Nordic_send - sending reply")

        control_twist = TwistStamped()
        control_twist.twist = self.twist
        control_twist.header.frame_id = "con"

        if self.send_deploy:
            control_twist.header.frame_id = "condep"
            self.send_deploy = False

        self.pub_control.publish(control_twist)
        
        rospy.loginfo("Nordic_send - writing buffer")

        self.write_buffer(control_twist)


    def callback_twist(self, data):
        self.twist = data

    def callback_empty(self,empty):
        self.send_deploy = True

    def write_buffer(self, message):
        buffer = BytesIO()
        message.serialize(buffer)

        #rospy.loginfo("buffer size: " + str(len(buffer.getvalue())))

        #rospy.loginfo("numpy size: " + str(sys.getsizeof(compressedImage.data)))

        # getbuffer on python 3, getvalue on python 2

        if self.enable:
            self.send_as_chunks(buffer.getbuffer())
        if self.direct_client:
            print(str(len(buffer.getbuffer())))
            print(binascii.hexlify(buffer.getbuffer()))
            self.direct_client.send_data(buffer.getbuffer())

    def send_as_chunks(self, data):

        size = len(data)
        
        #print("Size: " + str(size))
        #print("Entire message: \n" + binascii.hexlify(data))
        
        n = 64
        chunks = [data[i:i+n] for i in range(0, size, n)]

        last_packet = len(chunks[-1])
        # does this work on python 3?
        last_packet_byte = bytes(struct.pack("B", last_packet))

        chunks.insert(0,b'start'+last_packet_byte)
        chunks.append(b'end')

        for chunk in chunks:
            self.serial.write(chunk)
            # this is required, for some stupid reason
            self.serial = Serial(self.dev, timeout=1, baudrate=self.baud)
            #print(str(len(chunk)))
            print (binascii.hexlify(chunk))

if __name__ == '__main__':
    rospy.init_node('nordic_send', anonymous=True)
    node = Node()
    node.run()

    

