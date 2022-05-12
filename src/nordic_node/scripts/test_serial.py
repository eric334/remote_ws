#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Empty
from serial import Serial, serialutil
from io import BytesIO
import struct
import numpy as np
from ctypes import *
import binascii
import sys

class Node:

    def __init__(self):

        valtest = [65535,65536]

        val = c_uint32(c_uint32(valtest[0] << 16).value | valtest[1]).value

        message = TwistStamped()
        message.header.seq = val
        buffer = BytesIO()
        message.serialize(buffer)

        print(message)


        firstval = c_uint16(val >> 16).value
        secondval = c_uint16(val ^ (firstval << 16)).value

        print(firstval)
        print(secondval)


        #print(binascii.hexlify(bytes(val)))

    def uint32_to_two_uint16(self, val):
        firstval = c_uint16(val >> 16).value
        return firstval, c_uint16(val ^ (firstval << 16)).value

    def two_uint16_to_uint32(self, firstval, secondval):
        return val = c_uint32(c_uint32(valtest[0] << 16).value | valtest[1]).value

        

if __name__ == '__main__':
    rospy.init_node('test_serial', anonymous=True)
    node = Node()

    

