#!/usr/bin/env python3
import sys
import os
import time
from serial import Serial, serialutil
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import CompressedImage
from io import BytesIO
from pympler.asizeof import asizeof
import pickle

class Node:
    def __init__(self):

        obj = Twist()
        obj.linear = createVector3([.3,.1,0])
        obj.angular = createVector3([.2,0,.1])

        print (obj)

        print(asizeof(obj))

        buffer = BytesIO()
        obj.serialize(buffer)

        print(bytes(buffer.getvalue(), 'utf-8'))

        print(asizeof(buffer.getvalue()))

        first = buffer.getvalue()[:1]

        print(first)

        print(asizeof(first))

def createVector3(list):
    vector = Vector3()

    vector.x = list[0]
    vector.y = list[1]
    vector.z = list[2]

    return vector


if __name__ == '__main__':
    rospy.init_node('serialize_test', anonymous=True)
    node = Node()

