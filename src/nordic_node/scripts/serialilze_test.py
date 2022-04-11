#!/usr/bin/env python3
import sys
import os
import time
from serial import Serial, serialutil
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from io import BytesIO
from pympler.asizeof import asizeof

class Node:
    def __init__(self):

        twist = Twist()
        twist.linear = createVector3([.3,.1,0])
        twist.angular = createVector3([.2,0,.1])

        print(twist)

        print(asizeof(twist))

        buffer = BytesIO()

        twist.serialize(buffer)

        print(buffer.getvalue())

        print(asizeof(buffer))

def createVector3(list):
    vector = Vector3()

    vector.x = list[0]
    vector.y = list[1]
    vector.z = list[2]

    return vector


if __name__ == '__main__':
    rospy.init_node('serialize_test', anonymous=True)
    node = Node()

