#!/usr/bin/env python3
import rospy
import binascii
from sensor_msgs.msg import CompressedImage
from serial import Serial, serialutil
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from rospy.msg import AnyMsg
from io import BytesIO
from image_tools import ImageTools
import sys
import copy as copy_module
import cv2

class Node:
    def __init__(self):
        self.image_view = ImageTools()
        self.image_view.window_name = rospy.get_param("~window_name")
        self.image_view.display_width = int(rospy.get_param("~display_width"))
        self.image_view.save_name = "fullmap"

        fullmap_topic = rospy.get_param("~fullmap_topic")
        pose_topic = rospy.get_param("~pose_topic")
        empty_topic = rospy.get_param("~empty_topic")
        mark_topic = rospy.get_param("~mark_topic")

        self.ugv_position = Pose()
        self.nodepir_positions = []

        self.latest_map_cv2 = None

        self.pir_array = []

        self.mark = False

        self.sub_camera = rospy.Subscriber(fullmap_topic, CompressedImage, self.callback_map)
        rospy.loginfo("Fullmap_view - subscribed topic : " + fullmap_topic)

        self.sub_pose = rospy.Subscriber(pose_topic, PoseStamped, self.callback_pose)
        rospy.loginfo("Fullmap_view - subscribed topic : " + pose_topic)

        self.sub_mark = rospy.Subscriber(mark_topic, Empty, self.callback_mark)
        rospy.loginfo("Fullmap_view - subscribed topic : " + mark_topic)


    def callback_mark(self, empty):
        self.mark = True

    def callback_map(self, CompressedImage):
        self.latest_map_cv2 = self.image_view.convert_to_cv2(CompressedImage)
        self.update_display()

    def update_display(self):
        if self.latest_map_cv2 is None:
            return

        image_modded = self.latest_map_cv2.copy() 

        # BGR

        #cv2.circle(image, center_coordinates, radius, color, thickness)

        image_modded = cv2.circle(image_modded, (int(self.ugv_position.position.x), int(self.ugv_position.position.y)), 15, (255, 0, 0), -1)

        for position, activated in zip(self.nodepir_positions, self.pir_array):
            image_modded = cv2.circle(image_modded, (int(position.position.x), int(position.position.y)), 15, ((0, 0, 255) if activated else (0, 0, 150)), -1)

        self.image_view.display_image(image_modded)

    def callback_pose(self, poseStamped):
        if poseStamped.header.frame_id == "full" or self.mark:
            self.mark = False
            # deployment of new node
            self.nodepir_positions.append(poseStamped.pose)
        if poseStamped.header.frame_id == "tile":
            # update of ugv position from tile
            self.ugv_position = poseStamped.pose
        

        self.update_display()

    def callback_pir_string(self, string):
        self.pir_array = []
        for c in string:
            if c == "1":
                self.pir_array.append(True)
            else:
                self.pir_array.append(False)

        self.update_display()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('fullmap_view', anonymous=True)
    node = Node()
    node.run()
