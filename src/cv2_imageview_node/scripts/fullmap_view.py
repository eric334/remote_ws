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

class Node:
    def __init__(self):
        self.image_view = ImageTools()
        self.image_view.window_name = rospy.get_param("~window_name")
        self.image_view.display_width = int(rospy.get_param("~display_width"))

        fullmap_topic = rospy.get_param("~fullmap_topic")
        pose_topic = rospy.get_param("~pose_topic")

        self.ugv_position = Pose()
        self.nodepir_positions = []

        self.latest_map = None

        self.pir_array = []

        self.sub_camera = rospy.Subscriber(fullmap_topic, CompressedImage, self.callback_map)
        rospy.loginfo("Fullmap_view - subscribed topic : " + fullmap_topic)

        self.sub_pose = rospy.Subscriber(pose_topic, PoseStamped, self.callback_pose)
        rospy.loginfo("Fullmap_view - subscribed topic : " + pose_topic)

    def callback_map(self, CompressedImage):
        self.latest_map = CompressedImage
        update_display()

    def update_display(self):
        if self.latest_map is None:
            return

        image_copy = CompressedImage(self.latest_map)

        image_copy = self.draw_square(image_copy, (self.ugv_position.position.x, self.ugv_position.position.y), 6, (255, 255, 0))
        
        for position, activated in zip(nodepir_positions, pir_array):
            image_copy = self.draw_square(image_copy, (position.position.x, position.position.y), 6, ((255, 0, 0) if activated else (150, 0, 0)))

        self.image_view.display_image(CompressedImage)

    def callback_pose(self, poseStamped):
        if poseStamped.header.frame_id == "tile":
            # update of ugv position from tile
            self.ugv_position = poseStamped.pose
        if poseStamped.header.frame_id == "full":
            # deployment of new node
            self.nodepir_positions.append(poseStamped.pose)

        update_display()

    def callback_pir_string(self, string):
        self.pir_array = []
        for c in string:
            if c == "1":
                self.pir_array.append(True)
            else:
                self.pir_array.append(False)

        update_display()

    def draw_point(compressedImage, position, color):
        compressedImage.data[position[0]][position[1]][:] = color
        return compressedImage

    def draw_square(compressedImage, position, radius, color):
        shape = compressedImage.data.shape
        for i in range(pos[0] - radius, pos[0] + radius):
           for j in range(position[1] - radius, position[1] + radius): 
               if position[1] >= 0 and position[0] < shape[0] and position[1] >= 0 and position[1] < shape[1]:
                   compressedImage = draw_point(compressedImage, (i,j), color)
        return compressedImage

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('fullmap_view', anonymous=True)
    node = Node()
    node.run()
