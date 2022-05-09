#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import sys

# recieve data messages from nordic, get message type and publish
class Node:
    def __init__(self):
        control_info_topic = rospy.get_param("~control_info_topic")
        pir_string_topic = rospy.get_param("~pir_string_topic")

        self.num_nodes = 0
        self.last_send_time = None
        self.updates_received = 0
        self.pir_data = ""
        self.linear, self.angular, self.deploy_node = 0, 0, 0
        self.round_trip_time = ""

        self.sub_control_info = rospy.Subscriber(control_info_topic, TwistStamped, self.callback_stamped)
        rospy.loginfo("Status_node - subscribed to topic : " + control_info_topic)
        self.sub_string_topic = rospy.Subscriber(pir_string_topic, String, self.callback_pir_string)
        rospy.loginfo("Status_node - subscribed to topic : " + pir_string_topic)
        # its bad, I know
        rospy.Rate(2).sleep()
        self.log_status()

    def run(self):
        rospy.spin()
    
    def log_status(self):
        string = "\n\n\n    Remote Station Status\n"
        string += f"\n"
        string += f"Sent Control Data :\n"
        string += f"    Linear : {self.linear}\n"
        string += f"    Angular : {self.angular}\n"
        string += f"    Deploy Node : {self.deploy_node}\n"
        string += f"\n"
        string += f"Updates Received : {self.updates_received}\n"
        string += f"Round Trip Time : {str(self.round_trip_time)}\n"
        string += f"Nodes Online : {str(self.num_nodes)}\n"
        string += f"\n"
        string += f"Passive Infrared Sensors :\n"
        for i, item in enumerate(self.pir_data):
            string += f"    Node {i+1} : {str(bool(item))}\n"
        string += f"\n"

        rospy.loginfo(string)

    def callback_stamped(self, control_data):
        current_time = rospy.get_time()
        self.round_trip_time = "" if self.last_send_time is None else current_time - self.last_send_time
        self.last_send_time = current_time

        self.linear = control_data.twist.linear.x
        self.angular = control_data.twist.angular.z
        self.deploy_node = True if control_data.header.frame_id == "condep" else False

        self.log_status()

    def callback_pir_string(self, string):
        self.updates_received += 1
        self.num_nodes = len(string.data)
        self.pir_data = list(map(int, string.data))


if __name__ == '__main__':
    rospy.init_node('status_node', anonymous=True)
    node = Node()
    node.run()

