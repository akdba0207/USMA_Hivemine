#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import rospy
from geometry_msgs.msg import Point, PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import LatLongUTMconversion as ll
import tf

class DataCollectorNode:
    def __init__(self):

        # Initialize the ROS node
        rospy.init_node('data_collector', anonymous=True)

        # Create a publisher that will publish messages to the 'chatter' topic
        self.publisher = rospy.Publisher('/drone_pose_error', Point, queue_size=10)
        self.publisher2 = rospy.Publisher('/drone_utm_pose', PoseStamped,queue_size=10)
        self.pose_error = Point()
        self.utm_pose = PoseStamped()

        # Create a subscriber that will subscribe to the 'chatter' topic
        self.subscriber1 = rospy.Subscriber('/mavros/setpoint_position/global', GeoPoseStamped, self.callback1)
        self.subscriber2 = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.callback2)
        self.subscriber3 = rospy.Subscriber('/mavros/global_position/rel_alt', Float64, self.callback3)
        self.subscriber4 = rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.callback4)
        self.subscriber5 = rospy.Subscriber('/mavros/global_position/local', Odometry, self.callback5)

        # Set the rate at which messages will be published
        self.rate = rospy.Rate(50)  # 1 Hz
        self.input_Easting = 0
        self.input_Northing = 0
        self.measured_Easting = 0
        self.measured_Northing = 0
        self.input_alt = 0
        self.measured_alt = 0
        self.measured_hdg = 0
        self.roll=0
        self.pitch=0
        self.yaw = 0


    def publish_messages(self):
        while not rospy.is_shutdown():

            self.pose_error.x = self.input_Easting - self.measured_Easting
            self.pose_error.y = self.input_Northing - self.measured_Northing
            self.pose_error.z = self.input_alt - self.measured_alt

            self.utm_pose.header.stamp = rospy.Time.now()
            self.utm_pose.pose.position.x = self.measured_Easting
            self.utm_pose.pose.position.y = self.measured_Northing
            self.utm_pose.pose.position.z = self.measured_alt
            self.utm_pose.pose.orientation.x = self.roll / math.pi * 180.0
            self.utm_pose.pose.orientation.y = self.pitch / math.pi * 180.0
            self.utm_pose.pose.orientation.z = self.yaw / math.pi * 180.0
            self.utm_pose.pose.orientation.w = self.measured_hdg

            # Publish the message
            self.publisher.publish(self.pose_error)
            self.publisher2.publish(self.utm_pose)

            # Sleep for the specified rate
            self.rate.sleep()

    def callback1(self, data):
        self.input_lat = data.pose.position.latitude
        self.input_lon = data.pose.position.longitude
        self.input_alt = data.pose.position.altitude
        (self.zone, self.input_Easting, self.input_Northing) = ll.LLtoUTM(23, self.input_lat, self.input_lon)

    def callback2(self, data):
        self.measured_lat = data.latitude
        self.measured_lon = data.longitude
        (self.zone, self.measured_Easting, self.measured_Northing) = ll.LLtoUTM(23, self.measured_lat, self.measured_lon)

    def callback3(self,data):
        self.measured_alt = data.data

    def callback4(self,data):
        self.measured_hdg = data.data

    def callback5(self,data):
        ori_x = data.pose.pose.orientation.x
        ori_y = data.pose.pose.orientation.y
        ori_z = data.pose.pose.orientation.z
        ori_w = data.pose.pose.orientation.w

        euler = tf.transformations.euler_from_quaternion([ori_x,ori_y,ori_z,ori_w])
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]

if __name__ == '__main__':
    try:
        # Create an instance of the SimpleROSNode class
        simple_ros_node = DataCollectorNode()

        # Start publishing messages
        simple_ros_node.publish_messages()
    except rospy.ROSInterruptException:
        pass





