#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import rospy
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoseStamped
import LatLongUTMconversion as ll
import tf
import threading

class CircleNode:
    def __init__(self):
        # Pattern Name
        self._name = 'Yawonly'

        self.scale = 2.0

        offset_x = 0.0
        offset_y = 0.0

        self.home_lat = 41.3909669
        self.home_lon = -73.9531104
        self.home_alt = 15.0

        self.alt_change_thread = threading.Thread(target=self.change_altitude, daemon=True)
        self.alt_change_thread.start()
       
        
        self.drone_pose = GeoPoseStamped()
        self.drone_pose.pose.position.latitude = self.home_lat
        self.drone_pose.pose.position.longitude = self.home_lon
        self.drone_pose.pose.position.altitude = self.home_alt
        self.drone_pose.pose.orientation.w = 1.0
        self.drone_pose.pose.orientation.x = 0.0
        self.drone_pose.pose.orientation.y = 0.0
        self.drone_pose.pose.orientation.z = 0.0

        # Initialize the ROS node
        rospy.init_node('pattern_node', anonymous=True)

        # Create a publisher that will publish messages to the 'chatter' topic
        self.publisher = rospy.Publisher('/mavros/setpoint_position/global', GeoPoseStamped, queue_size=10)
        self.publisher.publish(self.drone_pose)

        # Create a subscriber that will subscribe to the 'chatter' topic
        self.subscriber = rospy.Subscriber('/HM24/SetMode', Point, self.callback)

        # Set the rate at which messages will be published
        self.rate = rospy.Rate(50)  # 1 Hz
        self.mode = 0
        self.count = 0
        self.adj_altitude = 0.0

    def publish_messages(self):
        while not rospy.is_shutdown():
            if (self.mode == 5):

                d_t = 0.005
                
                heading = d_t * self.count
                print(heading)

                self.drone_pose.pose.position.latitude = self.home_lat
                self.drone_pose.pose.position.longitude = self.home_lon

                # Convert heading to quaternion
                quaternion = tf.transformations.quaternion_from_euler(0, 0, heading)

                self.drone_pose.pose.orientation.x = quaternion[0]
                self.drone_pose.pose.orientation.y = quaternion[1]
                self.drone_pose.pose.orientation.z = quaternion[2]
                self.drone_pose.pose.orientation.w = quaternion[3]


                self.count +=1
                # Publish the message

            self.drone_pose.pose.position.altitude = self.home_alt + self.adj_altitude
            self.publisher.publish(self.drone_pose)

            # Sleep for the specified rate
            self.rate.sleep()

    def callback(self, data):
        self.mode = data.x

    def change_altitude(self):
        while not rospy.is_shutdown():
            self.adj_altitude = float(input("Adjust altitude "))

if __name__ == '__main__':
    try:
        # Create an instance of the SimpleROSNode class
        simple_ros_node = CircleNode()

        # Start publishing messages
        simple_ros_node.publish_messages()
    except rospy.ROSInterruptException:
        pass





