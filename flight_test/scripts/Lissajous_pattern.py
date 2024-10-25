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

class LissajousNode:
    def __init__(self):
        # Pattern Name
        self._name = 'Lissajous'

        self.scale = 25

        self.omega_x = 5
        self.omega_y = 9

        offset_x = -25.0
        offset_y = 0.0

        self.home_lat = 41.3909084
        self.home_lon = -73.9529137
        self.home_alt = 15.0

        self.alt_change_thread = threading.Thread(target=self.change_altitude, daemon=True)
        self.alt_change_thread.start()

        # Northings for y value, Eastings for x value
        (self.zone, self.home_Easting, self.home_Northing) = ll.LLtoUTM(23, self.home_lat, self.home_lon)

        self.center_Easting = self.home_Easting + offset_x
        self.center_Northing = self.home_Northing + offset_y

        # (self.center_lat,self.center_lon) = ll.UTMtoLL(23, next_Northing, next_Easting, self.zone)

        # Northings for y value, Eastings for x value
        # (self.zone, self.center_Easting, self.center_Northing) = ll.LLtoUTM(23, self.center_lat, self.center_lon)

        self._starting_Northing = self.center_Northing
        # self._starting_Easting = self.center_Easting
        self._starting_Easting = self.center_Easting + self.scale

        # Convert meters coordinates back to lat/lon values
        (self.starting_lat, self._starting_lon) = ll.UTMtoLL(23, self._starting_Northing, self._starting_Easting, self.zone)


        self.drone_pose = GeoPoseStamped()
        self.drone_pose.pose.position.latitude = self.starting_lat
        self.drone_pose.pose.position.longitude = self._starting_lon
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
                # if ( (current_time <= self.stop_time) ):
                d_t = 1/3000

                prev_Northing = self.scale * math.sin((self.omega_y * d_t*(self.count-1)))
                prev_Easting = self.scale * math.sin((self.omega_x * d_t*(self.count-1)) + (math.pi/2))

                next_Northing = self.scale * math.sin((self.omega_y * d_t*self.count))
                next_Easting = self.scale * math.sin((self.omega_x * d_t*self.count) + (math.pi/2))

                # Calculate heading
                delta_x = next_Easting - prev_Easting
                delta_y = next_Northing - prev_Northing
                #heading = math.atan2(delta_y, delta_x)
                heading = 0.0

                next_Northing = self.center_Northing + next_Northing
                next_Easting = self.center_Easting + next_Easting

                (next_lat, next_lon) = ll.UTMtoLL(23, next_Northing, next_Easting, self.zone)

                self.drone_pose.pose.position.latitude = next_lat
                self.drone_pose.pose.position.longitude = next_lon

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
        simple_ros_node = LissajousNode()

        # Start publishing messages
        simple_ros_node.publish_messages()
    except rospy.ROSInterruptException:
        pass





