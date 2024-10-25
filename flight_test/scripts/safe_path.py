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
import pandas as pd

class SafeNode:
    def __init__(self):
        # Pattern Name
        self._name = 'safe'

        self.home_alt = 15.0

        self.alt_change_thread = threading.Thread(target=self.change_altitude, daemon=True)
        self.alt_change_thread.start()

        self.waypoints = pd.read_csv('/home/dbkros/catkin_ws/src/flight_test/scripts/Output_Path.csv').values.tolist()


        self.drone_pose = GeoPoseStamped()
        self.drone_pose.pose.position.latitude = self.waypoints[0][0]
        self.drone_pose.pose.position.longitude = self.waypoints[0][1]
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
        self.rate = rospy.Rate(10)  # 1 Hz
        self.mode = 0
        self.count = 0
        self.adj_altitude = 0.0
        self.current_waypoint_index = 1

    def publish_messages(self):
        while not rospy.is_shutdown():
            if self.mode == 5:
                if self.current_waypoint_index < len(self.waypoints):

                    prev_lat = self.waypoints[self.current_waypoint_index - 1][0]
                    prev_lon = self.waypoints[self.current_waypoint_index - 1][1]
                    (self.zone, prev_Easting, prev_Northing) = ll.LLtoUTM(23, prev_lat, prev_lon)

                    next_lat = self.waypoints[self.current_waypoint_index][0]
                    next_lon = self.waypoints[self.current_waypoint_index][1]
                    (self.zone, next_Easting, next_Northing) = ll.LLtoUTM(23, next_lat, next_lon)

                    self.drone_pose.pose.position.latitude = next_lat
                    self.drone_pose.pose.position.longitude = next_lon

                    delta_x = next_Easting - prev_Easting
                    delta_y = next_Northing - prev_Northing
                    print(delta_y)
                    heading = math.atan2(delta_y, delta_x)

                   # heading = 0.0  # No heading change for the first waypoint

                    # Convert heading to quaternion
                    quaternion = tf.transformations.quaternion_from_euler(0, 0, heading)

                    self.drone_pose.pose.orientation.x = quaternion[0]
                    self.drone_pose.pose.orientation.y = quaternion[1]
                    self.drone_pose.pose.orientation.z = quaternion[2]
                    self.drone_pose.pose.orientation.w = quaternion[3]

                    # Move to the next waypoint
                    self.current_waypoint_index += 1

                    self.count += 1

            # Keep the altitude adjustment
            self.drone_pose.pose.position.altitude = self.home_alt + self.adj_altitude

            # Publish the message
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
        simple_ros_node = SafeNode()

        # Start publishing messages
        simple_ros_node.publish_messages()
    except rospy.ROSInterruptException:
        pass
