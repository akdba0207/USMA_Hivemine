#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import rospy
from geometry_msgs.msg import PoseStamped, Point
import LatLongUTMconversion as ll

class LissajousNode:
    def __init__(self):
        # Pattern Name
        self._name = 'Lissajous'

        self.scale = 25.0

        self.omega_x = 5
        self.freq_ratio = 0.1333333333
        # self.omega_y = self.omega_x / self.freq_ratio
        self.omega_y = 4

        # Start time of the pattern
        self.initial_time = time.time()
        self.previous_time = self.initial_time

        # End time of the pattern./run_cleanup.sh
        self.stop_time = ((2*math.pi * 19)/self.omega_x) # change this from hard-coded variables

        # Input altitude
        self._desired_alt = float(10)

        self.center_Northing = 0.0
        self.center_Easting = 0.0

        self._starting_Northing = self.center_Northing
        self._starting_Easting = self.center_Easting + self.scale

        self.drone_pose = PoseStamped()
        self.drone_pose.pose.position.x = self._starting_Easting
        self.drone_pose.pose.position.y = self._starting_Northing
        self.drone_pose.pose.position.z = 10.0

        # Initialize the ROS node
        rospy.init_node('pattern_node', anonymous=True)

        # Create a publisher that will publish messages to the 'chatter' topic
        self.publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.publisher.publish(self.drone_pose)

        # Create a subscriber that will subscribe to the 'chatter' topic
        self.subscriber = rospy.Subscriber('/HM24/SetMode', Point, self.callback)

        # Set the rate at which messages will be published
        self.rate = rospy.Rate(10)  # 1 Hz
        self.mode = 0
        self.count = 0

    def publish_messages(self):
        while not rospy.is_shutdown():
            current_time = time.time() - self.initial_time
            if (self.mode == 5):
                # if ( (current_time <= self.stop_time) ):
                d_t = 1/200

                # Python sine function uses radians, not degrees                   #- (math.pi/2)
                next_Northing = self.scale * math.sin((self.omega_y * d_t*self.count))
                next_Easting = self.scale * math.sin((self.omega_x * d_t*self.count) + (math.pi/2))
                # Next position is only based on the change in time from the last position. It is not based on the most recent position.

                next_Northing = self.center_Northing + next_Northing
                next_Easting = self.center_Easting + next_Easting

                print("current time ", current_time, self.count)
                self.previous_time = current_time

                self.drone_pose.pose.position.x = next_Easting
                self.drone_pose.pose.position.y = next_Northing
                self.drone_pose.pose.position.z = 10.0
                self.count +=1
                # Publish the message
            self.publisher.publish(self.drone_pose)

            # Sleep for the specified rate
            self.rate.sleep()
    def callback(self, data):
        self.mode = data.x


if __name__ == '__main__':
    try:
        # Create an instance of the SimpleROSNode class
        simple_ros_node = LissajousNode()

        # Start publishing messages
        simple_ros_node.publish_messages()
    except rospy.ROSInterruptException:
        pass





