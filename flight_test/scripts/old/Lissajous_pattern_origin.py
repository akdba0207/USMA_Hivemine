import numpy as np
import math
import timeit
import time
import random
import LatLongUTMconversion as ll



#extra libraries
import copy
import subprocess
# from matplotlib import pyplot as plot


# The program is published to multiple drones at a time. 
# Three instances of this program will be running during the flight demonstration. 
class Lissajous(ss.Tactic):

    def init(self, params):
        # Pattern Name
        self._name = 'Lissajous'
        # Create the starting point based on the drones id
        # self._id = int(params['id'])

        # The following 3 variables will be changed to parameters that users can choose:
        # self.scale = int(params['Search Area'])# 10         # Dictates size of the search area (scale = 10 means search area will be 20m x 20m)

        self.scale = 10
        # self.num_drones = int(params['Number of Drones'])

        # self.omega_x = float(params['Omega X'])
        self.omega_x = 5
        self.freq_ratio = 0.1333333333
        self.omega_y = self.omega_x / self.freq_ratio
        

        # Start time of the pattern
        self.initial_time = time.time()
        self.previous_time = self.initial_time

        # End time of the pattern./run_cleanup.sh
        self.stop_time = ((2*math.pi * 19)/self.omega_x) # change this from hard-coded variables

        # Input altitude
        self._desired_alt = float(10)
        self.alt_offset = float(3)
        self._desired_alt = self._desired_alt - self.alt_offset + ((self._id%self.num_drones) * self.alt_offset)      # This will offset the drones' altitudes. It is based on the tail numbers of the drones, so it may not be reliable.
        # (Generally want to set altitude to 10-15 meters in accordance with our constraints)
        print("drone ID ", self._id)

        # The lat and long (define the search area's center latitude and longitude coordinates)
        # self.center_lat = float(params['Lat'])
        # self.center_lon = float(params['Lon'])
        
        # Lines 41 - 56 set the starting coordinates of each drone

        # Northings for y value, Eastings for x value
        # (self.zone, self.center_Easting, self.center_Northing) = ll.LLtoUTM(23, self.center_lat, self.center_lon)

        self.center_Northing = 0
        self.center_Easting = 0
        # This depends on the tail numbers (subtracting 1 may not be necessary)
        # self._starting_Northing = self.center_Northing + self.scale * math.sin((self._id%self.num_drones) * (2*math.pi) / self.num_drones)
        # self._starting_Easting = self.center_Easting + self.scale * math.sin((self._id%self.num_drones) * (2*math.pi) / self.num_drones)

        self._starting_Northing = self.center_Northing + self.scale
        self._starting_Easting = self.center_Easting + self.scale
        # print("next ", self._starting_Northing, self._starting_Easting)

        # Convert meters coordinates back to lat/lon values
        # (self.starting_lat, self._starting_lon) = ll.UTMtoLL(23, self._starting_Northing, self._starting_Easting, self.zone)

        # The next (x,y) positions
        # self._desired_lat = self.starting_lat       # y position
        # self._desired_lon = self._starting_lon      # x position

        # self._wp = [self._desired_lat, self._desired_lon, self._desired_alt]

        

    def generate_waypoint_for3(self):   
        # Time is the only variable in the set of Lissajous equations that changes while the program runs
        current_time = time.time() - self.initial_time
        d_t = current_time - self.previous_time

        # Python sine function uses radians, not degrees                   #- (math.pi/2)
        next_Northing = self.scale * math.sin((self.omega_y * current_time) - (math.pi/2) + (self._id % self.num_drones)* (2*math.pi) / self.num_drones)
        next_Easting = self.scale * math.sin((self.omega_x * current_time) + (math.pi/2) + (self._id % self.num_drones) * (2*math.pi) / self.num_drones)
        # Next position is only based on the change in time from the last position. It is not based on the most recent position. 

        next_Northing = self.center_Northing + next_Northing
        next_Easting = self.center_Easting + next_Easting


        print("Meters coordinates", next_Northing, next_Easting, "center coords ", self.center_Northing, self.center_Easting)

        #####################################################################################################################
        # Convert (x,y) meters coordinates back to lon/lat coordinates
        (next_lat, next_lon) = ll.UTMtoLL(23, next_Northing, next_Easting, self.zone)

        print("self.id followed by next lat and lon", self._id, next_lat, next_lon)###########
        # The next (x,y) positions
        self._desired_lon = next_lon   # x pos
        self._desired_lat = next_lat   # y pos


        print("current time ", current_time, d_t)
        self.previous_time = current_time

        self._wp = [self._desired_lat, self._desired_lon, self._desired_alt]



    # Determine when to alter pattern from most efficient for three drones to most efficient for two
    # Check flight status of all drones throughout duration of the flight; if one goes down, switch pattern (all that changes between for3 and for2 is the coefficients and 2pi/(# of drones))


    # stepautonomy() is the main function of the program. It just calls the "generate waypoints" functions. Potential modifications are:
        # If a drone falls out, change the pattern
        # With all drones flying at the same altitude, check for collisions before generating waypoints


    # This main function will continue generating waypoints until the pattern is "complete"
    def step_autonomy(self, t, dt):
        current_time = time.time() - self.initial_time

        # Plot the waypoints to get a clear visual of the pattern (will remove this conditional statement later)
        if ( (current_time <= self.stop_time) ):   # (current_time == self.stop_time) == False
            self.generate_waypoint_for3() 
        # else: tell drones to land
        
        return True







