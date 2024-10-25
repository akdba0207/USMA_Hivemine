cmake_minimum_required(VERSION 2.8.3)
project(flight_test)

#Author : Dongbin Kim
#Title : MM-UAV flight test for Dongbin Kim's work.

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES flight_test
#  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


#Drone Avatar
add_executable(hm24_response src/flight_mode.cpp)

target_link_libraries(hm24_response
  ${catkin_LIBRARIES}
)

add_executable(hm24_da_mode src/mode_input.cpp)

target_link_libraries(hm24_da_mode
  ${catkin_LIBRARIES}
)

