
# USMA Swarm Capstone - Hivemine

Contributor
* Dongbin Kim (dbk@hartford.edu)
* Josiah Steckenrider (john.steckenrider@westpoint.edu)

## Requirements

This package requires two computers (one PC or Laptop and one Single Board Computer).

* Ground Station Computer (GSC):  [Ubuntu 20.04 LTS](https://releases.ubuntu.com/focal/) with [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu), [PX4](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html), [MAVROS](https://docs.px4.io/main/en/ros/mavros_installation.html), [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)
    
* Drone Onboard Computer (DC): 
    * Software: [Ubuntu 20.04 LTS](https://releases.ubuntu.com/focal/) with [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu),  [MAVROS](https://docs.px4.io/main/en/ros/mavros_installation.html)
    * Hardware: [ODroid XU4](https://wiki.odroid.com/odroid-xu4/odroid-xu4)

* Check [GCS_DC_Network_Setup.md](https://github.com/akdba0207/USMA_Hivemine/blob/main/GCS_DC_Network_Setup.md) to establish SSH connection over Wifi

## Ground Station Computer (GSC) Preparation

* Install Ubuntu 20.04
* Install ROS Noetic (ros-noetic-desktop-full)
* Install QGround Control 
* Install following packages to meet dependencies (PX4, MAVROS, etc)
        
        (PX4)
        git clone https://github.com/PX4/PX4-Autopilot.git --recursive
        bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx
        sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
        (MAVROS)
        sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs
        wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
        sudo bash ./install_geographiclib_datasets.sh
        (OpenCV)
        sudo apt install python3-opencv
        
* Install following python packages (pip)

        sudo apt install python3-pip
        pip install pandas
* Reboot the PC

        sudo reboot

* Download and unzip the USMA_VRTeleM pacakge in your ROS workspace, then build the code

        cd catkin_ws/src
        unzip {the package}
        cd ..
        catkin_make
        cd /src/flight_test/scripts
        chmod +x 
        cd ../src/vision_processing/scripts
        chmod +x        


## Drone Onboard Computer (DC) Preparation

* Install Ubuntu 20.04
* Install ROS Noetic (ros-noetic-desktop-full)
* Install following packages to meet dependencies (MAVROS, etc)   

        (MAVROS)
        sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs
        wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
        sudo bash ./install_geographiclib_datasets.sh
        (OpenCV)
        sudo apt install python3-opencv
        sudo usermod -a -G video $LOGNAME 

* Install following python packages (pip)

        sudo apt install python3-pip
        pip install pandas

* Reboot the PC

        sudo reboot
* Update and upgrade the debian pacakges

        sudo apt update
        sudo apt upgrade
* Download and unzip the USMA_VRTeleM pacakge in your ROS workspace, then build the code

        cd catkin_ws/src
        unzip {the package}
        cd ..
        catkin_make
        cd /src/flight_test/scripts
        chmod +x 
        cd ../src/vision_processing/scripts
        chmod +x 

## Operation in the air

Make sure to have both GSC and DC are on the same network over Alfa Wireless USB!

#### Pixhawk Side (not DC)

* Open QGroundControl
* Set the following parameter 

        MAV_1_CONFIG = TELEM2   # If TELEM2 is not an option enter 102
* Reboot Pixhawk, set the following paramters

        MAV_1_MODE = Onboard   # If Onboard is not an option enter 2
        SER_TEL2_BAUD = 921600
        MAV_1_RATE = 0
        MAV_1_FORWARD = Disabled
        MPC_LAND_SPEED = 0.6


#### GSC Side

* Open QGroundControl to check the drone status over Telemetry

* Open Terminal (T1) - Connect DC to Pixhawk over MAVROS

        ssh drone@192.168.11.{tail_number}
        source /opt/ros/noetic/setup.bash
        source ~/catkin_ws/devel/setup.bash
        
        (Do the below 3 lines to set the serial port to USB, once you do this, skip this command for the next operation)
        roscd mavros/launch
        sudo nano px4.launch
        **edit this line** <arg name="fcu_url" default="/dev/ttyUSB0:921600" />
        (Save and exit)
        roslaunch mavros px4.launch

* Open Terminal (T2) - Send the command to the drone (1: Arm, 2: OFFBOARD (Takeoff), 3: Landing, 4: Disarm, 5: Mission)

        ssh drone@192.168.11.{tail_number}
        source /opt/ros/noetic/setup.bash
        source ~/catkin_ws/devel/setup.bash
        rosrun flight_test hm24_da_mode

* Open Terminal (T3) - Receive the commmand from hm24_da_mode, control the drone

        ssh drone@192.168.11.{tail_number}
        source /opt/ros/noetic/setup.bash
        source ~/catkin_ws/devel/setup.bash
        rosrun flight_test hm24_response

* Open Terminal (T4) - Run Lissajous Pattern flight missions. You can also adjust the flight altitude

        ssh drone@192.168.11.{tail_number}
        source /opt/ros/noetic/setup.bash
        source ~/catkin_ws/devel/setup.bash
        (You can adjust the home position)
        roscd flight_test/scripts/
        nano Lissajous_pattern.py
        **Line 26-28 (below)** 
        self.home_lat = 41.3909084
        self.home_lon = -73.9529137
        self.home_alt = 15.0
        (Change, Save, and exit)
        rosrun flight_test Lissajous_pattern.py

* Open Terminal (T5) - Run computer vision processing

        ssh drone@192.168.11.{tail_number}
        source /opt/ros/noetic/setup.bash
        source ~/catkin_ws/devel/setup.bash
        (Sanity check! You have to comment "imshow" because Odroid does not have display connected)
        roscd vision_processing/scripts
        nano img_process_node.py
        **Line 63, add '##' to comment the line** ##cv2.imshow("images", combined_frame) 
        (Save and exit)
        python3 img_process_node.py

* When all the terminal are running without any problem, go to T2, do the following   
   * Type 1 and enter: Arm the drone
   * Type 2 and enter: OFFBOARD mode and Takeoff
   * Check if drone is hovering at the home position with desired altitude after Takeoff
   * If the desired altitude is not matched to what you see from QGroundControl, go to T4, adjust the height by adding numbers. This will increase/descrease the altitude in (m)
   * If the altitude looks good, go back to T2, type 5 and enter! The drone will begin the Lissajous pattern mission
   * Make sure to keep the setmode stick to Position Hold and throttle stick in the mid-way on your transmitter.
   * After the mission, use your transmitter to bring back the drone for safe landing.


#### ENJOY!
