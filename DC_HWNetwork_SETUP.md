
# Drone Onboard Computer (DC) Hardware Setup


## Requirements

This setup requires one GSC and one ODroid with a SD card (minimum 32GB).

* PC or Laptop (GSC)
    * Ubuntu 20.04
    * One Alfa wireless USB card
* Odroid (DC): 
    * SD card should have Ubuntu 20.04 for Odroid XU4
    * One Alfa wireless USB card

## Odroid Preparation

* Plug keyboard/mouse/monitor/ethernet cable into the Odroid. Boot up
* Log in using credentials
    * User: root
    * Password: odroid
* Install necessary packages

        apt-get update
        sudo apt-get install avahi-daemon
 
* Create user

        useradd -m drone

* Change Password
        
        passwd drone
        swarm

* Start your user in bash shell

        sudo chsh -s /bin/bash drone
* Update netplan

        cd /etc/netplan/
        sudo nano 00-all.yaml
    Enter the following in 00-all.yaml file, then save
    
        network:
           version: 2
           renderer: NetworkManager
           wifis:
              all-wlans:
                 match: {}
                 access-points:
                      "drone":
                          mode: adhoc
                          band: 2.4GHz
                          channel: 1
                          bssid: 00:99:88:77:66:55
                 addresses:
                     - 192.168.11.<num>/24
    Here, <num> is your desired drone's tail number; i.e. 192.168.11.135 (tail number 135)
* Plug in an Alfa wireless USB card to the Odroid, then reboot and log back in
* Check if ip address is set to the desired you put in 00-all.yaml
        
        ip a
* If the check is positive, you are ready to go. you do not need keyboard/mouse/monitor/ethernet cable at this point


## Ground Station Computer (GSC) preparation

### Alfa wireless USB connection setup
* Plug in an Alfa wireless USB card to your PC (Groundstation)
* Open a terminal(1), Run
        
        ip a
* Copy the id of the last wireless device. It should be similar to wlx00c0ca904414.
* Navigate to $HOME/catkin_ws/
* Run the wifi_config.sh script, entering in the id from step 1 and the last octet of the desired IP for your GCS (must be a number between 206 and 254):
       
        ./wifi_config.sh <wireless device id> <Choose a # between 206-254>
        example) wifi_config.sh wlx00c0ca904414 206


#### DC Initial Setup (GSC Side - Terminal 1)
* To ssh onto a drone for the first time, turn on the target drone
* On your GSC, check if your GSC and DC are connected to the local network by ./wifi_config.sh script
* Open terminal
        
        ping 192.168.11.<drone tail num>
* If ping is working, setup an ssh key using the below commands
* NOTE The first command generates a key and only needs to be run once per computer, not once per drone

        ssh-keygen -t rsa
        ssh-copy-id -i $HOME/.ssh/id_rsa.pub drone@192.168.11.<drone tail num>

* Open another terminal(2), ssh connection into the Odroid 
    
        ssh drone@192.168.11.<drone tail num>



