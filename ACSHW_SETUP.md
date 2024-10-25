
# ACS Hardware Setup




## Requirements

This setup requires one PC and one ODroid with a SD card (minimum 32GB).

* PC: 
    * Ubuntu 20.04 LTS
    * Install ROS Noetic
    * Follow ACS simulation setup
    * One Alfa wireless USB card
* Odroid: 
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

    
* Install/configure docker 

        sudo apt install docker.io
        cd /etc/docker
        sudo nano daemon.json
    Enter the following in daemon.json, then save
        
        {
            "insecure-registries" : ["<hostname of ground station>.local:5000"]
        }
    Once finished, restart the Odroid and log back in

* Create user

        useradd -m drone

* Change Password
        
        passwd drone
        swarm
* Add yourself to sudo and docker group

        usermod -aG sudo,docker drone
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


## PC preparation

### Alfa wireless USB connection setup
* Plug in an Alfa wireless USB card to your PC (Groundstation)
* Open a terminal(1), Run
        
        ip a
* Copy the id of the last wireless device. It should be similar to wlx00c0ca904414.
* Navigate to $HOME/acs/
* Run the wifi_config.sh script, entering in the id from step 1 and the last octet of the desired IP for your GCS (must be a number between 206 and 254):
       
        ./wifi_config.sh <wireless device id> <Choose a # between 206-254>
        example) wifi_config.sh wlx00c0ca904414 206

### Docker Setup Build and Update

#### Initial docker setup (PC Side - Terminal 1)
* If you've never installed nor setup docker, source the /acs/setup/setup_docker.sh script

        cd acs/setup
        . setup_docker.sh
#### Build Docker image (PC Side - Terminal 1)
* To create a docker image, run the build_docker.sh script

        cd acs
        ./build_docker.sh
* Anytime time code changes are made, you will need to rerun the build_docker.sh script to reflect those changes.
* Note The build script builds the image based on your current source code (including uncommitted changes). If you change branches and rebuild, the new image will reflect the new code base.

#### Drone Initial Setup (PC Side - Terminal 1)
* To load a docker image onto a drone for the first time, turn on the target drone
* On your PC, check if your PC and Odroid are connected to the local network by ./wifi_config.sh script
* Open terminal
        
        ping 192.168.11.<drone tail num>
* If ping is working, setup an ssh key using the below commands
* NOTE The first command generates a key and only needs to be run once per computer, not once per drone

        ssh-keygen -t rsa
        ssh-copy-id -i $HOME/.ssh/id_rsa.pub drone@192.168.11.<drone tail num>

* Open another terminal(2), ssh connection into the Odroid 
    
        ssh drone@192.168.11.<drone tail num>

#### Sanaty Check before updating the drone (PC Side - Terminal 2)
* Run following command

        ping <hostname of ground station>.local
* If ping is active, you are ready for the update

#### Updating a Drone (PC Side - Terminal 1)
* To update a drone, run the update drone.sh script with the target drone id

        cd acs
        ./update_drone <drone tail num>
* NOTE The update script assumes the first three octets of your IP are 192.168.11. If that is not true, you can simply edit or run the commands in the script with the correct IP address.        

#### Drone update check up (PC Side - Terminal 2)
* To check if the update went well, run the following command

        docker ps
    If docker is active, copy the container name, run the following command
  
        docker logs -f <container_name>
* If you see the successful ROS Launch logs, you are good to go.

#### Run SwarmCommander (PC Side - Terminal 1)

* Launch the SwarmCommander

        sc
* In the Swarm Commander GUI, use the network drop down to select your network device (ex. wlx00c0ca904414)


## Important docker command to know

* Purging All Unused or Dangling Images, Containers, Volumes, and Networks

      docker system prune -a
* Removing Docker Images

      docker images -a
      docker rmi $(docker images -a -q)
* Removing Docker Containers

      docker stop $(docker ps -a -q)
      docker rm $(docker ps -a -q)
* When Docker doesn't start after reboot/boot
Inspection

      systemctl status docker.socket
      systemctl status docker.service
Find if docker service is disabled, if yes run the following command

      sudo systemctl enable --now docker.service

##Instructions to rebuild docker container and push updates to drones.
      
Open Terminal 1
-Make sure the drone is connected on the same network through Alfa wireless 

-ping 192.168.11.(drone's tail number)

- If postive, run the following command for each drone to register drone's ip address to SSH Key.
You only need to do this once. This line will work when drone is active on the network
This line also will help you SSH into your drone without typing a password

as of 01/25/2024, drone tail number 148 and 149 are registered

ssh-copy-id -i $HOME/.ssh/id_rsa.pub drone@192.168.11.(drone's tail number)

Finally get into the drone's odroid via SSH

-ssh drone@192.168.11.(drone's tailnumber)

-password: swarm

-cd /etc/docker

-sudo nano daemon.json

- Check if the contents inside of "daemon.json" file is following with the hostname "hivemines"
{
        "insecure-registries" : ["hivemines.local:5000"]
}

- If everything looks good, move to the next step


Open Terminal 2

Go to ACS directory
- cd /acs

Build the docker before updating the drone
- ./build_docker.sh 

If no error, proceed update drone
- ./update_drone.sh (drone tail number)

