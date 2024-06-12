#!/bin/bash -eu
# Author: Marina Nelson

name_ros_distro=noetic 
user_name=$(whoami)

#Install ROS Noetic using QBotics Labs ros_install_noetic.sh
cd && wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && echo "3" | ./ros_install_noetic.sh
source ~/.bashrc

echo "#######################################################################################################################"
echo ""
echo ">>> {Starting MESS-ROS Configuration}"
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 1: Create MESS-ROS Workspace}"
echo ""

#Create ROS workspace for MESS supporting packages.
echo ">>> {Making mess_ros workspace}"
echo ""

cd && mkdir mess_ros && cd mess_ros && mkdir src && catkin_make
echo "source ~/mess_ros/devel/setup.bash" >> /home/$user_name/.bashrc
source ~/.bashrc

echo ">>> {Done: Created mess_ros Workspace}"
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 2: Installing TurtleBot3 Packages}"
echo ""

sudo apt update
sudo apt install libudev-dev
cd ~/mess_ros/src && git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/mess_ros/src && git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
cd ~/mess_ros/src && git clone -b develop https://github.com/ROBOTIS-GIT/ld08_driver.git
cd ~/mess_ros/src/turtlebot3 && git pull
cd ~/mess_ros/src/turtlebot3 && rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/

echo ""
echo ">>> {Done: Added turtlebot3 packages}"
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 3: Installing MESS-ROS Packages}"
echo ""

#Clone mess_ros packages from github repo and remove unnecessary ones.
cd ~/mess_ros/src && git clone https://github.com/marinarasauced/mess_ros.git && cd mess_ros
cd ~/mess_ros/src/mess_ros && rm -r install/ messop_uav/

echo ""
echo ">>> {Done: Added mess_ros packages}"
echo ""
echo ">>> {Cloning modified files to turtlebot3_bringup}"
echo ""

#Replace turtlebot3_bringup files.
sudo apt-get install libjsoncpp-dev
cd ~/mess_ros/src/turtlebot3 && rm -r turtlebot3_bringup/CMakeLists.txt turtlebot3_bringup/src/turtlebot3_diagnostics.cpp
cp ~/mess_ros/src/mess_ros/messop_ugv/config/move2turtlebot3_bringup/CMakeLists.txt ~/mess_ros/src/turtlebot3/turtlebot3_bringup
cp ~/mess_ros/src/mess_ros/messop_ugv/config/move2turtlebot3_bringup/turtlebot3_diagnostics.cpp ~/mess_ros/src/turtlebot3/turtlebot3_bringup/src

echo ""
echo ">>> {Done: Replaced CMakeLists.txt and turtlebot3_diagnostics.cpp}"
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 4: Setting up OpenCR, you pick which turtlebot3 model to configure for}"
echo ""
echo "     [1. Burger]"
echo ""
echo "     [2. Waffle]"
echo ""
echo "     [3. WafflePi]"
echo ""

#Assigning default value as 1: Burger
read -p "Enter your turtlebot3 model (Default is 1):" answer 

case "$answer" in
  1)
    firmware_model="burger"
    turtlebot3_model="burger"
    ;;
  2)
    firmware_model="waffle"
    turtlebot3_model="waffle"
    ;;
  3)
    firmware_model="waffle"
    turtlebot3_model="waffle_pi"
    ;;
  * )
    firmware_model="burger"
    turtlebot3_model="burger"
    ;;
esac

sudo dpkg --add-architecture armhf
sudo apt-get update
sudo apt-get install libc6:armhf

export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=${firmware_model}_noetic
cd && rm -rf ./opencr_update.tar.bz2

cd && wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2 
tar -xvf opencr_update.tar.bz2 

cd ./opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr

echo ""
echo ">>> {Done: Updated OpenCR Firmware, you pick which lidar model to configure for}"
echo ""
echo "     [1. LDS-01]"
echo ""
echo "     [2. LDS-02]"
echo ""

#Assigning default value as 1: LDS-02
read -p "Enter your lidar model (Default is 2):" answer 

case "$answer" in
  1)
    lidar_model="LDS-01"
    ;;
  2)
    lidar_model="LDS-02"
    ;;
  * )
    lidar_model="LDS-02"
    ;;
esac

echo "export TURTLEBOT3_MODEL=${turtlebot3_model}" >> /home/$user_name/.bashrc
echo "export LDS_MODEL=${lidar_model}" >> /home/$user_name/.bashrc

echo ""
echo ">>> {Done: Configured turtlebot3 and lidar models}"
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 5: Configuring Netplan, you enter the network SSID and password}"
echo ""

read -p "Enter your network SSID:" network_ssid
read -p "Enter your network password:" network_pw

echo ""
echo ">>> {Writing netplan file}"
echo ""

echo "network:" >> /etc/netplan/01-network-manager-all.yaml
ehco "  version: 2" >> /etc/netplan/01-network-manager-all.yaml
echo "  renderer: networkd" >> /etc/netplan/01-network-manager-all.yaml
echo "  wifis:" >> /etc/netplan/01-network-manager-all.yaml
ehco "    wlan0:" >> /etc/netplan/01-network-manager-all.yaml
echo "      dhcp4: true" >> /etc/netplan/01-network-manager-all.yaml
echo "      access-points:" >> /etc/netplan/01-network-manager-all.yaml
echo "        ${network_ssid}:" >> /etc/netplan/01-network-manager-all.yaml
echo "          password: ${network_pw}" >> /etc/netplan/01-network-manager-all.yaml
netplan apply

echo ""
echo ">>> {Done: Netplan written and applied}"
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 6: Finalizing ROS Configuration}"
echo ""

ip_address=$(ifconfig wlan0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
cd mess_ros && catkin_make
echo "export ROS_MASTER_URI=http://192.168.0.229:11311" >> /home/$user_name/.bashrc
echo "export ROS_HOSTNAME=${ip_address}" >> /home/$user_name/.bashrc

echo ""
echo ">>> {Done: MESS-ROS configured}"
echo ""
echo "#######################################################################################################################"
