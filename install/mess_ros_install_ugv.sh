#!/bin/bash -eu
# Author: Marina Nelson

name_ros_distro=noetic 
user_name=$(whoami)

echo "#######################################################################################################################"
echo ""
echo ">>> {Starting MESS-ROS Configuration}"
echo ""

#
echo ""
echo ">>> {User: Enter desired name for agent}"
echo ""
#
read -p "Enter your agent's name i.e., burger2, hawk2, etc.:" agent_name
#
echo ""
echo ">>> {User: Enter desired turtlebot3 model for configuration}"
echo ""
echo "     [1. Burger]"
echo ""
echo "     [2. Waffle]"
echo ""
echo "     [3. WafflePi]"
echo ""
#
read -p "Enter your turtlebot3 model (Default is 1):" answer1 
case "$answer1" in
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
#
echo ""
echo ">>> {User: Enter desired lds model for configuration}"
echo ""
echo "     [1. LDS-01]"
echo ""
echo "     [2. LDS-02]"
echo ""
#
read -p "Enter your lidar model (Default is 2):" answer2
case "$answer2" in
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
#
echo ""
echo ">>> {User: Enter you network SSID and password}"
echo ""
#
read -p "Enter your network SSID:" network_ssid
read -p "Enter your network password:" network_pw
echo ""
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 1: ROS Noetic Installation}"
echo ""
echo ">>> {Task: Setting up ubuntu repositories}"
echo ""
#
sudo add-apt-repository universe
sudo add-apt-repository restricted
sudo add-apt-repository multiverse
sudo apt update
#
echo ""
echo ">>> {Done: Set up ubuntu repositories}"
echo ""
echo ">>> {Task: Setting up sources.list}"
echo ""
#
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#
echo ""
echo ">>> {Done: Set up sources.list}"
echo ""
echo ">>> {Task: Setting up keys}"
echo ""
#
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
#
echo ""
echo ">>> {Done: Set up keys}"
echo ""
echo ">>> {Task: Installing ROS Noetic base}"
echo ""
#
sudo apt update
sudo apt install -y ros-noetic-ros-base
#
echo ""
echo ">>> {Done: Installed ROS Noetic base}"
echo ""
echo ">>> {Task: Setting up environment}"
echo ""
#
echo "source /opt/ros/noetic/setup.bash" >> /home/$user_name/.bashrc
source /home/$user_name/.bashrc
#
echo ""
echo ">>> {Done: Set up environment}"
echo ""
echo ">>> {Task: Installing dependencies}"
echo ""
#
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
#
echo ""
echo ">>> {Done: Installed dependencies}"
echo ""
echo ">>> {Task: Initializing dependencies}"
echo ""
#
sudo rosdep init
rosdep update
#
echo ""
echo ">>> {Done: Initialized dependencies}"
echo ""
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 2: MESS-ROS Workspace Configuration}"
echo ""
echo ">>> {Task: Creating mess_ros workspace}"
echo ">>> {Note: If you get an error, in a terminal, enter \"/source/opt/ros/noetic/setup.bash\", then rerun this bash script}"
echo ""
#
cd 
mkdir mess_ros
cd /home/$user_name/mess_ros
mkdir src
source /opt/ros/noetic/setup.bash && catkin_make
echo "source /home/$user_name/mess_ros/devel/setup.bash" >> /home/$user_name/.bashrc
source /home/$user_name/.bashrc
#
echo ""
echo ">>> {Done: Created mess_ros workspace}"
echo ""
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 3: Installing Extra Packages}"
echo ""
echo ">>> {Task: Adding turtlebot3 packages to mess_ros workspace}"
echo ""
#
sudo apt update
sudo apt install -y libudev-dev
cd /home/$user_name/mess_ros/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b develop https://github.com/ROBOTIS-GIT/ld08_driver.git
sudo apt install -y ros-noetic-rosserial-python
sudo apt install -y ros-noetic-tf
cd /home/$user_name/mess_ros/src/turtlebot3
git pull
rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
#
echo ""
echo ">>> {Done: Added turtlebot3 packages to mess_ros workspace}"
echo ""
echo ">>> {Task: Adding mavros packages to mess_ros workspace}"
echo ""
#
sudo apt-get install -y ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs
cd
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
#
echo ""
echo ">>> {Done: Added mavros packages to mess_ros workspace}"
echo ""
echo ">>> {Task: Adding ros_msgs_parser package to build}"
echo ""
#
cd
git clone https://github.com/Vik095/Ros_msgs_parser
mv /home/$user_name/Ros_msgs_parser /home/$user_name/ros_msgs_parser
#
echo ""
echo ">>> {Done: Added ros_msg_parser package to build}"
echo ""
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 4: Installing MESS-ROS Packages}"
echo ""
echo ">>> {Task: Added mess_ros packages to mess_ros workspace}"
echo ""
#
cd /home/$user_name/mess_ros/src
git clone https://github.com/marinarasauced/mess_ros.git
cd /home/$user_name/mess_ros/src/mess_ros
#
echo ""
echo ">>> {Done: Added mess_ros packages to mess_ros workspace}"
echo ""
echo ">>> {Task: Updating turtlebot3_bringup files}"
echo ""
#
sudo apt-get install -y libjsoncpp-dev
cd /home/$user_name/mess_ros/src/turtlebot3/turtlebot3_bringup
rm -r CMakeLists.txt src/turtlebot3_diagnostics.cpp
cp /home/$user_name/mess_ros/src/mess_ros/messop_ugv/move2turtlebot3_bringup/CMakeLists.txt /home/$user_name/mess_ros/src/turtlebot3/turtlebot3_bringup
cp /home/$user_name/mess_ros/src/mess_ros/messop_ugv/move2turtlebot3_bringup/turtlebot3_diagnostics.cpp /home/$user_name/mess_ros/src/turtlebot3/turtlebot3_bringup/src
#
echo ""
echo ">>> {Done: Updated turtlebot3_bringup files}"
echo ""
echo ">>> {Task: Writing agent.json to ~/mess_ros/src/mess_ros/}"
echo ""
#
echo "{\"name\": \"$agent_name\"}" > /home/$user_name/mess_ros/src/mess_ros/agent.json
#
echo ""
echo ">>> {Done: Wrote agent.json to ~/mess_ros/src/mess_ros/}"
echo ""
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 5: Setting up OpenCR}"
echo ""
echo ">>> {Task: Configuring turtlebot3 OpenCR for selected model}"
echo ""
#
sudo dpkg --add-architecture armhf
sudo apt-get update
sudo apt-get install -y libc6:armhf
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=${firmware_model}_noetic
cd
rm -rf ./opencr_update.tar.bz2
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2 
tar -xvf opencr_update.tar.bz2 
cd ./opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
#
echo ""
echo ">>> {Task: Updated OpenCR firmware for selected model}"
echo ""
echo ">>> {Task: Configuring environment for selected turtlebot3 and lidar models}"
echo ""
#
echo "export TURTLEBOT3_MODEL=${turtlebot3_model}" >> /home/$user_name/.bashrc
echo "export LDS_MODEL=${lidar_model}" >> /home/$user_name/.bashrc
#
echo ""
echo ">>> {Done: Configured environment}"
echo ""
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 6: Configuring Netplan}"
echo ""
echo ">>> {Task: Writing netplan file}"
echo ""
#
sudo apt install -y net-tools
sudo bash -c 'echo "network:" >> /etc/netplan/01-network-manager-all.yaml'
sudo bash -c 'echo "  version: 2" >> /etc/netplan/01-network-manager-all.yaml'
sudo bash -c 'echo "  renderer: networkd" >> /etc/netplan/01-network-manager-all.yaml'
sudo bash -c 'echo "  wifis:" >> /etc/netplan/01-network-manager-all.yaml'
sudo bash -c 'echo "    wlan0:" >> /etc/netplan/01-network-manager-all.yaml'
sudo bash -c 'echo "      dhcp4: true" >> /etc/netplan/01-network-manager-all.yaml'
sudo bash -c 'echo "      access-points:" >> /etc/netplan/01-network-manager-all.yaml'
sudo bash -c "echo '        "${network_ssid}":' >> /etc/netplan/01-network-manager-all.yaml"
sudo bash -c "echo '          password: "${network_pw}"' >> /etc/netplan/01-network-manager-all.yaml"
sudo netplan apply
#
echo ""
echo ">>> {Done: Wrote and applied netplan file}"
echo ""
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 6: Finalizing ROS Configuration}"
echo ""
echo ">>> {Task: Downloading mess_ros_install_finalize.sh}"
echo ""
#
cd
wget -c https://raw.githubusercontent.com/marinarasauced/mess_ros/ros1-noetic/install/mess_ros_install_finalize.sh
chmod +x mess_ros_install_finalize.sh
#
echo ""
echo ">>> {Done: Downloaded mess_ros_install_finalize.sh}"
echo ""
echo ">>> {User: Reboot needed to finalize network configuration}"
echo ">>> {User: After the reboot, in a terminal, enter \"ifconifg\"}"
echo ">>> {User: Then, in a terminal, enter \"./mess_ros_install_finalize.sh\" and enter the wlan0 inet address}"
echo ""
echo "#######################################################################################################################"
