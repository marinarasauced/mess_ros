echo "#######################################################################################################################"
echo ""
echo ">>> {Resuming MESS-ROS Configuration}"
echo ""
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 6: Finalizing ROS Configuration}"
echo ""
echo ">>> {Task: Finalizing environment configuration}"
echo ""
echo ">>> {User: Enter desired name for agent}"
echo ""
#
read -p "Enter your wlan0 inet address (it will look like 192.168.0.XX:" ip_address
#
cd
cd mess_ros/ && catkin_make
sudo echo "export ROS_MASTER_URI=http://192.168.0.229:11311" >> ~/.bashrc
sudo echo "export ROS_HOSTNAME=${ip_address}" >> ~/.bashrc
source ~/.bashrc
#
echo ""
echo ">>> {Done: Finalized environment configuration}"
echo ""
echo "#######################################################################################################################"
echo ""
echo ">>> {Ended MESS-ROS Configuration}"
echo ""
echo "#######################################################################################################################"
