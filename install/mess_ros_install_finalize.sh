
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
#
ip_address=$(ifconfig wlan0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
cd /home/$user_name/mess_ros && catkin_make
echo "export ROS_MASTER_URI=http://192.168.0.229:11311" >> /home/$user_name/.bashrc
echo "export ROS_HOSTNAME=${ip_address}" >> /home/$user_name/.bashrc
source /home/$user_name/.bashrc
#
echo ""
echo ">>> {Done: Finalized environment configuration}"
echo ""
echo "#######################################################################################################################"
echo ""
echo ">>> {Ended MESS-ROS Configuration}"
echo ""
echo "#######################################################################################################################"
