# messop_ugv
ROS Noetic nodes for TurtleBot3s. Allows for time-optimal waypoint navigation using primarilly VICON localization with onboard Odometry and SLAM during occlusion periods or deliberate interference.

## Installation requirements
- A TurtleBot3 device - *if you are working on a new device, following [these steps](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) to install ROS Noetic*
- An Ubuntu 20.04 machine
- A network with internet

## Installation instructions
To build from source, go to your catkin_ws ```cd ~/catkin_ws/src```.

Download the source for these nodes by running

```git clone https://github.com/marinarasauced/messop_ugv.git```

Replace the CMakeLists.txt and turtlebot3_diagnostics.cpp in the turtlebot3_bringup package with the modified versions from the move2turtlebot3_bringup directory in the messop_ugv package.

```cp ~/catkin_ws/src/messop_ugv/move2turtlebot3_bringup/CMakeLists.txt ~/catkin_ws/src/turtlebot3/turtlebot3_bringup/CMakeLists.txt```

```cp ~/catkin_ws/src/messop_ugv/move2turtlebot3_bringup/turtlebot3_diagnostics.cpp ~/catkin_ws/src/turtlebot3/turtlebot3_bringup/src/turtlebot3_diagnostics.cpp```

The modifications to the turtlebot3_diagnostics.cpp require libjsoncpp-dev to be installed.

```sudo apt install libjsoncpp-dev```

Compilete the code with ```catkin_make```.

## Usage instructions
Once you have built the package, you can launch the nodes. These nodes utilize localization in a VICON environment and are designed to work with ROBOTIS's [ROBOTIS TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3) and [VICON Bridge](https://github.com/ethz-asl/vicon_bridge) packages. Ensure that the vicon_bridge node is running on your remote Ubuntu machine.

Onboard the TurtleBot3, run the bringup node.

```roslaunch messop_ugv messbringup_ugv.launch```

This launch file will initiate the turtlebot3_bringup.launch, the messop node, and the messlogger node. Data is saved to /home/ubuntu/catkin_ws/logs/csv/ by default when the node is shut down.

To control the TurtleBot3, publish a MESS2UGV message from the [mess_msgs package](https://github.com/marinarasauced/mess_msgs).

## Messop node information
Topics:
- ```/{UGV_NAME}/cmd_vel```: Publishes ```geometry_msgs/Twist``` control input to bringup node.
- ```/{UGV_NAME}/messop/logger/coefficients```: Publishes ```mess_msgs/CalibrateUGV``` coefficients that calibrate Odometry and SLAM to VICON environment.
- ```/{UGV_NAME}/messop/logger/global```: Publishes ```mess_msgs/StateUGV``` estimated from VICON, Odometry, and SLAM.
- ```/{UGV_NAME}/messop/logger/odom```: Publishes ```mess_msgs/StateUGV``` from Odometry and SLAM calibrated to the VICON environment.
- ```/{UGV_NAME}/messop/messop/vertex```: Subscribes to ```mess_msgs/MESS2UGV``` vertex and operation type.
- ```/{UGV_NAME}/messop/messop/interference```: Subscribes to ```std_msgs/Bool``` for deliberately ignoring VICON localization.
- ```/vicon/{UGV_NAME}/{UGV_NAME}```: Subscribes to ```geometry_msgs/TransformStamped```VICON localization.

## Logger node information
Topics:
- ```/{UGV_NAME}/cmd_vel```: Subscribes to ```geometry_msgs/Twist``` control input from messop node.
- ```/{UGV_NAME}/messop/logger/coefficients```: Subscribes to ```mess_msgs/CalibrateUGV``` coefficients that calibrate Odometry and SLAM to VICON environment.
- ```/{UGV_NAME}/messop/logger/global```: Subscribes to ```mess_msgs/StateUGV``` estimated from VICON, Odometry, and SLAM.
- ```/{UGV_NAME}/messop/logger/odom```: Subscribes to ```mess_msgs/StateUGV``` from Odometry and SLAM calibrated to the VICON environment.
- ```/vicon/{UGV_NAME}/{UGV_NAME}```: Subscribes to ```geometry_msgs/TransformStamped``` VICON localization.
