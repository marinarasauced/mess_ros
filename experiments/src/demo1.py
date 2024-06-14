#!/usr/bin/env python

import rospy

import os.path
import subprocess
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.abspath(__file__), "../../..")))


from mess_modules.vehicles import UGV
from mess_modules.nodes import Node

##################################################################################

def main():

    # Initialize node:
    rospy.init_node("demo1")

    # Clear cache:
    subprocess.run("cd ~/mess_ros/src/mess_ros/experiments/src/ && rm -r cache/ && mkdir cache/")

    # Define nodes (node types like bringup, mavros, loggers, etc. do not need to be specified):
    ugv_control = Node("ugv_control", "ugv_control.py", "control")


    # Add vehicles:
    burger2 = UGV("burger2", "192.168.0.58", "cowlagilab01")
    burger2.add_node(ugv_control)
    burger2.generate_launch_description()



    # Generate experiment launch file:


if __name__=="__main__":
    main()
