#!/usr/bin/env python

import rospy

import json
import os
import subprocess
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.abspath(__file__), "../../..")))

from mess_modules.experiments import * 
from mess_modules.vehicles import *
from mess_modules.nodes import *





##################################################################################

def main():

    # Start VICON bridge:
    #subprocess.Popen("roslaunch vicon_bridge vicon.launch", shell=True)

    # Initialize node:
    clear_cache()
    experiment = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(experiment)

    # Define nodes (node types like bringup, mavros, loggers, etc. do not need to be specified):
    ugv_control = UGVNode("ugv_calibration", "ugv_calibration.py", "calibration")

    # Add vehicles:
    burger2 = load_ugv("burger2", experiment)
    burger2.add_node(ugv_control)
    burger2.generate_launch_description()  # must be after all nodes are added
    burger2.generate_config_description(calibration_samples=1000, max_lin_vel_ratio=0.5, max_ang_vel_ratio=0.3, error_tol_tx=0.02, error_tol_ty=0.02, error_tol_rz=0.01, occlusion_tol=0.1, k_ty=4.6092, k_rz=2.6779)

    # Create list of vehicles:
    ugvs = [burger2]
    uavs = []
    vehicles = [ugvs, uavs]

    launch_vehicles(ugvs, uavs)
    # run experiment ... -> need to implement likely via publishers and subscibers -> flagging system (at least for ugv)
    download_logs(vehicles)

    #
    rospy.spin()


if __name__=="__main__":
    main()
