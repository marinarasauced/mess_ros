#!/usr/bin/env python

##################################################################################
# SINGLE UGV DEMO EXPERIMENT
#   Goal:  single ugv drives in a triangle formation for one minute
#   Steps:
#      01. clear cache    -> used to store generated vehicle configuration files to 
#                             run experiments before pushing them to vehicles
#      02. define nodes   -> define any additional ros nodes to launch besides 
#                             default nodes such as bringup and mavros
#      03. add vehicles   -> create object for each vehicle client in experiment
#      04. add nodes      -> add nodes to vehicle clients
#      05. write files    -> write launch file and optionally set config params
#      06. start vehicles -> for each vehicle in experiment, launch ros nodes
##################################################################################

import rospy

import json
import os
import subprocess
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.abspath(__file__), "../../..")))

from mess_modules.experiments import * 
from mess_modules.vehicles import *
from mess_modules.nodes import *

def start_experiment():
    pass

def end_experiment(ugvs, uavs):
    # shutdown_ugvs(ugvs)  # shutdown ros nodes, vehicle write logs and then remains online for log transfer
    # shutdown_uavs(uavs)  # shutdown ros nodes, vehicle lands, write logs, and then remains online for log transfer
    download_logs(ugvs + uavs)  # downloads log data from experiment to ~/mess_ros/logs/

##################################################################################

def main():

    # Start VICON bridge:
    #subprocess.Popen("roslaunch vicon_bridge vicon.launch", shell=True)

    # 01.
    clear_cache()
    experiment = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(experiment)

    # 02.
    ugv_control = UGVNode(package="ugv_calibration", file_name="ugv_calibration.py", node_name="calibration")  # vehicle name will be appended as prefix to node name

    # 03.
    burger2 = load_ugv("burger2", experiment)  # first input must be changed to ugv name, second input DO NOT modify
    
    # 04.
    burger2.add_node(ugv_control)

    # 05.
    burger2.generate_launch_description()  # must be after all nodes are added
    burger2.generate_config_description(calibration_samples=1000, max_lin_vel_ratio=0.5, max_ang_vel_ratio=0.3, error_tol_tx=0.02, error_tol_ty=0.02, error_tol_rz=0.01, occlusion_tol=0.1, k_ty=4.6092, k_rz=2.6779)

    # 06.
    ugvs = [burger2]
    uavs = []
    launch_vehicles(ugvs, uavs)


    # launch_2d_occupancy_field(ugvs) -> if num of ugs > 1, start occupancy grid that estimates bounding box collisions in threat plane using current vehicle trajectory (linear case?)
    # run experiment ... -> need to implement likely via publishers and subscibers -> flagging system (at least for ugv)
    start_experiment()  # 
    end_experiment(ugvs, uavs)   # write logs on vehicles, shut down vehicles, download logs to groundstation 
    

    #
    rospy.spin()
    rospy.on_shutdown(end_experiment(ugvs, uavs))  # if experiment needs to be terminated, vehicles will land, write and transfer logs, and shutdown


if __name__=="__main__":
    main()
