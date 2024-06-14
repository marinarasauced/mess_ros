#!/usr/bin/env python

import rospy

import json
import os
import subprocess
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.abspath(__file__), "../../..")))

from mess_modules.experiments import * 
from mess_modules.vehicles import *
from mess_modules.nodes import UGVNode

def load_ugv(name, experiment):
    path2client = os.path.abspath(os.path.join(os.path.dirname(__file__), f"../../mess_modules/clients/{name}.json"))
    f_ = open(path2client)
    f_data = json.load(f_)
    ugv = UGV(
        name=f_data["name"], 
        ip=f_data["ip"], 
        password=f_data["password"], 
        experiment=experiment,
        tb3_model=f_data["model"],
        lds_model=f_data["lds"]
    )
    return ugv

def launch_vehicles(ugvs, uavs):
    for ugv in ugvs:
        upload_cache(ugv)
        launch_ugv(ugv)
    for uav in uavs:
        pass

def upload_cache(vehicle):
    local_path = get_path2experiments()
    remote_path = "~/mess_ros/src/mess_ros/"
    upload(vehicle, local_path, remote_path)



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

    launch_vehicles(ugvs, uavs)


    #
    rospy.spin()


if __name__=="__main__":
    main()
