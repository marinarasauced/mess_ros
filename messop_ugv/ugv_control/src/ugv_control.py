#!/usr/bin/env python

import rospy

import os.path
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.abspath(__file__), "../../..")))

from ugv_modules.ugv_class import UGV
from ugv_modules.ugv_functions import get_max_velocities, load_config_from_node, load_calibration_from_node

##################################################################################

def main():

    # Initialize node:
    rospy.init_node("ugv_calibration")
    rospy.sleep(12)  # wait for turtlebot3_bringup

    # Retrieve UGV parameters:
    ugv_max_lin_vel, ugv_max_ang_vel = get_max_velocities(rospy.get_param("model", "burger"))
    f_data = load_config_from_node()
    c_data = load_calibration_from_node()

    # Create UGV object:
    ugv = UGV(
        f_data["ugv_name"],
        ugv_max_lin_vel,
        f_data["max_lin_vel_ratio"],
        ugv_max_ang_vel,
        f_data["max_ang_vel_ratio"],
        f_data["error_tol_tx"],
        f_data["error_tol_ty"],
        f_data["error_tol_rz"],
        f_data["occlusion_tol"],
        f_data["k_ty"],
        f_data["k_rz"],
        c_data["x"], 
        c_data["y"], 
        c_data["z"], 
        c_data["w"]
    )

    # Run MESSOP:
    ugv.transitionUGV()

if __name__=="__main__":
    main()
