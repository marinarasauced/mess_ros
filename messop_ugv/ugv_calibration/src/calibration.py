#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped, Quaternion

import json
import numpy as np
import os.path
import time
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.abspath(__file__), "../../..")))

from ugv_modules.quaternions import convert_eul2quat, invert_quat, multiply_quats
from ugv_modules.ugv_class import UGV
from ugv_modules.ugv_functions import get_max_velocities, load_config_from_node

##################################################################################

# Collect measurements from VICON motion capture system.
#   Input:  ugv class object
def sample_data(ugv, num_samples):
    ts_sample = TransformStamped()
    for iter in range(num_samples):
        ts_meas = rospy.wait_for_message(ugv.topic_vicon, TransformStamped)
        ts_sample = update_sample(ts_sample, ts_meas, iter)

    return ts_sample

# Recursively average new measurements with previous mean.
#   Input:  previous mean ROS1 TransformStamped message, newly measured ROS1 TransformStamped message, average weight
#   Output: new mean ROS1 TransformStamped message
def update_sample(ts_in1, ts_in2, iter):
    ts_Tx1 = ts_in1.transform.translation.x
    ts_Ty1 = ts_in1.transform.translation.y
    ts_Tz1 = ts_in1.transform.translation.z
    ts_Qx1 = ts_in1.transform.rotation.x
    ts_Qy1 = ts_in1.transform.rotation.y
    ts_Qz1 = ts_in1.transform.rotation.z
    ts_Qw1 = ts_in1.transform.rotation.w
    ts_Tx2 = ts_in2.transform.translation.x
    ts_Ty2 = ts_in2.transform.translation.y
    ts_Tz2 = ts_in2.transform.translation.z
    ts_Qx2 = ts_in2.transform.rotation.x
    ts_Qy2 = ts_in2.transform.rotation.y
    ts_Qz2 = ts_in2.transform.rotation.z
    ts_Qw2 = ts_in2.transform.rotation.w

    ts_out1 = TransformStamped()
    ts_out1.transform.translation.x = (ts_Tx1 * iter + ts_Tx2) / (iter + 1)
    ts_out1.transform.translation.y = (ts_Ty1 * iter + ts_Ty2) / (iter + 1)
    ts_out1.transform.translation.z = (ts_Tz1 * iter + ts_Tz2) / (iter + 1)
    ts_out1.transform.rotation.x = (ts_Qx1 * iter + ts_Qx2) / (iter + 1)
    ts_out1.transform.rotation.y = (ts_Qy1 * iter + ts_Qy2) / (iter + 1)
    ts_out1.transform.rotation.z = (ts_Qz1 * iter + ts_Qz2) / (iter + 1)
    ts_out1.transform.rotation.w = (ts_Qw1 * iter + ts_Qw2) / (iter + 1)
    
    return ts_out1

# Writes transformation quaternion to calibration.json for vicon callbacks.
#   Input:  ugv object and number of vicon samples to collect at each position
#   Output: calibration.json written to config directory
def run_calibration(ugv, num_samples):

    # Collect calibration samples at first position:
    ts_pos1 = sample_data(ugv, num_samples)

    # Translate forward linearly for one second:
    u_lin = ugv.max_lin_vel
    tic = time.time()
    while time.time() < tic + 1:
        ugv.controlUGV(u_lin, 0.0)
    ugv.controlUGV(0.0, 0.0)

    # Collect calibration samples at second position:
    ts_pos2 = sample_data(ugv)

    # Apply flat level lab assumptions:
    heading = np.arctan2(
        ts_pos2.transform.translation.y - ts_pos1.transform.translation.y,
        ts_pos2.transform.translation.x - ts_pos1.transform.translation.x
    )
    q0_ = convert_eul2quat(0.0, 0.0, heading)
 
    # Calculate transformation quaternion:
    q1_ = Quaternion()
    q1_.x = ts_pos1.transform.rotation.x
    q1_.y = ts_pos1.transform.rotation.y
    q1_.z = ts_pos1.transform.rotation.z
    q1_.w = ts_pos1.transform.rotation.w

    q1_inv = invert_quat(q1_)
    q_diff = multiply_quats(q1_inv, q0_)

    # Write calibration file:
    data = {"x": q_diff.x, "y": q_diff.y, "z": q_diff.z, "w": q_diff.w}
    with open(os.path.abspath(os.path.dirname(__file__) + "/../../config/") + "calibration.json", "w") as f:
        json.dump(data, f)

##################################################################################

def main():

    # Initialize node:
    rospy.init_node("ugv_calibration")
    rospy.sleep(12)  # wait for turtlebot3_bringup

    # Retrieve UGV parameters:
    ugv_max_lin_vel, ugv_max_ang_vel = get_max_velocities(rospy.get_param("model", "burger"))
    f_data = load_config_from_node()

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
        0, 0, 0, 1
    )

    # Run calibration:
    run_calibration(ugv, f_data["calibration_samples"])

if __name__=="__main__":
    main()
