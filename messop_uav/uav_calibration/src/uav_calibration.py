#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped, Quaternion
from std_msgs.msg import Bool

import json
import numpy as np
import os.path
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.abspath(__file__), "../../..")))

from uav_modules.quaternions import convert_eul2quat, invert_quat, multiply_quats
from uav_modules.uav_functions import load_config_from_node

##################################################################################

# Collect measurements from VICON motion capture system.
#   Input:  number of samples to collect
def sample_data(num_samples, topic_vicon):
    ts_sample = TransformStamped()
    for iter in range(num_samples):
        ts_meas = rospy.wait_for_message(topic_vicon, TransformStamped)
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
#   Input:  uav object and number of vicon samples to collect at each position
#   Output: calibration.json written to config directory
def run_calibration(num_samples, uav_name):

    #
    topic_vicon = "/vicon/" + uav_name + "/" + uav_name

    # Collect calibration samples at first position:
    print(f"When ready to sample at first point, run: rostopic pub /{uav_name}/messop/calibration1 std_msgs/Bool 'data=true'")
    rospy.wait_for_message("/" + uav_name + "/messop/calibration1", Bool)
    ts_pos1 = sample_data(num_samples, topic_vicon)


    # Collect calibration samples at second position:
    print(f"When ready to sample at second point, run: rostopic pub /{uav_name}/messop/calibration2 std_msgs/Bool 'data=true'")
    rospy.wait_for_message("/" + uav_name + "/messop/calibration2", Bool)
    ts_pos2 = sample_data(num_samples, topic_vicon)

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
    print("Calibration file written, shutting down")

##################################################################################

def main():

    # Initialize node:
    rospy.init_node("uav_calibration")

    # Retrieve UAV parameters:
    f_data = load_config_from_node()

    # Run calibration:
    run_calibration(f_data["calibration_samples"], f_data["uav_name"])

if __name__=="__main__":
    main()
