#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped

import json
import numpy as np
import os.path
import sys
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))

from mess_modules.paths import get_path2agent
from mess_modules.quaternions import convert_eul2quat, invert_quat, multiply_quats, average_quats
from mess_modules.ugv_secondary import UGVSecondary


def update_sample(ts_1, ts_2, idx):
    """
    Recursively update a sample estimate.
    
    Parameters
    ----------
    ts_1 : TransformStamped ROS message instance
        The current TransformStamped average.
    ts_2 : TransformStamped ROS message instance
        The new TransformStamped measurement.
    idx : int
        The index of the current measurement.

    Returns
    -------
    ts_3 : TransformStamped ROS message instance
        The updated TransformStamped average.
    """

    ts_tx1 = ts_1.transform.translation.x
    ts_ty1 = ts_1.transform.translation.y
    ts_tz1 = ts_1.transform.translation.z
    ts_qx1 = ts_1.transform.rotation.x
    ts_qy1 = ts_1.transform.rotation.y
    ts_qz1 = ts_1.transform.rotation.z
    ts_qw1 = ts_1.transform.rotation.w
    ts_tx2 = ts_2.transform.translation.x
    ts_ty2 = ts_2.transform.translation.y
    ts_tz2 = ts_2.transform.translation.z
    ts_qx2 = ts_2.transform.rotation.x
    ts_qy2 = ts_2.transform.rotation.y
    ts_qz2 = ts_2.transform.rotation.z
    ts_qw2 = ts_2.transform.rotation.w

    ts_3 = TransformStamped()
    ts_3.transform.translation.x = (ts_tx1 * iter + ts_tx2) / (iter + 1)
    ts_3.transform.translation.y = (ts_ty1 * iter + ts_ty2) / (iter + 1)
    ts_3.transform.translation.z = (ts_tz1 * iter + ts_tz2) / (iter + 1)
    ts_3.transform.rotation.x = (ts_qx1 * iter + ts_qx2) / (iter + 1)
    ts_3.transform.rotation.y = (ts_qy1 * iter + ts_qy2) / (iter + 1)
    ts_3.transform.rotation.z = (ts_qz1 * iter + ts_qz2) / (iter + 1)
    ts_3.transform.rotation.w = (ts_qw1 * iter + ts_qw2) / (iter + 1)
    return ts_3


def sample_data(ugv):
    """
    Collect sample data at a point and call a recursive update function.
    
    Parameters
    ----------
    ugv : UGVSecondary instance
        The UGVSecondary instance of the current agent.

    Returns
    -------
    ts_sample : TransformStamped ROS message instance
        The average VICON localization data after N samples. N is defined in agents/{agent.name}/config.json
    """
    
    ts_sample = TransformStamped()
    for idx in range(ugv.calibration_samples):
        ts_meas = rospy.wait_for_message(ugv.topic_vicon, TransformStamped)
        ts_sample = update_sample(ts_sample, ts_meas, idx)

def run_calibration(ugv):
    """
    Run the calibration and write the calibration file.

    Parameters
    ----------
    ugv : UGVSecondary instance
        The UGVSecondary instance of the current agent.
    """

    ts_pos1 = sample_data(ugv)
    tic = time.time()
    toc = tic + 1
    while time.time() < toc:
        ugv.controlUGV(u_lin=ugv.max_lin_vel, u_ang=0.0)
    ugv.controlUGV(u_lin=0.0, u_ang=0.0)
    ts_pos2 = sample_data(ugv)

    theta = np.arctan2(
        ts_pos2.transform.translation.y - ts_pos1.transform.translation.y,
        ts_pos2.transform.translation.x - ts_pos1.transform.translation.x
    )
    q_0 = convert_eul2quat(0.0, 0.0, theta)
    q_1 = average_quats(ts_pos1, ts_pos1)
    q_1_inv = invert_quat(q_1)
    q_diff = multiply_quats(q_1_inv, q_0)

    file_path = os.path.join(get_path2agent(ugv.name), f"calibration.json")
    content = {"x": q_diff.x, "y": q_diff.y, "z": q_diff.z, "w": q_diff.w}
    with open(file_path, "w") as file:
        json.dump(content)

def main():
    """
    Run the calibration of the agent's UGVSecondary instance.

    This node will start the ugv agent's dynamics. It is currently configured for waypoint navigation so that the agent can translate linearly between two points to collect data for the calibration.
    """

    rospy.init_node("ugv_calibration")
    rospy.sleep(12)  # wait for turtlebot3_bringup (ikr such a bad way to wait)

    ugv = UGVSecondary(rospy.get_param("ugv_name"))
    run_calibration(ugv)

if __name__=="__main__":
    main()
