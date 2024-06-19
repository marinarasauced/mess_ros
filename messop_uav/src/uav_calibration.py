#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool

import json
import numpy as np
import os.path
import sys
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))

from mess_modules.agents import get_agent_name
from mess_modules.paths import get_path2agent
from mess_modules.quaternions import convert_eul2quat, invert_quat, multiply_quats, average_quats
from mess_modules.uav_secondary import UAVSecondary


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
    ts_3.transform.translation.x = (ts_tx1 * idx + ts_tx2) / (idx + 1)
    ts_3.transform.translation.y = (ts_ty1 * idx + ts_ty2) / (idx + 1)
    ts_3.transform.translation.z = (ts_tz1 * idx + ts_tz2) / (idx + 1)
    ts_3.transform.rotation.x = (ts_qx1 * idx + ts_qx2) / (idx + 1)
    ts_3.transform.rotation.y = (ts_qy1 * idx + ts_qy2) / (idx + 1)
    ts_3.transform.rotation.z = (ts_qz1 * idx + ts_qz2) / (idx + 1)
    ts_3.transform.rotation.w = (ts_qw1 * idx + ts_qw2) / (idx + 1)
    return ts_3


def sample_data(uav):
    """
    Collect sample data at a point and call a recursive update function.
    
    Parameters
    ----------
    uav : UAVSecondary instance
        The UAVSecondary instance of the current agent.

    Returns
    -------
    ts_sample : TransformStamped ROS message instance
        The average VICON localization data after N samples. N is defined in agents/{agent.name}/config.json
    """
    
    ts_sample = TransformStamped()
    for idx in range(uav.calibration_samples):
        ts_meas = rospy.wait_for_message(uav.topic_vicon, TransformStamped)
        ts_sample = update_sample(ts_sample, ts_meas, idx)
    return ts_sample

def run_calibration(uav):
    """
    Run the calibration and write the calibration file.

    Parameters
    ----------
    uav : UAVSecondary instance
        The UAVSecondary instance of the current agent.
    """

    print("When ready to sample pos1, execute rostopic pub /uav_name/messop/calibration1 std_msgs/Bool \"data: true\"")
    rospy.wait_for_message(f"/{uav.name}/messop/calibration1", Bool)
    ts_pos1 = sample_data(uav)

    print("When ready to sample pos2, execute rostopic pub /uav_name/messop/calibration2 std_msgs/Bool \"data: true\"")
    rospy.wait_for_message(f"/{uav.name}/messop/calibration2", Bool)
    ts_pos2 = sample_data(uav)

    theta = np.arctan2(
        ts_pos2.transform.translation.y - ts_pos1.transform.translation.y,
        ts_pos2.transform.translation.x - ts_pos1.transform.translation.x
    )
    q_pos1 = ts_pos1.transform.rotation
    q_pos2 = ts_pos2.transform.rotation

    q_0 = convert_eul2quat(0.0, 0.0, theta)
    q_1 = average_quats(q_pos1, q_pos2)
    q_1_inv = invert_quat(q_1)
    q_diff = multiply_quats(q_1_inv, q_0)

    file_path = os.path.join(get_path2agent(uav.name), f"calibration.json")
    content = {"x": q_diff.x, "y": q_diff.y, "z": q_diff.z, "w": q_diff.w}
    with open(file_path, "w") as file:
        json.dump(content, file)

def main():
    """
    Run the calibration of the agent's UAVSecondary instance.

    ...
    """

    rospy.init_node("uav_calibration")

    uav = UAVSecondary(get_agent_name())
    run_calibration(uav)

if __name__=="__main__":
    main()
