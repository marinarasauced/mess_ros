#!/usr/bin/env python3

import rospy
from mess_msgs.msg import MESS2UGV

import numpy as np
import os.path
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))

from mess_modules.experiments import run_experiment_setup, shutdown_ros_except_vicon, download_logs
from mess_modules.log2terminal import *
from mess_modules.rosnode import ROSNode
from mess_modules.ugv_primary import UGVPrimary

def run_experiment(burger2, burger3, wafflepi1):
    """
    
    """
    burger2_points = np.array([
        [-5, -0.56],
        [-5, 0.81],
        [-3.68, 0.81],
        [-3.68, -0.56]
    ])
    burger3_points = np.array([
        [-0.7, 2.64],
        [-1.39, 1.96],
        [-0.35, 1.43]
    ])
    wafflepi1_points = np.array([
        [-2.28, -0.38],
        [-2.31, 0.657],
        [-2.36, 2.02],[-2.31, 0.657]
    ])

    burger2_counter = 0
    burger3_counter = 0
    wafflepi1_counter = 0

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        # burger2 tasks
        if burger2.status:
            print(f"new task: ({burger2_points[burger2_counter, 0]}, {burger2_points[burger2_counter, 1]})")
            msg = MESS2UGV()
            msg.pose.x = burger2_points[burger2_counter, 0]
            msg.pose.y = burger2_points[burger2_counter, 1]
            msg.pose.theta = 0
            msg.index = 2  # indicate linear translation operations
            burger2.pub_vertex.publish(msg)
            burger2.status = False
            burger2_counter += 1
            if burger2_counter > len(burger2_points):
                burger2_counter = 0

        # burger3 tasks
        if burger3.status:
            print(f"new task: ({burger3_points[burger3_counter, 0]}, {burger3_points[burger3_counter, 1]})")
            msg = MESS2UGV()
            msg.pose.x = burger3_points[burger3_counter, 0]
            msg.pose.y = burger3_points[burger3_counter, 1]
            msg.pose.theta = 0
            msg.index = 2  # indicate linear translation operations
            burger3.pub_vertex.publish(msg)
            burger3.status = False
            burger3_counter += 1
            if burger3_counter > len(burger3_points):
                burger3_counter = 0

        # wafflepi1 tasks
        if wafflepi1.status:
            print(f"new task: ({wafflepi1_points[wafflepi1_counter, 0]}, {wafflepi1_points[wafflepi1_counter, 1]})")
            msg = MESS2UGV()
            msg.pose.x = wafflepi1_points[wafflepi1_counter, 0]
            msg.pose.y = wafflepi1_points[wafflepi1_counter, 1]
            msg.pose.theta = 0
            msg.index = 2  # indicate linear translation operations
            wafflepi1.pub_vertex.publish(msg)
            wafflepi1.status = False
            wafflepi1_counter += 1
            if wafflepi1_counter > len(wafflepi1_points):
                wafflepi1_counter = 0

        rate.sleep()

def main(experiment):
    """
    
    """

    try:
        # 01. ADD  VEHICLES (PRIMARY INSTANCES):
        burger2 = UGVPrimary("burger2", experiment)
        burger3 = UGVPrimary("burger3", experiment)
        wafflepi1 = UGVPrimary("wafflepi1", experiment)

        # 02. ADD TASKS (ROS NODES):
        ugv_control_node = ROSNode(
            package="messop_ugv", node_file="ugv_control.py", node_name="control"
        )
        burger2.add_node(ugv_control_node)
        burger3.add_node(ugv_control_node)
        wafflepi1.add_node(ugv_control_node)

        # XX. OPTIONALLY CHANGE CONFIG FILES:
        burger2.write_config_file(
            calibration_samples=1000,
            max_lin_vel_ratio=0.8, max_ang_vel_ratio=0.4,
            error_tol_tx=0.02, error_tol_ty=0.04, error_tol_rz=0.015,
            occlusion_tol=0.1, 
            k_ty=4.6, k_rz=2.7
        )

        # 03. CREATE LIST OF AGENTS:
        ugvs = [burger2, burger3, wafflepi1]
        uavs = []

        # 04. CONFIGURE AND RUN EXPERIMENT:
        if not run_experiment_setup(ugvs, uavs):
            return 0
        run_experiment(burger2, burger3, wafflepi1)

        # 05. SHUTDOWN AGENTS AND DOWNLOAD LOGS
        shutdown_ros_except_vicon(experiment)
        download_logs(ugvs + uavs, experiment)

    except Exception as e:
        print_task_error(f"{e}")
        rospy.signal_shutdown("")



if __name__=="__main__":
    experiment = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(experiment)
    main(experiment)
