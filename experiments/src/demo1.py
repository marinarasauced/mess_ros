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

def run_experiment(burger2):
    """
    
    """
    
    points = np.array([
        [1.0, 2.0],
        [2.0, 0.0],
        [0.0, 0.0]
    ])
    counter = 0
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if burger2.status:
            print(f"new task: ({points[counter, 0]}, {points[counter, 1]})")
            msg = MESS2UGV()
            msg.pose.x = points[counter, 0]
            msg.pose.y = points[counter, 1]
            msg.pose.theta = 0
            msg.index = 2  # indicate linear translation operations
            burger2.pub_vertex.publish(msg)
            burger2.status = False
            counter += 1

        if counter > 2:
            counter = 0

        rate.sleep()

def main(experiment):
    """
    
    """

    try:
        # 01. ADD  VEHICLES (PRIMARY INSTANCES):
        burger2 = UGVPrimary("burger2", experiment)

        # 02. ADD TASKS (ROS NODES):
        ugv_vertex_node = ROSNode(
            package="messop_ugv", node_file="ugv_vertex.py", node_name="vertex"
        )
        burger2.add_node(ugv_vertex_node)

        # XX. OPTIONALLY CHANGE CONFIG FILES:
        burger2.write_config_file(
            calibration_samples=1000,
            max_lin_vel_ratio=0.8, max_ang_vel_ratio=0.4,
            error_tol_tx=0.02, error_tol_ty=0.04, error_tol_rz=0.015,
            occlusion_tol=0.1, 
            k_ty=4.6, k_rz=2.7
        )

        # 03. CREATE LIST OF AGENTS:
        ugvs = [burger2]
        uavs = []

        # 04. CONFIGURE AND RUN EXPERIMENT:
        status = run_experiment_setup(ugvs, uavs)
        if status:
            run_experiment(burger2)

        # 05. SHUTDOWN AGENTS AND DOWNLOAD LOGS
        shutdown_ros_except_vicon(experiment)
        if status:
            download_logs(ugvs + uavs, experiment)

    except Exception as e:
        print_task_error(f"{e}")
        rospy.signal_shutdown("")

if __name__=="__main__":
    experiment = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(experiment)
    main(experiment)
