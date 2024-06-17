#!/usr/bin/env python

import rospy
from mess_msgs.msg import MESS2UGV

import numpy as np
import os.path
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))

from mess_modules.experiments import run_experiment_setup
from mess_modules.log2terminal import *
from mess_modules.node import ROSNode
from mess_modules.ugv_primary import UGVPrimary

def run_experiment(burger2):
    """
    
    """
    
    points = np.array([
        [1.0, 2.0],
        [2.0, 3.0],
        [0.0, 0.0]
    ])
    counter = 0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if burger2.status:
            msg = MESS2UGV()
            msg.pose.x = points[counter, 0]
            msg.pose.y = points[counter, 1]
            msg.pose.theta = 0
            msg.index = 2  # indicate linear translation operations
            burger2.pub_vertex.publish(msg)
            burger2.status = False

        if counter > 2:
            counter = 0

        rate.sleep()
        rospy.spin()

def main(experiment):
    """
    
    """

    try:
        # 01. DEFINE PRIMARY INSTANCES:
        burger2 = UGVPrimary("burger2", experiment)

        # 02. ADD NODES TO PRIMARY INSTANCES:
        ugv_control_node = ROSNode(
            package="messop_ugv", node_file="ugv_control.py", node_name="control"
        )
        burger2.add_node(ugv_control_node)

        # XX. OPTIONALLY CHANGE CONFIG FILES
        burger2.write_config_file(
            calibration_samples=1000,
            max_lin_vel_ratio=0.5, max_ang_vel_ratio=0.3,
            error_tol_tx=0.02, error_tol_ty=0.04, error_tol_rz=0.01,
            occlusion_tol=0.1, 
            k_ty=4.6, k_rz=2.7
        )

        # 03. ADD PRIMARY INSTANCES TO LISTS OF AGENTS:
        ugvs = [burger2]
        uavs = []

######### DO NOT MODIFY BELOW THIS LINE UNLESS YOU ARE CERTAIN
################################################################

        if not run_experiment_setup(ugvs, uavs):
            return
        else:
            run_experiment(burger2)

        # download_logs(agents)

    except Exception as e:
        print_task_error(f"{e}")
        rospy.signal_shutdown("")



if __name__=="__main__":
    experiment = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(experiment)
    main(experiment)
