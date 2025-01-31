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

def run_experiment(burger2, burger3, wafflepi2):
    """
    
    """
    burger2_points = np.array([
        [-4.5, -0.25],
        [-4.5, 0.81],
        [-3.68, 0.81],
        [-3.68, -0.56]
    ])
    burger3_points = np.array([
        [-0.7, 2.64],
        [-1.39, 1.96],
        [-0.35, 1.43]
    ])
    wafflepi2_points = np.array([
        [-2.30, 0.20],
        [-2.32, 0.90],
        [-2.36, 2.17],
        [-2.32, 0.90],
    ])

    burger2_counter = 0
    burger3_counter = 0
    wafflepi2_counter = 0

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
            if burger2_counter > len(burger2_points) - 1:
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
            if burger3_counter > len(burger3_points) - 1:
                burger3_counter = 0

        # wafflepi2 tasks
        if wafflepi2.status:
            print(f"new task: ({wafflepi2_points[wafflepi2_counter, 0]}, {wafflepi2_points[wafflepi2_counter, 1]})")
            msg = MESS2UGV()
            msg.pose.x = wafflepi2_points[wafflepi2_counter, 0]
            msg.pose.y = wafflepi2_points[wafflepi2_counter, 1]
            msg.pose.theta = 0
            msg.index = 2  # indicate linear translation operations
            wafflepi2.pub_vertex.publish(msg)
            wafflepi2.status = False
            wafflepi2_counter += 1
            if wafflepi2_counter > len(wafflepi2_points) - 1:
                wafflepi2_counter = 0

        rate.sleep()

def main(experiment):
    """
    
    """

    try:
        # 01. ADD  VEHICLES (PRIMARY INSTANCES):
        burger2 = UGVPrimary("burger2", experiment)
        burger3 = UGVPrimary("burger3", experiment)
        wafflepi2 = UGVPrimary("wafflepi2", experiment)

        # 02. ADD TASKS (ROS NODES):
        ugv_vertex_node = ROSNode(
            package="messop_ugv", node_file="ugv_vertex.py", node_name="control"
        )
        burger2.add_node(ugv_vertex_node)
        burger3.add_node(ugv_vertex_node)
        wafflepi2.add_node(ugv_vertex_node)

        # 03. CREATE LIST OF AGENTS:
        ugvs = [burger2, burger3, wafflepi2]
        uavs = []

        # 04. CONFIGURE AND RUN EXPERIMENT:
        status = run_experiment_setup(ugvs, uavs)
        if status:
            run_experiment(burger2, burger3, wafflepi2)

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
