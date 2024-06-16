
################################################################################
### ROS NODE: EXPERIMENT DEMO ONE
#   Purpose:    demonstrate experimental framework with a single ugv agent 
#                   driving in a triangle formation for 60 seconds
#   Structure:  
#           01. initialize ros node
#           02. clear experiment cache
#           03. define ros nodes
#           04. define agents of primary type class
#           05. add nodes to agents
#           06. write cache files for agents
#           07. create list of agents
#           08. check vicon_bridge ros node status
#           09. upload experiment cache to agents
#           10. launch ros nodes on agents
#           11. check statuses of all agent ros nodes
#           12. run experiment
#           13. download logs
#   Notes:
#           01. to build an own experiment, modify steps 03. through 07., 12.,
#           02. to run an experiment, add a .launch file to the launch directory
#           03. before running an experiment for the first time, ensure that 
#                   the name of the experiment file, i.e., demo1.py is in
#                   experiments/CMakeLists.txt -> catkin_install_python(); then,
#                   compile the ros environment by running the following command 
#                   in a terminal: cd ~/mess_ros && catkin_make
#           04. agents use a flagging system ...
################################################################################

import rospy
from mess_msgs.msg import MESS2UGV

import numpy as np
import os.path
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))

from mess_modules.agents import launch_agents, upload_config, check4agent_nodes, download_logs
from mess_modules.cache import clear_cache, upload_cache
from mess_modules.chatlogs import *
from mess_modules.experiments import download_logs
from mess_modules.node import _Node
from mess_modules.vicon import check4viconbridge
from mess_modules.ugv_primary import UGVPrimary



#
#
#
def run_experiment(burger2):
    chatlog_task("all check conditions met, running experiment")

    waypoints = np.array([
        [1.0, 2.0],
        [2.0, 3.0],
        [4.0, 3.1]
    ])
    burger2.status = True

    rate = rospy.Rate(10)
    counter = 0
    while not rospy.is_shutdown():
        print("hi")

        if burger2.status:
            msg = MESS2UGV()
            msg.Tx = waypoints[counter, 0]
            msg.Ty = waypoints[counter, 1]
            msg.Rz = 0.0
            msg.Op = 2
            burger2.flag_.publish(msg)
            burger2.status = False
            counter += 1
            if counter > len(waypoints) - 1:
                counter = 0



        rate.sleep()  # controls iter duration




#############################
# DO NOT EDIT BELOW THIS LINE
################################################################################



#
#
#
def main():

    # 01. node is initialized as name of file; i.e., demo1:
    experiment_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(experiment_name)

    # 02. clear experiment cache:
    clear_cache()

################################################################################
# DO NOT EDIT ABOVE THIS LINE
#############################

    # 03. define ros nodes:
    ugv_control = _Node(package="messop_ugv", node_file="ugv_control.py", node_name="control")

    # 04. define ugv and uav agents:
    burger2 = UGVPrimary("burger2", experiment_name)

    # 05. add nodes to agents:
    burger2.add_node(ugv_control)

    # 06. generate config and launch files for each agent:
    burger2.generate_config_file(
        calibration_samples=1000,
        max_lin_vel_ratio=0.5,
        max_ang_vel_ratio=0.3,
        error_tol_tx=0.02,
        error_tol_ty=0.04,
        error_tol_rz=0.01,
        occlusion_tol=0.1,
        k_ty=4.6092,
        k_rz=2.6779
    )   # optional if config file already exists or ugv agent uses dynamics other than messop_ugv/src/ugv_control.py
    burger2.generate_launch_file()

    # 07. create list of ugvs, uavs, and agents:
    ugvs = [burger2]
    uavs = []
    agents = ugvs + uavs
    
#############################
# DO NOT EDIT BELOW THIS LINE
################################################################################

    # 08. check for vicon_bridge node:
    if not check4viconbridge(agents):
        pass
    #    return

    # 09. upload experiments cache and config to agents:
    if not upload_cache(agents):
        pass
    #    return

    if not upload_config(agents):
        pass
    #    return

    # 10. launch ros nodes onboard agents:
    if not launch_agents(ugvs, uavs):
        pass
    #    return

    # 11. check for agent nodes:
    if not check4agent_nodes(agents):
        pass
    #    return

################################################################################
# DO NOT EDIT ABOVE THIS LINE
#############################

    # 12. run experiment:
    run_experiment(burger2)

#############################
# DO NOT EDIT BELOW THIS LINE
################################################################################

    # 13. download logs:
    if not download_logs(agents):
        pass

    # XX. shutdown rospy
    rospy.signal_shutdown("experiment completed")




#
if __name__=="__main__":
    main()
