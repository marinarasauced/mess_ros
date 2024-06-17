
import rospy
import rosnode

import time

from mess_modules.log2terminal import *
from mess_modules.paths import get_path2agents, get_path2experiments
from mess_modules.roslaunch import ROSLaunch
from mess_modules.scp import upload
from mess_modules.ssh import get_client

def run_experiment_setup(ugvs, uavs):
    """
    




    Returns
    -------
    int
        1 if the experiment is ready to start, 0 else

    See Also
    --------
    check_vicon_node : Check the status of the vicon_bridge node and topics.
    update_agents : Update all agents by pushing agents/ and experiments/ directories.
    launch_agents : Launch ROS nodes on all agents.
    wait_for_agent_nodes : Wait for agent nodes to be running.

    """
    agents = ugvs + uavs

    if not check_vicon_node(agents):
        return 0
    
    if not update_agents(agents):
        return 0
    
    if not launch_agents(ugvs, uavs):
        return 0
    
    if not wait_for_agent_nodes(agents, timeout=30):
        return 0

def check_vicon_node(agents):
    """
    Check the status of the vicon_bridge node and topics.

    Arguments
    ---------
    agents : list of UGVPrimary and UAVPrimary instances.
        The agents that are part of the experiment.

    Returns
    -------
    int 
        1 if vicon_bridge is running and there is an active topic for all agents, 0 else
    """

    print_task_start("checking if /vicon_bridge is running")
    nodes = rosnode.get_node_names()
    node = "/vicon"
    if not node in nodes:
        print_task_error(f"{node} does not appear to be running")
        return 0
    print_task_done(f"{node} is running")
    print_task_start(f"checking if {node} is publishing localization data for each agent")
    topics = rospy.get_published_topics()
    for agent in agents:
        print_task_doing(f"checking for /vicon/{agent.name}/{agent.name}")
        topic = f"/vicon/{agent.name}/{agent.name}"
        if not topic in [topic_ for topic_, _ in topics]:
            print_task_error(f"/vicon_bridge does not appear to be publishing {topic}")
            return 0
    print_task_done("/vicon bridge is publishing localization data for all agents")
    return 1

def update_agents(agents):
    """
    Update all agents by writing launch files and pushing agents/ and experiments/ directories recursively.

    Arguments
    ---------
    agents : list of UGVPrimary and UAVPrimary instances.
        The agents that are part of the experiment.
    
    Returns
    -------
    int
        1 if agents are updated, 0 else
    """

    print_task_start("writing agent launch files")
    for agent in agents:
        agent.write_launch_file()
    print_task_done("wrote agent launch files")

    print_task_start("updating agents")
    for agent in agents:
        try:
            print_task_doing(f"pushing agents/ experiments/ to {agent.name}")

            local_path2agents = get_path2agents()
            remote_path2agents = "~/mess_ros/src/mess_ros/"
            upload(agent=agent, local_path=local_path2agents, remote_path=remote_path2agents)
            
            local_path2experiments = get_path2experiments()
            remote_path2experiments = "~/mess_ros/src/mess_ros/"
            upload(agent=agent, local_path=local_path2experiments, remote_path=remote_path2experiments)
        except:
            print_task_error(f"unable to update {agent.name}")
            return 0
    print_task_done("updated all agents")
    return 1

def launch_agents(ugvs, uavs):
    """
    Launch ROS nodes on all agents.

    Arguments
    ---------
    ugvs : list of UGVPrimary instances.
        The UGVs that are part of the experiment.
    uavs : list of UAVPrimary instances.
        The UAVs that are part of the experiment.
    
    Returns
    -------
    int
        1 if agents are launched, 0 else
    """
    print_task_start("launching ugvs")
    for ugv in ugvs:
        try:
            commands = [
                "source /opt/ros/noetic/setup.bash && cd ~/mess_ros && catkin_make",
                f"source /opt/ros/noetic/setup.bash && source ~/mess_ros/devel/setup.bash && export ROS_MASTER_URI=http://192.168.0.229:11311 && export ROS_HOSTNAME={ugv.ip} && export TURTLEBOT3_MODEL={ugv.tb3_model} && export LDS_MODEL={ugv.lds_model} && nohup roslaunch experiments {ugv.launch} > /dev/null 2>&1 &"
            ]
            launcher = ROSLaunch(ugv, commands)
            launcher.close()
        except Exception as e:
            print_task_error(f"unable to launch {ugv.name}: {str(e)}")
            return 0
    print_task_done("launched ugvs")

    #print_task_start("launching uavs")
    #for uav in uavs:
    #    pass
    #print_task_done("launched uavs")

    return 1

def wait_for_agent_nodes(agents, timeout=30):
    """
    Wait for agent nodes to be running.

    Arguments
    ---------
    agents : list of UGVPrimary and UAVPrimary instances.
        The agents that are part of the experiment.
    
    Returns
    -------
    int
        1 if all agents nodes are running, 0 else
    """

    print_task_start("waiting for all agent nodes to start")
    rate = rospy.Rate(1)
    nodes = []
    for agent in agents:
        for node in agent.node_names:
            nodes.append(f"/{node}")
    print(nodes)

    tic = time.time()
    toc = tic + timeout
    while time.time() < toc:
        print_task_doing("checking if all agent nodes are running")
        running = rosnode.get_node_names()
        status = [1 if node in running else 0 for node in nodes]
        if all(status):
            break
        rate.sleep()
    if not all(status):
        print_task_error("not all agent nodes are running")
        return 0
    if all(status):
        print_task_done("all agent nodes are running")
        return 1




