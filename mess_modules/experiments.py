
import rospy
import rosnode

import datetime
import os.path
import re
import subprocess
import time

from mess_modules.log2terminal import *
from mess_modules.paths import get_path2agents, get_path2experiments
from mess_modules.roslaunch import ROSLaunch
from mess_modules.scp import upload, download

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
    
    return 1

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
                f"source /opt/ros/noetic/setup.bash && source ~/mess_ros/devel/setup.bash && export ROS_MASTER_URI=http://192.168.0.229:11311 && export ROS_HOSTNAME={ugv.ip} && export TURTLEBOT3_MODEL={ugv.tb3_model} && export LDS_MODEL={ugv.lds_model} && roslaunch experiments {ugv.launch}"
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

    print_task_start("checking if all agent nodes are launched (may take up to 30 seconds)")
    rate = rospy.Rate(1)
    nodes = []
    for agent in agents:
        for node in agent.node_names:
            nodes.append(f"/{node}")

    tic = time.time()
    toc = tic + timeout
    while time.time() < toc:
        print_task_doing("waiting for all agent nodes to fully launch")
        running = rosnode.get_node_names()
        status = [1 if node in running else 0 for node in nodes]
        if all(status):
            print_task_done("all agent nodes are running")
            return 1
        rate.sleep()
    if not all(status):
        print_task_error("not all agent nodes are running")
        return 0
    if all(status):
        print_task_done("all agent nodes are running")
        return 1

def shutdown_ros_except_vicon(experiment):
    """
    Shutdown all ROS nodes except /vicon (master) at the end of an experiment.
    """

    print_task_start("shutting down all ros node except /vicon")
    vicon = "/vicon"
    rosout = "/rosout"
    experiment = f"/{experiment}"
    exceptions = [vicon, rosout, experiment]
    nodes = rosnode.get_node_names() 
    for node in nodes:
        if node not in exceptions:
            print_task_doing(f"shutting down {node}")
            killcmd = ["rosnode", "kill", node]
            subprocess.run(killcmd, stdout=subprocess.DEVNULL)
    print_task_done("shut down all ros nodes except /vicon")

def download_logs(agents, experiment):
    """
    Download all logs from all agents to ~/mess_ros/logs/experiment-trial-YYYY:MM:DD-HH:MM:SS/agent_name/.

    This function downloads the logs directory aboard each agent at the constant remote path "~/mess_ros/logs/" to subdir for the current trial of any given experiment.

    Parameters
    ----------
    agents : List of UGV and UAV Primary class type instances.
        A list of all ugvs and uavs from the experiment handler.
    experiment : str
        The name of the experiment for log path generation.
    """

    print_task_start("generating write path for current trial")
    write_path = get_write_path(experiment)
    for agent in agents:
        try:
            print_task_doing(f"downloading logs from {agent.name}")
            local_path = f"{write_path}/{agent.name}/"
            remote_path = "~/mess_ros/logs/"
            download(agent=agent, local_path=local_path, remote_path=remote_path)
        except:
            print_task_error(f"unable to download logs from {agent.name}")
            return 0
    print_task_done("downloaded logs from all agents")
    return 1

def get_write_path(experiment):
    """
    Get the absolute write path to the directory where the current trial's logs will be stored.

    Parameters
    ----------
    experiment : str
        The name of the current experiment i.e., demo1.

    Returns
    -------
    directory : str
        The absolute path to this trial's write directory.
    """

    local_path = os.path.expanduser("~/mess_ros/logs")
    date = datetime.strftime("%Y-m-%d")
    time = datetime.strftime("%H-%M-%S")
    trial = get_this_trial(local_path, experiment)
    directory = f"{trial}--{date}--{time}"
    return directory


def get_this_trial(path, experiment):
    """
    Read all trial directories on the input path and return the index of the current trial.

    This function looks at all directories in the mess_ros/logs/experiment/ directory and returns a string of the next trial i.e., if the last trial's directory started with 0002, this function will return 0003.

    Parameters
    ----------
    path : str
        The absolute path to the logs/ subdir.
    experiment : str
        The name of the current experiment.

    Returns
    -------
    str
        The index of the current trial incremented from the highest value in the logs/ subdir.
    """

    path2experiment = os.path.join(path, experiment)
    if not os.path.exists(path2experiment):
        os.makedirs(path2experiment)
        return "0001"
    past_trials = [name for name in os.listdir(path2experiment) if os.path.isdir(os.path.join(path2experiment, name))]
    last_index = 0
    for trial in past_trials:
        match = re.match(r'^(\d+)--', trial)
        if match:
            index = int(match.group(1))
            if index > last_index:
                last_index = index
    curr_index = last_index + 1
    return f"{curr_index:04d}"

