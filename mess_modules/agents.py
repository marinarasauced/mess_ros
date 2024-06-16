
################################################################################
# AGENT FUNCTIONS
#   Purpose:    define functions relating to agents
#   Funcs:   
#           01. get_path2agents
#           02. get_path2agent
#           03. load_agent
#           04. get_primary_ugv_agent
#           05. get_primary_uav_agent
#           06. launch_agents
#           07. launch_ugv_agent
#           08. launch_uav_agent
#           09. write2agent
#           10. upload_config
#           11. check4agent_nodes
#           12. check4agent_nodes
################################################################################

import rosnode
import json
import os.path

from mess_modules.chatlogs import *
from mess_modules.misc import check4path
from mess_modules.scp import upload
from mess_modules.ssh import init_client



# Get path to agents directory.
#   In:     none
#   Out:    path to agents directory in mess_ros/
def get_path2agents():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "../agents/"))



# Get path to agent directory.
#   In:     name of agent
#   Out:    path to agent directory in mess_ros/agents/
def get_path2agent(name):
    return os.path.abspath(os.path.join(get_path2agents(), f"{name}/"))



# Load agent from agents directory.
#   In:     name of agent
#   Out:    dictionary of agent parameters
def load_agent(name):
    path2agent = get_path2agent(name)
    file_name = f"{name}.json"
    file_path = os.path.join(path2agent, file_name)
    with open(file_path, "r") as file:
        load = json.load(file)

    agent = {}
    for idx, key in enumerate(load.keys()):
        agent[key] = idx
    return agent



# Get ugv agent of primary class type.
#   In:     name of agent
#   Out:    ugv agent parameters
def get_primary_ugv_agent(name):
    load = load_agent(name)
    ip=load["ip"],
    username=load["username"],
    password=load["password"],
    tb3_model=load["tb3_model"],
    lds_model=load["lds_model"],
    return ip, username, password, tb3_model, lds_model



# Get uav agent of primary class type.
#   In:     
#   Out:    
def get_primary_uav_agent():
    pass



# Launch agents.
#   In:     list of ugv and uav primary class type objects
#   Out:    launches .launch file in cache onboard each agent
def launch_agents(ugvs, uavs):
    chatlog_task("launching agents")
    for ugv in ugvs:
        chatlog_subtask(f"launching {ugv.name}")
        if not launch_ugv_agent(ugv):
            chatlog_error(f"unable to launch {ugv.name}")
            return 0
    for uav in uavs:
        chatlog_subtask(f"launching {uav.name}")
        if not launch_ugv_agent(uav):
            chatlog_error(f"unable to launch {uav.name}")
            return 0
    chatlog_done("launched agents")
    return 1
        



#
#
#
def launch_ugv_agent(agent):
    try:
        ssh = init_client(host=agent.ip, user=agent.username, password=agent.password, port=-1)
    except:
        chatlog_error(f"could not connect to {agent.name}, check to ensure the agent is online")
        return 0
    ssh.exec_command(f"source /opt/ros/noetic/setup.bash && cd ~/mess_ros && catkin_make")
    ssh.exec_command(f"source /opt/ros/noetic/setup.bash && cd ~/mess_ros && catkin_make")
    stdin, stdout, stderr = ssh.exec_command(f"source /opt/ros/noetic/setup.bash && source ~/mess_ros/devel/setup.bash && export ROS_MASTER_URI=http://192.168.0.229:11311 && export ROS_HOSTNAME={agent.ip} && export TURTLEBOT3_MODEL={agent.tb3_model} && export LDS_MODEL={agent.lds_model} && roslaunch experiments {agent.launch_name}")
    print(stdout.read())
    print(stderr.read())
    ssh.close()
    return 1



#
#
#
def launch_uav_agent(uav):
    pass



# Write config descriptions to agent.
#   In:     none
#   Out:    returns path to cache directory in mess_ros
def write2agent(agent_name, file_name, file_content):
    path2agent = get_path2agent(agent_name)
    path2file = os.path.join(path2agent, f"{file_name}")
    
    check4path(path2agent)
    with open(path2file, "w") as file:
        file.write(file_content)



# Upload config to agents.
#   In:     list of agents
#   Out:    experiments cache uploaded to agents
def upload_config(agents):
    chatlog_task("uploading agent cache to agents")
    for agent in agents:
        try:
            chatlog_subtask(f"uploading config to {agent.name}")
            path2agent = os.path.join(get_path2agent(agent.name), "config.txt")
            remote_path = f"~/mess_ros/src/mess_ros/agents/{agent.name}/config.txt"
            if not upload(primary=agent, local_path=path2agent, remote_path=remote_path):
                return 0
        except:
            chatlog_error(f"unable to upload config to {agent.name}")
            return 0
    return 1



# Check for ros node in list of current nodes.
#   In:     desired ros node, list of currently running nodes
#   Out:    bool
def check4nodes(agent_nodes, env_nodes):
    agent_set = set(agent_nodes)
    env_set = set(env_nodes)
    if agent_set.issubset(env_set):
        return 1
    else:
        return 0


# Check ros environment to ensure agent nodes are running before experiment beings.
#   In:     list of primary type class agents
#   Out:    bool status of nodes
def check4agent_nodes(agents):
    chatlog_task("checking ros environment for agent nodes")
    agent_nodes = []
    for agent in agents:
        for node_name in agent.node_names:
            agent_nodes.append(f"/{node_name}")
    wait = 10
    status = 0
    rate = rospy.Rate(1)
    chatlog_subtask(f"checking for agent nodes, will timeout after {wait} seconds")
    try:
        for _ in range(wait):
            chatlog_subtask("checking for agent nodes")
            env_nodes = rosnode.get_node_names()
            status = check4nodes(agent_nodes, env_nodes)
            if status == 1:
                chatlog_done("confirmed agent nodes are running")
                for agent in agents:
                    agent.status = True
                return 1
            rate.sleep()
        else:
            chatlog_error("timeout while checking for agent nodes")
            return 0
    except KeyboardInterrupt:
        return 0


        