
################################################################################
# VICON BRIDGE FUNCTIONS
#   Purpose:    define functions relating to vicon_bridge
#   Funcs:   
#           01. get_viconbridge_node_status
#           02. get_viconbridge_topic_status
#           03. check4viconbridge
################################################################################

import rospy
import rosnode

from mess_modules.chatlogs import *



#
#
#
def get_viconbridge_node_status():
    agent_node = "/vicon_bridge"
    nodes = rosnode.get_node_names()
    return agent_node in nodes



#
#
#
def get_viconbridge_topic_status(agent):
    agent_topic = f"/vicon/{agent.name}/{agent.name}"
    topics = rospy.get_published_topics()
    for topic, _ in topics:
        if topic == agent_topic:
            return True
    return False


#
#
#
def check4viconbridge(agents):
    chatlog_task("checking vicon_bridge node status")
    node_status = True
    node_status = get_viconbridge_node_status()
    if not node_status:
        chatlog_error("vicon_bridge offline, please open a separate terminal and type \"roslaunch vicon_bridge vicon.launch\"")
        return 0
    else:
        chatlog_done("vicon_bridge node is online")
        chatlog_task("verifying vicon topics are active for each agent")
        for agent in agents:
            chatlog_subtask(f"checking vicon topic for {agent.name}")
            
            topic_status = True
            topic_status = get_viconbridge_topic_status(agent)
            if not topic_status:
                chatlog_error(f"cannot find /vicon/{agent.name}/{agent.name}, please check that {agent.name} is being tracked in vicon tracker")
                return 0
        chatlog_done("verified vicon topics are active for all agents")
    return 1


