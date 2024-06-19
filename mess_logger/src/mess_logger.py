#!/usr/bin/env python3
# Author: 

import rospy
import rosgraph
from actionlib.msg import *
from actionlib_msgs.msg import *
from bond.msg import *
from diagnostic_msgs.msg import *
from dynamic_reconfigure.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mess_msgs.msg import *
from nav_msgs.msg import *
from roscpp.msg import *
from rosgraph_msgs.msg import *
from sensor_msgs.msg import *
from shape_msgs.msg import *
from std_msgs.msg import *
from stereo_msgs.msg import *
from tf.msg import *
from tf2_msgs.msg import *
from trajectory_msgs.msg import *
from turtlebot3_msgs.msg import *
from visualization_msgs.msg import *

import csv
import os.path
import shutil
import sys

sys.path.append((os.path.join(os.path.expanduser("~"), "ros_msgs_parser")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))

import breaker as unfold
from mess_modules.agents import get_agent_name

class LogThisTopic():
    def __init__(self, topic):
        self.topic = topic
        self.msgtype = self.get_topic_type()
        self.cstr = unfold.process(str(self.msgtype), "msg")
        self.filepath = self.setup_log()
        rospy.Subscriber(self.topic, eval(self.msgtype), self.callback)

    def callback(self, msg):
        """
        Receive messages of self.msgtype on the self.topic topic.
        """
        
        list = []
        for field in self.cstr:
            try:
                field_data = str(eval(f"{field}"))
                if field_data.startswith("(") and field_data.endswith(")"):
                    field_data = field_data.replace(",", "")
                list.append(field_data)
            except Exception as e:
                list.append("")
            
        with open(self.filepath, "a", newline="") as file:
            writer = csv.writer(file, delimiter="\t")
            writer.writerow(list)
        list.clear()

    def get_topic_type(self):
        """
        Get the message type of a ROS topic.

        This function retrieves all active topics from the ROS master and compares them to the topic of the current LogThisTopic instance. If the topic matches one from the environment, this function returns the message type of that topic.

        Returns
        -------
        arr : str
            The string name of the message type of the instance's topic.
        """

        master = rosgraph.Master("/rostopic")
        topic_types = master.getTopicTypes()
        for topic, type in topic_types:
            if self.topic == topic:
                arr = type.split("/")
                return arr[-1]
            
    def setup_log(self):
        """
        Setup the write file so that seperate LogThisTopic instances do not receive the wrong callbacks.
        
        Returns
        -------
        filepath : str
            The absolute path to the log .csv file.
        """
            
        directory = os.path.expanduser("~/mess_ros/logs/")
        filename = self.topic[1:].replace("/", "_") + ".csv"
        filepath = os.path.join(directory, filename)
        if not os.path.exists(filepath):
            with open(filepath, "w", newline="") as file:
                writer = csv.writer(file, delimiter="\t")
                writer.writerow(self.cstr)
        return filepath

def init():
    """
    Configure the node for logging.

    If the path to the logs dir does not exist, this function will make directories. This function will also clear any logs or paths in the logs dir.
    """

    logpath = os.path.expanduser("~/mess_ros/logs/")
    if not os.path.exists(logpath):
        os.makedirs(logpath)
    for filename in os.listdir(logpath):
        filepath = os.path.join(logpath, filename)
        if os.path.isfile(filepath) or os.path.islink(filepath):
            os.unlink(filepath)
        elif os.path.isdir(filepath):
            shutil.rmtree(filepath)

def main():
    """
    Start the logger node.

    This function initializes a ROS logger node for the Modular Experiment Software System. For each topic identified for the current agent, a csv file will be written to the name "/this/is/your/topic" -> "this_is_your_topic.csv."
    """

    rospy.sleep(16)  # wait for other processes to start
    rospy.init_node("mess_logger")
    init()

    ns = get_agent_name()
    topics = []
    for topic, _ in rospy.get_published_topics():
        if ns in topic:
            topics.append(topic)

    loggers = [LogThisTopic(topics[i]) for i in range(len(topics))]

    rospy.spin()

if __name__=="__main__":
    main()
