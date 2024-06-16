
################################################################################
### ROS NODE: MESS LOGGER
#   Purpose:    log topics to csv
#   Funcs:   
#           01. clear_logs
#           02. write_logs
################################################################################

import rospy

import os.path
import shutil

class LogThisTopic():
    def __init__(self, topic, msgtype):
        self.topic = topic      # "/some/topic"
        self.msgtype = msgtype  # "Imu"

        rospy.on_shutdown(self.write_logs()) # easier to put this here than outside of loop since itll write for all objects

        # need to define a way to get a subclass for the msgtype, probs should put subclasses in mess_modules.messages


    #
    #
    #
    def write_logs(self):
        pass


# Clear log files from ~/mess_ros/logs/ on agent.
#   In:     none
#   Out:    empties directory on path
def clear_logs(path2logs):
    if os.path.exists(path2logs) and os.path.isdir(path2logs):
        shutil.rmtree(path2logs)
    os.makedirs(path2logs)



# ROS NODE
def main():

    # Initialize node:
    rospy.init_node("mess_logger")

    # Clear logs:
    path2logs = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../logs/"))
    clear_logs(path2logs)

    # Initialize subscribers (loggers):




    #
    while not rospy.is_shutdown():
        l = 1

if __name__=="__main__":
    main()