
################################################################################
# EXPERIMENT FUNCTIONS
#   Purpose:    define functions relating to the mess_ros experiments
#   Funcs:   
#           01. get_path2experiments
################################################################################

import rospy

import os.path



# Gets path to experiments directory.
#   In:     none
#   Out:    returns path to experiments directory in mess_ros
def get_path2experiments():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "../experiments/"))

