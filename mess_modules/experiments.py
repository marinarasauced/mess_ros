
################################################################################
# EXPERIMENT FUNCTIONS
#   Purpose:    define functions relating to the mess_ros experiments
#   Funcs:   
#           01. get_path2experiments
#           02. download_logs
################################################################################

import datetime
import os.path

from mess_modules.misc import check4path


# Gets path to experiments directory.
#   In:     none
#   Out:    returns path to experiments directory in mess_ros
def get_path2experiments():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "../experiments/"))



#
#
#
def download_logs(agents):
    
    path2log = os.path.join(path2logs, get_directory4exp)
    for agent in agents
    
    path2log = os.path.join(path2logs, get_directory4agent)
    check4path(path2logs)



#
#
#
def get_directory4exp():
    path2logs = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../logs"))
    date = datetime.datetime.now().strftime("%Y-%m-%d")
    time = datetime.datetime.now().strftime("%H-%M-%S")