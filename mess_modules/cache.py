
################################################################################
# CACHE FUNCTIONS
#   Purpose:    define functions relating to the mess_ros cache
#   Funcs:   
#           01. get_path2cache
#           02. write2cache
#           03. clear_cache
#           04. upload_cache
################################################################################

import os.path
import subprocess

from mess_modules.chatlogs import *
from mess_modules.experiments import get_path2experiments
from mess_modules.misc import check4path
from mess_modules.scp import upload



# Get path to cache directory.
#   In:     none
#   Out:    returns path to cache directory in mess_ros
def get_path2cache():
    path2experiments = get_path2experiments()
    return os.path.abspath(os.path.join(path2experiments, "cache/"))



# Write launch and config descriptions to cache.
#   In:     none
#   Out:    returns path to cache directory in mess_ros
def write2cache(agent_name, file_name, file_content):
    path2cache = get_path2cache()
    path2file = os.path.join(path2cache, f"{file_name}")
    
    check4path(path2cache)
    with open(path2file, "w") as file:
        file.write(file_content)



# Delete all files in experiments/cache/.
#   In:     none
#   Out:    empties cache directory
def clear_cache():
    path2cache = get_path2cache()
    subprocess.run(["rm", "-rf", path2cache])
    return 0



# Upload experiments cache to agents.
#   In:     list of agents
#   Out:    experiments cache uploaded to agents
def upload_cache(agents):
    chatlog_task("uploading experiment cache to agents")
    path2experiments = get_path2experiments()
    remote_path = "~/mess_ros/src/mess_ros/"
    for agent in agents:
        try:
            chatlog_subtask(f"uploading experiment cache to {agent.name}")
            if not upload(primary=agent, local_path=path2experiments, remote_path=remote_path):
                return 0
        except:
            chatlog_error(f"unable to upload experiment cache to {agent.name}")
            return 0
    return 1

