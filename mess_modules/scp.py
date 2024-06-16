
################################################################################
# SCP FUNCTIONS
#   Purpose:    define functions using scp to communicate with agents
#   Funcs:   
#           01. upload
#           02. download
################################################################################

from scp import SCPClient
from mess_modules.chatlogs import *
from mess_modules.ssh import init_client



# Upload directory to an agent.
#   In:     agent of primary type class, read path, write path
#   Out:    directory uploaded to agent
def upload(primary, local_path, remote_path):
    try:
        ssh = init_client(
            host=primary.ip,
            user=primary.username,
            password=primary.password,
            port=-1
        )
    except:
        chatlog_error(f"could not connect to {primary.name}, check to ensure the agent is online")
        return 0

    scp = SCPClient(ssh.get_transport())
    scp.put(local_path=local_path, remote_path=remote_path, recursive=True)
    scp.close()
    ssh.close()
    return 1



# Downlaod directory from an agent.
#   In:     agent of primary type class, read path, write path
#   Out:    directory downloaded from agent
def download(primary, local_path, remote_path):
    try:
        ssh = init_client(
            host=primary.ip,
            user=primary.username,
            password=primary.password,
            port=-1
        )
    except:
        chatlog_error(f"could not connect to {primary.name}, check to ensure the agent is online")
        return 0

    scp = SCPClient(ssh.get_transport())
    scp.get(local_path=local_path, remote_path=remote_path, recursive=True)
    scp.close()
    ssh.close()
    return 1