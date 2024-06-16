
################################################################################
# SSH FUNCTIONS
#   Purpose:    define functions using ssh to communicate with agents
#   Funcs:   
#           01. init_client
################################################################################

from paramiko import SSHClient, AutoAddPolicy



# Generates secure shell client with an agent.
#   In:     hostname, username, password, port
#   Out:    secure shell client
def init_client(host, user, password, port):
    ssh = SSHClient()
    ssh.load_system_host_keys()
    ssh.set_missing_host_key_policy(AutoAddPolicy())
    if port == -1:
        ssh.connect(hostname=host, username=user, password=password)
    else:
        ssh.connect(hostname=host, port=port, username=user, password=password)
    return ssh
