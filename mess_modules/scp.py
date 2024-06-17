
from scp import SCPClient

from mess_modules.log2terminal import *
from mess_modules.ssh import get_client

def upload(agent, local_path, remote_path):
    """
    Upload a file or directory from the local machine to a remote agent via SSH using SCP.

    Parameters
    ----------
    agent : UGVPrimary or UAVPrimary
        An agent's corresponding object on the local machine running the experiment.
    local_path : str
        The path to a file or directory on the local machine.
    remote_path : str
        The path to a file or directory on the agent's remote SSH server.

    See Also
    --------
    get_client : Initialize an SSHClient instance to a remote agent using SSH key-based authentication.
    download : Download a file or directory from a remote agent via SSH using SCP to the local machine.
    """

    ssh = get_client(
        host=agent.ip,
        user=agent.username,
        password=agent.password,
        port=-1
    )
    scp = SCPClient(ssh.get_transport())
    scp.put(local_path, recursive=True, remote_path=remote_path)
    scp.close()
    ssh.close()

def download(agent, local_path, remote_path):
    """
    Download a file or directory from a remote agent via SSH using SCP to the local machine.

    Parameters
    ----------
    agent : UGVPrimary or UAVPrimary
        An agent's corresponding object on the local machine running the experiment.
    local_path : str
        The path to a file or directory on the local machine.
    remote_path : str
        The path to a file or directory on the agent's remote SSH server.

    See Also
    --------
    get_client : Initialize an SSHClient instance to a remote agent using SSH key-based authentication.
    upload : Upload a file or directory from the local machine to a remote agent via SSH using SCP.
    """

    ssh = get_client(
        host=agent.ip,
        user=agent.username,
        password=agent.password,
        port=-1
    )
    scp = SCPClient(ssh.get_transport())
    scp.get(remote_path=remote_path, local_path=local_path, recursive=True)
    scp.close()
    ssh.close()

