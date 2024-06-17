
from paramiko import SSHClient, AutoAddPolicy

def get_client(host, user, password, port):
    """
    Initialize an SSHClient instance to a remote agent using SSH key-based authentication.

    Parameters
    ----------
    host : str
        The IP address of the agent's remote SSH server.
    user: str
        The agent's username.
    password: str
        The agent's password.
    port : int
        The port number to connect to on the agent.

    Returns
    -------
    SSHClient
        An SSHClient instance connected to the specified agent.

    See Also
    --------
    upload : Upload a file or directory from the local machine to a remote agent via SSH using SCP.
    download : Download a file or directory from a remote agent via SSH using SCP to the local machine.
    """

    ssh = SSHClient()
    ssh.load_system_host_keys()
    ssh.set_missing_host_key_policy(AutoAddPolicy())
    if port == -1:
        ssh.connect(
            hostname=host, 
            username=user,
            password=password
        )
    else:
        ssh.connect(
            hostname=host, 
            username=user,
            password=password,
            port=port
        )
    return ssh