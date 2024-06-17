
import os.path

def get_path2agents():
    """
    Get the absolute path to the agents/ directory in mess_ros/.

    Returns
    -------
    str
        The absolute path to mess_ros/agents/.

    See Also
    --------
    get_path2agent : Get the absolute path to an agents subdir in agents/.
    """

    return os.path.abspath(os.path.join(os.path.dirname(__file__), "../agents/"))

def get_path2agent(name):
    """
    Get the absolute path to an agents subdir in agents/.

    Parameters
    ----------
    name : str
        An agent's name.

    Returns
    -------
    str
        The absolute path to mess_ros/agents/{name}/.

    See Also
    --------
    get_path2agents : Get the absolute path to the agents/ directory in mess_ros/.
    """

    path2agents = get_path2agents()
    return os.path.join(path2agents, name)

def get_path2experiments():
    """
    Get the absolute path to the experiments/ directory in mess_ros/.

    Returns
    -------
    str
        The absolute path to mess_ros/experiments/.

    See Also
    --------
    get_path2cache : Get the absolute path to an cahce subdir in experiments/.
    """

    return os.path.abspath(os.path.join(os.path.dirname(__file__), "../experiments/"))

def get_path2cache():
    """
    Get the absolute path to an cache subdir in experiments/.

    Returns
    -------
    str
        The absolute path to mess_ros/experiments/cache/.

    See Also
    --------
    get_path2experiments : Get the absolute path to the experiments/ directory in mess_ros/.
    """

    path2experiments = get_path2experiments()
    return os.path.join(path2experiments, "cache/")

def check4path(path):
    """
    If a path does not exist, make directories.

    Parameters
    ----------
    path : str
        The absolute path that is being checked for.
    """

    if not os.path.exists(path):
        os.makedirs(path)

def write_file2path(path, content):
    """
    Write a file to an agent's subdir in mess_ros/agents/.

    Parameters
    ----------
    name : str
        The agent's name.
    path : str
        The path of the file that is to be written including its name.
    content : str
        The content of the file that is to be written.
    """

    with open(path, "w") as file:
        file.write(content)

