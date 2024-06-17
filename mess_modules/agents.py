
import rospy
from geometry_msgs.msg import Quaternion

import json
import os.path

from mess_modules.paths import get_path2agent

def load_json(name, file_name):
    """
    Load  data from a JSON file.

    Parameters
    ----------
    name : str
        Name of the agent whose data is to be loaded.

    Returns
    -------
    dict
        Dictionary containing agent's loaded data.
    """

    path2agent = get_path2agent(name)
    file_path = os.path.join(path2agent, file_name)
    with open(file_path, "r") as file:
        load = json.load(file)
    data = {}
    for idx, key in enumerate(load.keys()):
        data[key] = idx
    return data

def load_ugvprimary(name):
    """
    Load a ugv agent's data from a JSON file for a UGVPrimary instance.

    Parameters
    ----------
    name : str
        Name of the agent whose data is to be loaded.

    Returns
    -------
    ip : str
        The IP address of the agent's remote SSH server.
    username : str
        The agent's username.
    password : str
        The agent's password.
    tb3_model : str
        The agent's turtlebot3 model i.e., burger, waffle, waffle_pi.
    lds_model : str
        The agent's lds model i.e., LDS-01, LDS-02
    priority : int
        The agent's priority in collision avoidance. Smaller integers have greater priority.
    """
    
    data = load_json(name, f"{name}.json")
    ip = data["ip"]
    username = data["username"]
    password = data["password"]
    tb3_model = data["tb3_model"]
    lds_model = data["lds_model"]
    priority = data["priority"]
    return ip, username, password, tb3_model, lds_model, priority

def load_uavprimary(name):
    """
    """
    pass

def get_ugv_velocities():
    """
    Get the hardware-specified maximum allowable linear and angular velocities.

    Returns
    -------
    ugv_max_lin_vel : float
        Hardware-specified maximum allowable linear velocity.
    ugv_max_ang_vel : float
        Hardware-specified maximum allowable angular velocity.
    """
    model = rospy.get_param("model", "burger")
    if model == "burger":
        ugv_max_lin_vel = 0.22  # manufacturer specified
        ugv_max_ang_vel = 2.84  # manufacturer specified
    elif model == "waffle" or model == "waffle_pi":
        ugv_max_lin_vel = 0.26  # manufacturer specified
        ugv_max_ang_vel = 1.82  # manufacturer specified
    else:
        ugv_max_lin_vel = 0.22
        ugv_max_ang_vel = 1.82
    return ugv_max_lin_vel, ugv_max_ang_vel

def load_ugvsecondary(name):
    """
    Load a ugv agent's data from a JSON file for a UGVPrimary instance.

    Parameters
    ----------
    name : str
        Name of the agent whose data is to be loaded.

    Returns
    -------
    calibration_samples : int
        The number of samples to collect at each calibration point.
    max_lin_vel : float
        The agent's maximum linear velocity.
    max_ang_vel : float
        The agent's maximum angular velocity.
    error_tol_tx : float
        The error tolerance parallel to the agent's direction of motion during a linear translation. Units are meters.
    error_tol_ty : float
        The error tolerance perpendicular to the agent's direction of motion during a linear translation. Units are meters.
    error_tol_rz : float
        The error tolerance during a rotation. Units are radians.
    occlusion_tol : float
        The number of seconds the agent can rely on VICON localization after being occluded from the VICON cameras before switching to an onboard state estimate.
    k_ty : float
        The state-feedback controller gain for local y-pos error.
    k_rz : float
        The state-feedback controller gain for local z-rot error.
    """

    data = load_json(name, f"config.json")
    calibration_samples = data["calibration_samples"]
    max_lin_vel_ratio = data["max_lin_vel_ratio"]
    max_ang_vel_ratio = data["max_ang_vel_ratio"]
    error_tol_tx = data["error_tol_tx"]
    error_tol_ty = data["error_tol_ty"]
    error_tol_rz = data["error_tol_rz"]
    occlusion_tol = data["occlusion_tol"]
    k_ty = data["k_ty"]
    k_rz = data["k_rz"]
    max_lin_vel, max_ang_vel = get_ugv_velocities()
    max_lin_vel *= max_lin_vel_ratio
    max_ang_vel *= max_ang_vel_ratio
    return calibration_samples, max_lin_vel, max_ang_vel, error_tol_tx, error_tol_ty, error_tol_rz, occlusion_tol, k_ty, k_rz

def load_calibration(name):
    """
    Load an agent's calibration data.

    Parameters
    ----------
    name : str
        Name of the agent whose data is to be loaded.

    Returns
    -------
    q_diff : Quaternion ROS message instance
        Difference quaternion used to callibrate VICON orientation callbacks.
    """

    data = load_json(name, "calibration.json")
    q_diff = Quaternion()
    q_diff.x = data["x"]
    q_diff.y = data["y"]
    q_diff.z = data["z"]
    q_diff.w = data["w"]
    return q_diff