
import json
import os.path

##################################################################################

# Retrieves hardware-specified maximum velocities.
#   Input:  1x rospy string parameter
#   Output: 2x maximum linear and angular velocity floats
def get_max_velocities(model):
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

# Retrieves vehicle-specific config parameters.
#   Output: 1x structure containing all config parameters
def load_config_from_node():
    config_path = os.path.abspath(os.path.dirname(__file__) + "/../../config/")
    f_ = open(config_path + "config.json")
    f_data = json.load(f_)

    return f_data

# Retrieves vehicle-specific calibration parameters.
#   Output: 1x structure containing all config parameters
def load_calibration_from_node():
    config_path = os.path.abspath(os.path.dirname(__file__) + "/../../config/")
    c_ = open(config_path + "calibration.json")
    c_data = json.load(c_)

    return c_data
