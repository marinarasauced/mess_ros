
import json
import os.path

##################################################################################

# Retrieves vehicle-specific config parameters.
#   Output: 1x structure containing all config parameters
def load_config_from_node():
    config_path = os.path.abspath(os.path.dirname(__file__) + "/../config/")
    f_ = open(os.path.join(config_path, "config.json"))
    f_data = json.load(f_)

    return f_data

# Retrieves vehicle-specific calibration parameters.
#   Output: 1x structure containing all config parameters
def load_calibration_from_node():
    config_path = os.path.abspath(os.path.dirname(__file__) + "/../config/")
    c_ = open(os.path.join(config_path, "calibration.json"))
    c_data = json.load(c_)

    return c_data
