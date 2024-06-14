
import datetime
import json
import os.path
import subprocess

from mess_modules.vehicles import *
from mess_modules.nodes import *

def clear_cache():
    path2cache = get_path2cache()
    subprocess.run(["rm", "-rf", path2cache])

def get_path2cache():
    path2cache = os.path.abspath(os.path.join(os.path.dirname(__file__), "../experiments/cache/"))
    return path2cache

def get_path2experiments():
    path2cache = os.path.abspath(os.path.join(os.path.dirname(__file__), "../experiments/"))
    return path2cache

def get_path2logs():
    path2logs = os.path.abspath(os.path.join(os.path.dirname(__file__), "../logs/"))

def get_path2write(name):
    path2cache = get_path2cache()
    path2write = os.path.abspath(os.path.join(path2cache, f"{name}/"))
    if not os.path.exists(path2write):
        os.makedirs(path2write)
    return path2write

def load_ugv(name, experiment):
    path2client = os.path.abspath(os.path.join(os.path.dirname(__file__), f"../../mess_modules/clients/{name}.json"))
    f_ = open(path2client)
    f_data = json.load(f_)
    ugv = UGV(
        name=f_data["name"], 
        ip=f_data["ip"], 
        password=f_data["password"], 
        experiment=experiment,
        tb3_model=f_data["model"],
        lds_model=f_data["lds"]
    )
    return ugv

def launch_vehicles(ugvs, uavs):
    for ugv in ugvs:
        upload_cache(ugv)
        launch_ugv(ugv)
    for uav in uavs:
        pass

def upload_cache(vehicle):
    local_path = get_path2experiments()
    remote_path = "~/mess_ros/src/mess_ros/"
    upload(vehicle, local_path, remote_path)

def download_logs(vehicle):
    path2logs = get_path2logs()
    write_path = os.path.join(path2logs, datetime.datetime.now().strftime("%Y-%m-%d"))
    write_dir = get_write_directory_index(write_path)

def get_write_directory_index(write_path):
    