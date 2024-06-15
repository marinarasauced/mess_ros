
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
    print("\n >>> [Task:   launching all vehicles]")
    for ugv in ugvs:
        print(f" >>> [Doing:  launching {ugv.name}]")
        upload_cache(ugv)
        launch_ugv(ugv)
        print(f" >>> [Status: launched {ugv.name}]")
    for uav in uavs:
        print(f" >>> [Doing:  launching {uav.name}]")
        upload_cache(uav)
        # launch_ugv(ugv)
        print(f" >>> [Status: launched {uav.name}]")
        pass
    print(" >>> [Done:   launched all vehicles]")

def upload_cache(vehicle):
    local_path = get_path2experiments()
    remote_path = "~/mess_ros/src/mess_ros/"
    upload(vehicle, local_path, remote_path)

def download_logs(vehicles, write_directory):
    write_directory = get_write_directory()
    remote_path = "~/mess_ros/logs/"
    for vehicle in vehicles:
        local_path = os.path.join(write_directory, f"{vehicle.name}/")
        download(vehicle, local_path, remote_path)

def get_write_directory():
    path2logs = get_path2logs()
    write_path = os.path.join(path2logs, datetime.datetime.now().strftime("%Y-%m-%d"))
    now = datetime.datetime.now().strftime("%H-%M-%S")
    trials = [d for d in os.listdir(write_path) if os.path.isdir(os.path.join(write_path, d))]
    last_trial = max([int(d.split('-')[0]) for d in trials]) if trials else 0
    this_trial = last_trial + 1
    this_trial = f"{this_trial:03d}"[-3:]
    write_directory = f"{this_trial}-{now}"
    write_directory = os.path.join(write_path, write_directory)
    if not os.path.exists(write_directory):
        os.makedirs(write_directory)
    return write_directory