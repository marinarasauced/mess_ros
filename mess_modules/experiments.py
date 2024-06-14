
import os.path
import subprocess

def clear_cache():
    path2cache = get_path2cache()
    subprocess.run(["rm", "-rf", path2cache])

def get_path2cache():
    path2cache = os.path.abspath(os.path.join(os.path.dirname(__file__), "../experiments/cache/"))
    return path2cache

def get_path2experiments():
    path2cache = os.path.abspath(os.path.join(os.path.dirname(__file__), "../experiments/"))
    return path2cache

def get_path2write(name):
    path2cache = get_path2cache()
    path2write = os.path.abspath(os.path.join(path2cache, f"{name}/"))
    if not os.path.exists(path2write):
        os.makedirs(path2write)
    return path2write