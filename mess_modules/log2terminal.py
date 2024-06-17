
def print_task_start(msg):
    print(f"\n >>> [Task:  {msg}]")

def print_task_doing(msg):
    print(f" >>> [Doing: {msg}]")

def print_task_done(msg):
    print("\033[92m" + f" >>> [Done:  {msg}]\n" + "\033[0m")

def print_task_error(msg):
    print("\033[91m" + f" >>> [Error: {msg}]" + "\033[0m")

def print_task_agent(msg, name):
    print(f" >>> [{name}: {msg}]")