
from mess_modules.log2terminal import print_task_doing
from mess_modules.ssh import get_client

class ROSLaunch():
    def __init__(self, agent, commands):
        self.agent = agent
        self.commands = commands
        self.ssh = get_client(
            host=self.agent.ip,
            user=self.agent.username,
            password=self.agent.password,
            port=-1
        )
        self.execute()

    def execute(self):
        for command in self.commands:
            print_task_doing(f"on {self.agent.name} executing {command}")
            self.ssh.exec_command(command)

    def close(self):
        self.ssh.close()
