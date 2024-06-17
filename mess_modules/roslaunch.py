
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
            stdin, stdout, stderr = self.ssh.exec_command(command)
            #print(stderr.read())
