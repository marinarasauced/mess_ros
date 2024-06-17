
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
            print(f"Executing command on {self.agent.name}: {command}")
            stdin, stdout, stderr = self.ssh.exec_command(command)
            error_output = stderr.read().decode('utf-8')
            if error_output:
                print(f"Error on {self.agent.name}: {error_output}")
            else:
                print(f"Command executed successfully on {self.agent.name}")

    def close(self):
        self.ssh.close()
