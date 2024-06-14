

class Node():
    def __init__(self, package, file_name, node_name):
        self.pkg = package
        self.type = file_name
        self.name = node_name
        self.node = f"    <node pkg=\"{self.pkg}\" type=\"{self.type}\" name=\"{self.name}\" output=\"screen\"/>"

