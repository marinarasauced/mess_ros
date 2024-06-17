
class ROSNode():
    def __init__(self, package, node_file, node_name):
        self.package = package
        self.node_file = node_file
        self.node_name = node_name
        self.node_string = f"    <node pkg=\"{self.package}\" type=\"{self.node_file}\" name=\"$(arg ugv_name)_{self.node_name}\" output=\"screen\"/>"
