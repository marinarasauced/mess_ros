
################################################################################
# _NODE CLASS
#   Purpose:    store information about a node used in an experiment
#   Inputs:  
#           01. name of package where ros node is located; i.e., "mess_logger"
#           02. name of file containing ros node; i.e., "mess_logger.py"
#           03. name of node in ros environment excluding vehicle name which is 
#                   automatically added as a prefix; i.e., "logger"
################################################################################

class _Node():
    def __init__(self, package, node_file, node_name):
        self.package = package
        self.node_file = node_file
        self.node_name = node_name
        self.node = f"    <node pkg=\"{self.package}\" type=\"{self.node_file}\" name=\"$(arg ugv_name)_{self.node_name}\" output=\"screen\"/>"
