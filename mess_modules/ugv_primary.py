
################################################################################
# PRIMARY UGV CLASS
#   Purpose:    coordinate operations of ugv agent from machine handling experiments
#   Inputs:  
#           01. name of agent 
#           02. name of experiment
#   Funcs:   
#           01. self.add_nodes
#           02. self.callback_flags
#           03. self.generate_config_file
#           04. self.generate_launch_file
################################################################################

import rospy
from mess_msgs.msg import MESS2UGV
from std_msgs.msg import Bool

import textwrap

from mess_modules.agents import get_primary_ugv_agent, write2agent
from mess_modules.cache import write2cache
from mess_modules.chatlogs import * 

class UGVPrimary():
    def __init__(self, ugv_name, experiment_name):
        
        # Input parameters:
        self.name = ugv_name
        self.experiment = experiment_name

        self.ip, self.username, self.password, self.tb3_model, self.lds_model = get_primary_ugv_agent(self.name)

        # ROS nodes (not including turtlebot3_bringup, messop_logger):
        self.node_names = []
        self.node_names.append(f"{self.name}_turtlebot3_diagnostics")
        self.node_names.append(f"{self.name}_logger")
        self.node_strings = []

        # ROS topics:
        self.flag_topic_primary = f"{self.name}/messop/vertex"
        self.flag_topic_secondary = f"{self.name}/messop/status"

        # ROS publishers:
        self.flag_ = rospy.Publisher(self.flag_topic_primary, MESS2UGV, queue_size=10)

        # ROS subscribers:
        self.status = False  # readiness to perform a task
        rospy.Subscriber(self.flag_topic_secondary, Bool, self.callback_flags)



    # Append list of custom node with input (NOT INCLUDING BRINGUP OR LOGGER).
    #   In:     _Node class object from _node.py
    #   Out:    appends self.nodes
    def add_node(self, node):
        self.node_names.append(f"{self.name}_{node.node_name}")
        self.node_strings.append(node.node)



    # Callback function for self.flag_topic with msgtype = Bool.
    #   In:     ros1 bool msg
    #   Out:    updates self.status with msg value
    def callback_flags(self, msg):
        if msg.data == True:
            self.status = True
        elif msg.data == False:
            self.status = False



    # Generates file content for UGV config file.
    #   In:     config parameters
    #   Out:    writes config.json to cache
    def generate_config_file(self, calibration_samples, max_lin_vel_ratio, max_ang_vel_ratio, error_tol_tx, error_tol_ty, error_tol_rz, occlusion_tol, k_ty, k_rz):
        file_content = f"""
        {{
            "ugv_name": "{self.name}",
            "calibration_samples": {calibration_samples},
            "max_lin_vel_ratio": {max_lin_vel_ratio},
            "max_ang_vel_ratio": {max_ang_vel_ratio},
            "error_tol_tx": {error_tol_tx},
            "error_tol_ty": {error_tol_ty},
            "error_tol_rz": {error_tol_rz},
            "occlusion_tol": {occlusion_tol},
            "k_ty": {k_ty},
            "k_rz": {k_rz}
        }}
        """

        file_content = textwrap.dedent(file_content).strip()
        write2agent(agent_name=self.name, file_name="config.json", file_content=file_content)

    

    # Generates experiment .launch file for UGV.
    #   In:     none (nodes must be added beforehand)
    #   Out:    writes {self.experiment}_{self.name}.launch to cache
    def generate_launch_file(self):
        first = f"""
        <launch>

            <arg name="ugv_name" default="{self.name}"/>

            <arg name="multi_robot_name" default=""/>
            <arg name="set_lidar_frame_id" default="base_scan"/>
            <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
                
            <node pkg="rosserial_python" type="serial_node.py" name="$(arg ugv_name)_turtlebot3_core" output="screen">
                <param name="port" value="/dev/ttyACM0"/>
                <param name="baud" value="115200"/>
                <param name="tf_prefix" value="$(arg multi_robot_name)"/>

                <remap from="battery_state" to="$(arg ugv_name)/battery_state"/>
                <remap from="cmd_vel" to="$(arg ugv_name)/cmd_vel"/>
                <remap from="cmd_vel_rc100" to="$(arg ugv_name)/cmd_vel_rc100"/>
                <remap from="diagnostics" to="$(arg ugv_name)/diagnostics"/>
                <remap from="firmware_version" to="$(arg ugv_name)/firmware_version"/>
                <remap from="imu" to="$(arg ugv_name)/imu"/>
                <remap from="joint_states" to="$(arg ugv_name)/joint_states"/>
                <remap from="magnetic_field" to="$(arg ugv_name)/magnetic_field"/>
                <remap from="motor_power" to="$(arg ugv_name)/motor_power"/>
                <remap from="odom" to="$(arg ugv_name)/odom"/>
                <remap from="reset" to="$(arg ugv_name)/reset"/>
                <remap from="sensor_state" to="$(arg ugv_name)/sensor_state"/>
                <remap from="sound" to="$(arg ugv_name)/sound"/>
                <remap from="tf" to="$(arg ugv_name)/tf"/>
            </node>
                
            <arg name="set_frame_id" default="base_scan"/>
            <arg name="lds_model" default="$(env LDS_MODEL)" doc="LDS MODEL [LDS-01, LDS-02]"/>

            <group if = "$(eval lds_model == 'LDS-01')">
                <node pkg="hls_lfcd_lds_driver" type="hlds_laser_publisher" name="$(arg ugv_name)_turtlebot3_lds" output="screen">
                    <param name="port" value="/dev/ttyUSB0"/>
                    <param name="frame_id" value="$(arg set_frame_id)"/>

                    <remap from="rpms" to="$(arg ugv_name)/rpms"/>
                    <remap from="scan" to="$(arg ugv_name)/scan"/>
                </node>
            </group>
            <group if = "$(eval lds_model == 'LDS-02')">
                <node pkg="ld08_driver" type="ld08_driver" name="$(arg ugv_name)_turtlebot3_lds" output="screen" args="LD08">
                    <param name="frame_id" value="$(arg set_frame_id)"/>

                    <remap from="rpms" to="$(arg ugv_name)/rpms"/>
                    <remap from="scan" to="$(arg ugv_name)/scan"/>
                </node>
            </group>
                
            <node pkg="turtlebot3_bringup" type="turtlebot3_diagnostics" name="$(arg ugv_name)_turtlebot3_diagnostics" output="screen"/>
        """
        second = """
            <node pkg="mess_logger" type="mess_logger.py" name="$(arg ugv_name)_logger" output="screen"/>

        </launch>
        """

        file_content = first
        for node in self.node_strings:
            file_content += node
        file_content += second

        file_content = textwrap.dedent(file_content).strip()
        write2cache(agent_name=self.name, file_name=f"{self.experiment}_{self.name}.launch", file_content=file_content)
