
import json
import os.path

def get_path2cache():
    path2cache = os.path.abspath(os.path.join(os.path.dirname(__file__), "../experiments/cache/"))
    return path2cache

class UGV():
    def __init__(self, name):
        self.name = name
        
        f_path = os.path.abspath(os.path.join(os.path.dirname(__file__), f"vehicles/{self.name}.json"))
        f_ = open(f_path)
        f_data = json.load(f_)
        self.ip = f_data["ip"]

        self.nodes = []
        self.path2cache = get_path2cache()
        self.path2write = get_path2write

    #
    def add_node(self, node):
        self.nodes.append(node.node)




    #
    def generate_launch_description(self):
        if self.nodes is None:
            SyntaxError(f"{self.name} is missing nodes.")
        content1 = f"""
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
        content2 = """
            <node pkg="ugv_control" type="logger_legacy.py" name="$(arg ugv_name)_logger" output="screen"/>

        </launch>
        """

        content = content1
        for node in self.nodes:
            content += node
        content += content2

        self.launch_description = content

