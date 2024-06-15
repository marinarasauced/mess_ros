
import rospy
from std_msgs.msg import Bool

from paramiko import SSHClient, AutoAddPolicy
from scp import SCPClient
import os.path
import textwrap

from mess_modules.experiments import get_path2cache, get_path2write
from mess_modules.nodes import *

def upload(vehicle, local_path, remote_path):
    ssh = init_client(host=vehicle.ip, user="ubuntu", password=f"{vehicle.password}", port=-1)
    scp = SCPClient(ssh.get_transport())
    scp.put(local_path, recursive=True, remote_path=remote_path)
    scp.close()
    ssh.close()

def download(vehicle, local_path, remote_path):
    ssh = init_client(host=vehicle.ip, user="ubuntu", password=f"{vehicle.password}", port=-1)
    scp = SCPClient(ssh.get_transport())
    scp.get(remote_path=remote_path, local_path=local_path, recursive=True) 
    scp.close()
    ssh.close()

def launch_ugv(vehicle):
    ssh = init_client(host=vehicle.ip, user="ubuntu", password=f"{vehicle.password}", port=-1)
    ssh.exec_command(f"cp ~/mess_ros/src/mess_ros/experiments/cache/{vehicle.name}/config.json ~/mess_ros/src/mess_ros/messop_ugv/config/config.json")
    ssh.exec_command(f"source /opt/ros/noetic/setup.bash && cd ~/mess_ros && catkin_make")
    stdin, stdout, stderr = ssh.exec_command(f"source /opt/ros/noetic/setup.bash && source ~/mess_ros/devel/setup.bash && export ROS_MASTER_URI=http://192.168.0.229:11311 && export ROS_HOSTNAME={vehicle.ip} && export TURTLEBOT3_MODEL={vehicle.tb3_model} && export LDS_MODEL={vehicle.lds_model} && roslaunch experiments {vehicle.launch_name}")
    print(stdout.read())
    print(stderr.read())
    ssh.close()
    print(1)

def init_client(host, user, password, port):
    ssh = SSHClient()
    ssh.load_system_host_keys()
    ssh.set_missing_host_key_policy(AutoAddPolicy())
    if port == -1:
        ssh.connect(host, username=user, password=password)
    else:
        ssh.connect(host, port, username=user, password=password)
    return ssh

class UGV():
    def __init__(self, name, ip, password, experiment, tb3_model, lds_model):
        self.name = name
        self.ip = ip
        self.password = password
        self.experiment = experiment
        self.tb3_model = tb3_model
        self.lds_model = lds_model
        
        self.nodes = []
        self.path2cache = get_path2cache()
        self.path2write = get_path2write(self.name)

        self.status = False
        self.add_subscriber(f"/{self.name}/messop/flag", Bool)

    #
    def add_node(self, node):
        self.nodes.append(node.node)

    def add_subscriber(self, topic, msgtype):
        rospy.Subscriber(topic, msgtype, self.callback_flag)

    #
    def callback_flag(self, msg):
        if msg.data == True:
            self.status = True

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
            

        </launch>
        """

        content = content1
        for node in self.nodes:
            content += node
        content += content2

        content = textwrap.dedent(content).strip()

        self.launch_description = content
        self.launch_name = self.experiment + f"_{self.name}" + ".launch"
        path2write = os.path.abspath(os.path.join(self.path2write, self.launch_name))
        self.write2cache(path2write, self.launch_description)

    #
    def generate_config_description(self, calibration_samples, max_lin_vel_ratio, max_ang_vel_ratio, error_tol_tx, error_tol_ty, error_tol_rz, occlusion_tol, k_ty, k_rz):
        content = textwrap.dedent(f"""
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
        """)
        name2write = os.path.abspath(os.path.join(self.path2write, "config.json"))
        self.config_description = content.strip()
        self.write2cache(name2write, self.config_description)

    #
    def write2cache(self, path, content):
        with open(path, "w") as file:
            file.write(content)


