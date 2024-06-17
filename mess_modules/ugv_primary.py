
import rospy
from mess_msgs.msg import MESS2UGV
from std_msgs.msg import Bool

import os.path
import textwrap

from mess_modules.agents import load_ugvprimary
from mess_modules.paths import get_path2agent, get_path2cache, write_file2path

class UGVPrimary():
    def __init__(self, ugv_name, experiment_name):
        """
        Initialize UGVPrimary instance.

        Arguments
        ---------
        ugv_name : str
            The name of the ugv agent that the instance represents on the local machine running the experiment.
        experiment_name : str
            The name of the current experiment.
        """

        self.name = ugv_name
        self.experiment = experiment_name
        self.ip, self.username, self.password, self.tb3_model, self.lds_model, self.priority = load_ugvprimary(self.name)
        self.status = False  # The agent's readiness to perform a task.

        # ROS nodes:
        self.node_strings = []
        self.node_names = []
        self.node_names.append(f"{self.name}_turtlebot3_diagnostics")
        self.node_names.append(f"{self.name}_logger")

        # ROS topics: 
        self.topic_to_secondary = f"/{self.name}/messop/vertex"  
        self.topic_from_secondary = f"/{self.name}/messop/status"

        # ROS publishers and subscribers:
        self.pub_vertex = rospy.Publisher(self.topic_to_secondary, MESS2UGV, queue_size=10)
        rospy.Subscriber(self.topic_from_secondary, Bool, self.callback_status)
 
    def add_node(self, node):
        """
        Add a node to the agent for the experiment.

        Parameters
        ----------
        node : ROSNode instance
            The ROSNode instance which is to be added to the agent for the experiment.
        """

        self.node_names.append(f"{self.name}_{node.node_name}")
        self.node_strings.append(node.node_string)
    
    def callback_status(self, msg):
        """
        Upade the agent's status from a ROS1 message of type Bool.

        Parameters
        ----------
        msg : Bool
            A message from the corresponding UGVSecondary instance.
        """

        self.status = msg.data

    def write_config_file(self, calibration_samples, max_lin_vel_ratio, max_ang_vel_ratio, error_tol_tx, error_tol_ty, error_tol_rz, occlusion_tol, k_ty, k_rz):
        """
        Write agent-specific parameters to a JSON file.

        Parameters
        ----------
        calibration_samples : int
            The number of VICON localization samples to collect at each calibration position.
        max_lin_vel_ratio : float
            The agent's maximum allowable linear velocity during the experiment is calculated by multiplying this ratio by the hardware-specified maximum allowable linear velocity.
        max_ang_vel_ratio : float
            The agent's maximum allowable angular velocity during the experiment is calculated by multiplying this ratio by the hardware-specified maximum allowable angular velocity.
        error_tol_tx : float
            The error tolerance parallel to the agent's direction of motion during a linear translation. Units are meters.
        error_tol_ty : float
            The error tolerance perpendicular to the agent's direction of motion during a linear translation. Units are meters.
        error_tol_rz : float
            The error tolerance during a rotation. Units are radians.
        occlusion_tol : float
            The number of seconds the agent can rely on VICON localization after being occluded from the VICON cameras before switching to an onboard state estimate.
        k_ty : float
            The state-feedback controller gain for local y-pos error.
        k_rz : float
            The state-feedback controller gain for local z-rot error.
        """

        file_content = textwrap.dedent(f"""
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
        """).strip()

        file_path = os.path.join(get_path2agent(self.name), f"config.json")
        write_file2path(path=file_path, content=file_content)

    def write_launch_file(self):
        """
        Write an agent-specific launch file for the experiment.

        This function has no arguments; however, it iterates through the user-added nodes in the experiments ros node for the agent and adds them to a custom launch file that is later pushed to the agent.
        """

        upper = f"""
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
        lower = f"""
            <node pkg="mess_logger" type="mess_logger.py" name="$(arg ugv_name)_logger" output="screen"/>

        </launch>
        """
        file_content = upper
        for node_string in self.node_strings:
            file_content += node_string
        file_content += lower
        file_content = textwrap.dedent(file_content).strip()

        self.launch = f"{self.experiment}_{self.name}.launch"
        file_path = os.path.join(get_path2cache(), self.launch)
        write_file2path(path=file_path, content=file_content)






