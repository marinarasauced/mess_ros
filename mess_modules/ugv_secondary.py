    
import rospy
from geometry_msgs.msg import TransformStamped, Twist, Quaternion
from mess_msgs.msg import MESS2UGV, UGVState
from std_msgs.msg import Bool, Int16

import numpy as np

from mess_modules.agents import load_ugvsecondary, load_calibration
from mess_modules.log2terminal import print_task_agent
from mess_modules.quaternions import multiply_quats, convert_quat2eul


class UGVSecondary():
    def __init__(self, ugv_name):
        """
        Initialize UGVSecondary instance.

        Arguments
        ---------
        ugv_name : str
            The name of the ugv agent that the instance represents on the agent machine.
        """

        self.name = ugv_name
        self.calibration_samples, self.max_lin_vel, self.max_ang_vel, self.error_tol_tx, self.error_tol_ty, self.error_tol_rz, self.occlusion_tol, self.k_ty, self.k_rz = load_ugvsecondary(self.name)
        self.q_diff = load_calibration(self.name)

        # ROS states:
        self.e_local_curr = UGVState()
        self.x_global_curr = UGVState()
        self.x_vertex_init = UGVState()
        self.x_vertex_trgt = UGVState()

        # ROS topics: 
        self.topic_control = f"/{self.name}/cmd_vel"
        self.topic_to_primary = f"/{self.name}/messop/status"
        self.topic_from_primary = f"/{self.name}/messop/vertex"
        self.topic_occupancy = f"/{self.name}/occupancy"
        self.topic_vicon = f"/vicon/{self.name}/{self.name}"
        self.topic_vicon_calibrated = f"/vicon/{self.name}/{self.name}/calibrated"

        # ROS publishers and subscribers:
        self.pub_control = rospy.Publisher(self.topic_control, Twist, queue_size=10)
        self.pub_status = rospy.Publisher(self.topic_to_primary, Bool, queue_size=10, latch=True)
        self.pub_vicon_calibrated = rospy.Publisher(self.topic_vicon_calibrated, TransformStamped, queue_size=10)
        rospy.Subscriber(self.topic_occupancy, Int16, self.callback_occupancy)
        rospy.Subscriber(self.topic_vicon, TransformStamped, self.callback_vicon)

    def callback_occupancy(self, msg):
        """
        Stop the agent temporarily.

        An occupancy grid tracks the position of ground-based agents in the laboratory environment. If two agents are expected to collide during linear translations, the agent with lower priority will sleep a number of seconds specified in the msg.

        Parameters
        ----------
        msg : Int16
            A message from the MESS Occupancy Grid ROS node.
        """

        rospy.sleep(msg.data)

    def callback_vicon(self, msg):
        """
        Update the onboard states.

        The /vicon_bridge node publishes mm localization data to agents where a calibration is applied before the onboard state is updated.

        Parameters
        ----------
        msg : TransformStamped
            A message consisting of a translation and a rotation.
        """

        self.x_global_curr.pose.x = msg.transform.translation.x
        self.x_global_curr.pose.y = msg.transform.translation.y

        q_meas = Quaternion()
        q_meas.x = msg.transform.rotation.x
        q_meas.y = msg.transform.rotation.y
        q_meas.z = msg.transform.rotation.z
        q_meas.w = msg.transform.rotation.w
        q_true = multiply_quats(self.q_diff, q_meas)
        _, _, self.x_global_curr.pose.theta = convert_quat2eul(q_true)

        ts_true = TransformStamped()
        ts_true.transform.translation.x = self.x_global_curr.pose.x
        ts_true.transform.translation.y = self.x_global_curr.pose.y
        ts_true.transform.translation.z = msg.transform.translation.z
        ts_true.transform.rotation.x = q_true.x
        ts_true.transform.rotation.y = q_true.y
        ts_true.transform.rotation.z = q_true.z
        ts_true.transform.rotation.w = q_true.w
        self.pub_vicon_calibrated.publish(ts_true)

    def controlUGV(self, u_lin, u_ang):
        """
        Publish a control input to the agent.

        TurtleBot3 agents take linear and angular control input in the form of a Twist ROS message instance. The vehicles are configured so that on bringup, the topic are remapped to /{self.name}/cmd_vel.

        Parameters
        ----------
        u_lin : float
            The linear velocity control input.
        u_ang : float
            The angular velocity control input.
        """

        twist = Twist()
        twist.linear.x = u_lin
        twist.angular.z = u_ang
        if abs(twist.linear.x) > self.max_lin_vel:
            twist.linear.x = np.sign(u_lin) * self.max_lin_vel
        if abs(twist.angular.z) > self.max_ang_vel:
            twist.angular.z = np.sign(u_ang) * self.max_ang_vel
        self.pub_control.publish(twist)

    def rotateUGV(self):
        """
        Rotate the ugv agent to self.x_vertex_trgt.pose.theta.

        When this function is called, control input proportional to the rotational error are published until the agent's orientation is within tolerance.
        """

        rate = rospy.Rate(20)
        self.e_reset()
        print_task_agent(f"rotating to {self.x_vertex_trgt.pose.theta}", self.name)
        while abs(self.e_local_curr.pose.theta) > self.error_tol_rz:
            self.e_update1()
            u_ang = -self.max_ang_vel * self.e_local_curr.pose.theta
            self.controlUGV(u_lin=0.0, u_ang=u_ang)
            rate.sleep()
        self.controlUGV(u_lin=0.0, u_ang=0.0)

    def translateUGV(self):
        """
        Translate the ugv agent to self.x_vertex_trgt.pose.x and self.x_vertex_trgt.pose.y.

        When this function is called, control input proportional to the local y-pos and z-rot error are published to the agent so that it follows the vector from the starting position to the target position.
        """

        rate = rospy.Rate(20)
        self.e_reset()
        print_task_agent(f"translating to ({self.x_vertex_trgt.pose.x}, {self.x_vertex_trgt.pose.y})", self.name)
        while abs(self.e_local_curr.pose.x) > self.error_tol_tx or abs(self.e_local_curr.pose.y) > self.error_tol_ty:
            self.e_update2()
            u_ang = -self.k_ty * self.e_local_curr.pose.y -self.k_rz * self.e_local_curr.pose.theta
            self.controlUGV(u_lin=self.max_lin_vel, u_ang=u_ang)
            rate.sleep()
        self.controlUGV(u_lin=0.0, u_ang=0.0)

    def transitionUGV(self):
        """
        """

        status = Bool()
        status.data = True
        self.pub_status.publish(status)
        print("waiting for somethign to happen")

        rospy.wait_for_message(self.topic_vicon, TransformStamped)
        self.x_vertex_init.pose.x = self.x_global_curr.pose.x
        self.x_vertex_init.pose.y = self.x_global_curr.pose.y
        self.x_vertex_init.pose.theta = self.x_global_curr.pose.theta
        self.x_vertex_trgt.pose.x = self.x_vertex_init.pose.x
        self.x_vertex_trgt.pose.y = self.x_vertex_init.pose.y
        self.x_vertex_trgt.pose.theta = self.x_vertex_init.pose.theta
        while not rospy.is_shutdown():
            print_task_agent("waiting for vertex", self.name)
            while self.x_vertex_init.pose.x == self.x_vertex_trgt.pose.x and self.x_vertex_init.pose.y == self.x_vertex_trgt.pose.y and self.x_vertex_init.pose.theta == self.x_vertex_trgt.pose.theta:
                vertex = rospy.wait_for_message(self.topic_from_primary, MESS2UGV)
                self.x_vertex_trgt.pose.x = vertex.pose.x
                self.x_vertex_trgt.pose.y = vertex.pose.y
                self.x_vertex_trgt.pose.theta = self.wrap2pi(vertex.pose.theta)
            operation = vertex.index

            if operation == 1:
                self.rotateUGV()

            elif operation == 2:
                dx = self.x_vertex_trgt.pose.x - self.x_vertex_init.pose.x
                dy = self.x_vertex_trgt.pose.y - self.x_vertex_init.pose.y
                theta0 = np.arctan2(dy, dx)
                self.x_vertex_trgt.pose.theta = theta0
                self.rotateUGV()
                self.translateUGV()

            elif operation == 3:
                dx = self.x_vertex_trgt.pose.x - self.x_vertex_init.pose.x
                dy = self.x_vertex_trgt.pose.y - self.x_vertex_init.pose.y
                theta0 = np.arctan2(dy, dx)
                theta1 = self.x_vertex_trgt.pose.theta
                self.x_vertex_trgt.pose.theta = theta0
                self.rotateUGV()
                self.translateUGV()
                self.x_vertex_trgt.pose.theta = theta1
                self.rotateUGV()

            else:
                pass

            self.controlUGV(u_lin=0.0, u_ang=0.0)
            self.x_vertex_init.pose.x = self.x_global_curr.pose.x
            self.x_vertex_init.pose.y = self.x_global_curr.pose.y
            self.x_vertex_init.pose.theta = self.x_global_curr.pose.theta
            self.x_vertex_trgt.pose.x = self.x_vertex_init.pose.x
            self.x_vertex_trgt.pose.y = self.x_vertex_init.pose.y
            self.x_vertex_trgt.pose.theta = self.x_vertex_init.pose.theta

            status = Bool()
            status.data = True
            self.pub_status.publish(status)

    def e_reset(self):
        """
        Reset local error state to infinity before transition.
        """

        self.e_local_curr.pose.x = np.inf
        self.e_local_curr.pose.y = np.inf
        self.e_local_curr.pose.theta = np.inf

    def e_update1(self):
        """
        Update local error during rotation.
        """

        self.e_local_curr.pose.x = self.x_vertex_trgt.pose.x - self.x_global_curr.pose.x
        self.e_local_curr.pose.y = self.x_vertex_trgt.pose.y - self.x_global_curr.pose.y
        self.e_local_curr.pose.theta = self.wrap2pi(self.x_global_curr.pose.theta -self.x_vertex_trgt.pose.theta)

    def e_update2(self):
        """
        Update local error during translation.
        """

        try:
            A = np.array([[self.x_vertex_trgt.pose.x - self.x_vertex_init.pose.x], [self.x_vertex_trgt.pose.y - self.x_vertex_init.pose.y]])
            B = np.array([[self.x_vertex_trgt.pose.x - self.x_global_curr.pose.x], [self.x_vertex_trgt.pose.y - self.x_global_curr.pose.y]])
            C = np.array([[self.x_global_curr.pose.x - self.x_vertex_init.pose.x], [self.x_global_curr.pose.y - self.x_vertex_init.pose.y]])
            a = np.sqrt(A[0, 0] ** 2 + A[1, 0] ** 2)
            b = np.sqrt(B[0, 0] ** 2 + B[1, 0] ** 2)

            theta = np.arccos(sum(np.multiply(A, B)) / (a * b))
            alpha = np.arctan2(C[1, 0], C[0, 0])
            psi = np.arctan2(A[1, 0], A[0, 0])

            self.e_local_curr.pose.x = b * np.cos(theta)
            self.e_local_curr.pose.y = b * np.sin(theta) * np.sign(alpha - psi)
            self.e_local_curr.pose.theta = self.wrap2pi(self.x_global_curr.pose.theta - psi)
        except:
            print("error")

    def wrap2pi(self, theta):
        """
        Wrap a heading to [-pi, pi].

        Parameters
        ----------
        theta : float
            The heading of the agent in the threat plane.

        Returns
        -------
        float
            The theta parameter wrapped to [-pi, pi].
        """

        while np.abs(theta) > np.pi:
            theta -= np.sign(theta) * (2 * np.pi)
        return theta
