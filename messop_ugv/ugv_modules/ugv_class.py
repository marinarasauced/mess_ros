
import rospy
from geometry_msgs.msg import TransformStamped, Twist, Quaternion
from mess_msgs.msg import MESS2UGV, StateUGV
from std_msgs.msg import Bool

import numpy as np
from quaternions import multiply_quats, convert_quat2eul

class UGV():
    def __init__(self, name, max_lin_vel, max_lin_vel_ratio, max_ang_vel, max_ang_vel_ratio, error_tol_Tx, error_tol_Ty, error_tol_Rz, occlusion_tol, K_Ty, K_Rz, Qx, Qy, Qz, Qw):

        # Input parameters:
        self.name = name
        self.max_lin_vel = max_lin_vel * max_lin_vel_ratio
        self.max_ang_vel = max_ang_vel * max_ang_vel_ratio
        self.error_tol_Tx = error_tol_Tx
        self.error_tol_Ty = error_tol_Ty
        self.error_tol_Rz = error_tol_Rz
        self.occlusion_tol = occlusion_tol
        self.K_Ty = K_Ty
        self.K_Rz = K_Rz

        self.q_diff = Quaternion()
        self.q_diff.x = Qx
        self.q_diff.y = Qy
        self.q_diff.z = Qz
        self.q_diff.w = Qw

        # ROS states:
        self.e_local_curr = StateUGV()   # current local error
        self.x_global_curr = StateUGV()  # current global state
        self.x_vertex_init = StateUGV()  # vertex before transition
        self.x_vertex_trgt = StateUGV()  # vertex after transition

        # ROS topics:
        self.topic_control_input = "/" + self.name + "/cmd_vel"
        self.topic_messop_flag = "/" + self.name + "/messop/messop/flag"
        self.topic_messop_vertex = "/" + self.name + "/messop/messop/vertex"
        self.topic_vicon = "/vicon/" + self.name + "/" + self.name

        # ROS publishers:
        self.u_ = rospy.Publisher(self.topic_control_input, Twist, queue_size=10)        
        self.flag_ = rospy.Publisher(self.topic_messop_flag, Bool, queue_size=10)

        # ROS subsribers:
        rospy.Subscriber(self.topic_vicon, TransformStamped, self.callback_vicon)



    # Update current global state with VICON callback.
    def callback_vicon(self, msg):

        # Update position:
        self.x_global_curr.Tx = msg.transform.translation.x

        # Correct measured quaternion orientation for calibrated orientation of tracked object in VICON tracker:
        q_meas = Quaternion()
        q_meas.x = msg.transform.rotation.x
        q_meas.y = msg.transform.rotation.y
        q_meas.z = msg.transform.rotation.z
        q_meas.w = msg.transform.rotation.w

        q_true = multiply_quats(self.q_diff, q_meas)
        Rx, Ry, Rz = convert_quat2eul(q_true)

        # Update orientation:
        self.x_global_curr.Rz = Rz


    
    # Publish control input to UGV.
    #   Input:  linear and angular velocities
    def controlUGV(self, u_lin, u_ang):

        # Initialize control input message:
        twist = Twist()
        twist.linear.x = u_lin
        twist.angular.z = u_ang

        # Wrap linear velocity to acceptable range:
        if abs(twist.linear.x) > self.max_lin_vel:
            twist.linear.x = np.sign(u_lin) * self.max_lin_vel

        # Wrap angular velocity to acceptable range:
        if abs(twist.angular.z) > self.max_ang_vel:
            twist.angular.z = np.sign(u_ang) * self.max_ang_vel

        #
        self.u_.publish(twist)



    # Rotate to a desired orientation.
    #   Note:   desired orientation is dependent on the float and operation in the received MESS2UGV message.
    def rotateUGV(self): 

        # Indiciate rotation:
        rospy.loginfo("".join(["Rotating towards: (",str(self.x_vertex_trgt.Rz),")"]))

        # Rotate to heading at a varying rate proportional to the current error:
        self.e_reset()
        while abs(self.e_local_curr.Rz) > self.error_tol_Rz:
            
            # Update local error:
            self.e_update1()

            # Calculate and publish control input:
            u_lin = 0.0
            u_ang = -self.max_ang_vel * self.e_local_curr.Rz
            self.u_.publish(u_lin, u_ang)



    # Translate to a desired position.
    #   Note:   desired position is dependent on the float and operation in the received MESS2UGV message.
    def translateUGV(self):

        # Indicate translation:
        rospy.loginfo("".join(["Translating to: (",str(self.x_vertex_trgt.Tx),",",str(self.x_vertex_trgt.Ty),")"]))

        #
        self.e_reset()
        while abs(self.e_local_curr.Tx) > self.error_tol_Tx or abs(self.e_local_curr.Ty) > self.error_tol_Ty:

            # Update local error:
            self.e_update2()

            # Calculate and publish control input:
            u_lin = self.max_lin_vel
            u_ang = -self.K_Ty * self.e_local_curr.Ty -self.K_Rz * self.e_local_curr.Rz
            self.u_.publish(u_lin, u_ang)



    # Transition between vertices.
    #    Note:  this loop will run until the node is shutdown.
    def transitionUGV(self):
        while not rospy.is_shutdown():
            rospy.loginfo("Waiting for new vertex ...")

            # Wait for new vertex to be published:
            while self.x_vertex_init.Tx == self.x_vertex_trgt.Tx and self.x_vertex_init.Ty == self.x_vertex_trgt.Ty and self.x_vertex_init.Rz == self.x_vertex_trgt.Rz:
                vertex = rospy.wait_for_message(self.topic_messop_ugv, MESS2UGV)
                self.x_vertex_trgt.Tx = vertex.Tx
                self.x_vertex_trgt.Ty = vertex.Ty
                self.x_vertex_trgt.Rz = self.wrap_heading(vertex.Rz)

            # Retrieve operation index:
            op = vertex.index

            # Operation one (rotation):
            if op == 1:

                # Rotate to orientation from vertex message:
                self.rotateUGV()

            # Operation two (rotation -> translation):
            elif op == 2:

                # Calculate heading to position from vertex message:
                dx = self.x_vertex_trgt.Tx - self.x_global_curr.Tx
                dy = self.x_vertex_trgt.Ty - self.x_global_curr.Ty
                Rz_before_translation = np.arctan2(dy, dx)

                # Rotate and translation to position from vertex message:
                self.x_vertex_trgt.Rz = Rz_before_translation
                self.rotateUGV()
                self.translateUGV()

            # Operation three (rotation -> translation -> rotation)
            elif op == 3:

                # Calculate heading to position from vertex message:
                dx = self.x_vertex_trgt.Tx - self.x_global_curr.Tx
                dy = self.x_vertex_trgt.Ty - self.x_global_curr.Ty
                Rz_before_translation = np.arctan2(dy, dx)
                Rz_after_translation = self.x_vertex_trgt.Rz

                # Rotate and translation to position from vertex message, then rotate to orientation from vertex message:
                self.x_vertex_trgt.Rz = Rz_before_translation
                self.rotateUGV()
                self.translateUGV()
                self.x_vertex_trgt.Rz = Rz_after_translation
                self.rotateUGV()

            else:
                rospy.loginfo("Invalid operation index.")

            # Prepare for next transition:
            u_lin = 0.0
            u_ang = 0.0
            self.u_.publish(u_lin, u_ang)

            self.x_vertex_init.Tx = vertex.Tx
            self.x_vertex_init.Ty = vertex.Ty 
            self.x_vertex_init.Rz = vertex.Rz

            # Publish flag to MESS indiciating current transition is complete:
            flag2MESS = Bool()
            flag2MESS.data = True
            self.flag_.publish(flag2MESS)      



    # Reset local error to infinity before transitions.
    def e_reset(self):
        self.e_local_curr.Tx = np.inf
        self.e_local_curr.Ty = np.inf
        self.e_local_curr.Rz = np.inf

    # Update local error during rotation loops.
    def e_update1(self):

        self.e_local_curr.Tx = self.x_vertex_trgt.Tx - self.x_global_curr.Tx
        self.e_local_curr.Ty = self.x_vertex_trgt.Ty - self.x_global_curr.Ty
        self.e_local_curr.Rz = self.wrap_heading(self.x_vertex_trgt.Rz - self.x_global_curr.Rz)

    # Update local error during translation loops.
    def e_update2(self):

        A = np.array([[self.x_vertex_trgt.Tx - self.x_vertex_init.Tx], [self.x_vertex_trgt.Ty - self.x_vertex_init.Ty]])
        B = np.array([[self.x_vertex_trgt.Tx - self.x_global_curr.Tx], [self.x_vertex_trgt.Ty - self.x_global_curr.Ty]])
        C = np.array([[self.x_global_curr.Tx - self.x_vertex_init.Tx], [self.x_global_curr.Ty - self.x_vertex_init.Ty]])

        a = np.sqrt(A[0, 0] ** 2 + A[1, 0] ** 2)
        b = np.sqrt(B[0, 0] ** 2 + B[1, 0] ** 2)

        theta = np.arccos(sum(np.multiply(A, B)) / (a * b))
        alpha = np.arctan2(C[1, 0], C[0, 0])
        psi = np.arctan2(A[1, 0], A[0, 0])

        self.e_local_curr.Tx = b * np.cos(theta)
        self.e_local_curr.Ty = b * np.sin(theta) * np.sign(alpha - psi)
        self.e_local_curr.Rz = self.wrap_heading(self.x_global_curr.Rz - psi)

    # Wrap heading to radian domain [-pi, pi].
    #   Input:  heading angle in radians
    #   Output: heading angle in radians wrapped to [-pi, pi]
    def wrap_heading(Rz):
        while np.abs(Rz) > np.pi:
            Rz -= np.sign(Rz) * (2 * np.pi)
        return Rz