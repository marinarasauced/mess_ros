
import rospy
from geometry_msgs.msg import TransformStamped, Quaternion
from mess_msgs.msg import StateUAV

import numpy as np

class UAV():
    def __init__(self, name, Qx, Qy, Qz, Qw):

        # Input paramters:
        self.name = name


        self.q_diff = Quaternion()
        self.q_diff.x = Qx
        self.q_diff.y = Qy
        self.q_diff.z = Qz
        self.q_diff.w = Qw

        # ROS states:
        self.x_global_curr = StateUAV()  # current global state

        # ROS topics:
        self.topic_vicon = "/vicon/" + self.name + "/" + self.name

        # ROS publishers:

        # ROS subscribers:
        rospy.Subscriber(self.topic_vicon, TransformStamped, queue_size=10)



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

        # Publish calibrated pose to logger:
        ts_true = TransformStamped()
        ts_true.transform.translation.x = self.x_global_curr.Tx
        ts_true.transform.translation.y = self.x_global_curr.Ty
        ts_true.transform.translation.z = msg.transform.translation.z
        ts_true.transform.rotation.x = q_true.x
        ts_true.transform.rotation.y = q_true.y
        ts_true.transform.rotation.z = q_true.z
        ts_true.transform.rotation.w = q_true.w

        self.v_.publish(ts_true)
