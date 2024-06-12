
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from mess_msgs.msg import StateUAV

from quaternions import multiply_quats

class UAV():
    def __init__(self, name, Qx, Qy, Qz, Qw):

        # Input paramters:
        self.name = name

        self.q_diff = Quaternion()
        self.q_diff.x = Qx
        self.q_diff.y = Qy
        self.q_diff.z = Qz
        self.q_diff.w = Qw

        self.vicon_offset_Tx = None
        self.vicon_offset_Ty = None
        self.vicon_offset_Tz = None
        self.vicon_offset_check = False

        # ROS states:
        self.x_global_curr = StateUAV()  # current global state

        # ROS topics:
        self.topic_vicon = "/vicon/" + self.name + "/" + self.name
        self.topic_mavros_pose = "/" + self.name + "/mavros/vision_pose/pose"

        # ROS publishers:
        self.mavros_vp_ = rospy.Publisher(self.topic_mavros_pose, PoseStamped, queue_size=10)

        # ROS subscribers:
        rospy.Subscriber(self.topic_vicon, TransformStamped, self.callback_vicon)



    # Update current global state with VICON callback.
    def callback_vicon(self, msg):

        # Update position:
        self.x_global_curr.Tx = msg.transform.translation.x
        self.x_global_curr.Ty = msg.transform.translation.y
        self.x_global_curr.Tz = msg.transform.translation.z

        # Correct measured quaternion orientation for calibrated orientation of tracked object in VICON tracker:
        q_meas = Quaternion()
        q_meas.x = msg.transform.rotation.x
        q_meas.y = msg.transform.rotation.y
        q_meas.z = msg.transform.rotation.z
        q_meas.w = msg.transform.rotation.w

        q_true = multiply_quats(self.q_diff, q_meas)

        # Update orientation:
        self.x_global_curr.Qx = q_true.x
        self.x_global_curr.Qy = q_true.y
        self.x_global_curr.Qz = q_true.z
        self.x_global_curr.Qw = q_true.w

        # Set VICON origin if not already set:
        if not self.vicon_offset_check:
            self.set_vicon_origin(self.x_global_curr)
            self.vicon_offset_check = True

        x_local_curr = PoseStamped()
        x_local_curr.header.stamp = rospy.Time.now()
        x_local_curr.header.frame_id = "map"  # Indicates global coordinates

        # Set local position for mavros vision pose:
        x_local_curr.transform.translation.x = self.x_global_curr.Tx - self.vicon_offset_Tx
        x_local_curr.transform.translation.y = self.x_global_curr.Ty - self.vicon_offset_Ty
        x_local_curr.transform.translation.z = self.x_global_curr.Tz - self.vicon_offset_Tz

        # Set local orientation for mavros vision pose:
        x_local_curr.transform.orentation.x = self.x_global_curr.Qx
        x_local_curr.transform.orentation.y = self.x_global_curr.Qy
        x_local_curr.transform.orentation.z = self.x_global_curr.Qz
        x_local_curr.transform.orentation.w = self.x_global_curr.Qw

        #
        self.mavros_vp_.publish(x_local_curr)

    # Store initial position in vicon frame for local transformation:
    #   Input:  global state at first callback
    def set_vicon_origin(self, state):
        self.vicon_offset_Tx = state.Tx
        self.vicon_offset_Ty = state.Ty
        self.vicon_offset_Tz = state.Tz




        

        




