
import rospy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Quaternion
from mavros_msgs.srv import CommandBool, CommandBoolRequest, CommandTOL, CommandTOLRequest, SetMode, SetModeRequest
from mess_msgs.msg import UAVState
from std_msgs.msg import Bool, Int16

import numpy as np

from mess_modules.agents import load_uavsecondary, load_calibration
from mess_modules.quaternions import multiply_quats, convert_quat2eul

class UAVSecondary():
    def __init__(self, uav_name):
        """
        Initialize UAVSecondary instance.

        Arguments
        ---------
        uav_name : str
            The name of the uav agent that the instance represents on the agent machine.
        """

        self.name = uav_name
        self.calibration_samples = load_uavsecondary(self.name)
        self.q_diff = load_calibration(self.name)

        # ROS states:
        self.x_global_curr = UAVState()
        self.offset = None

        # ROS topics:
        self.topic_to_primary = f"/{self.name}/messop/status"
        self.topic_from_primary = f"/{self.name}/messop/vertex"
        self.topic_occupancy = f"/{self.name}/occupancy"
        self.topic_vicon = f"/vicon/{self.name}/{self.name}"
        self.topic_vicon_calibrated = f"/vicon/{self.name}/{self.name}/calibrated"
        self.topic_vision_pose = f"/{self.name}/mavros/vision_pose/pose"

        # ROS publishers and subscribers:
        self.pub_status = rospy.Publisher(self.topic_to_primary, Bool, queue_size=10, latch=True)
        self.pub_vicon_calibrated = rospy.Publisher(self.topic_vicon_calibrated, TransformStamped, queue_size=10)
        self.pub_vision_pose = rospy.Publisher(self.topic_vision_pose, PoseStamped, queue_size=10)
        rospy.Subscriber(self.topic_occupancy, Int16, self.callback_occupancy)
        rospy.Subscriber(self.topic_vicon, TransformStamped, self.callback_vicon)

    def callback_occupancy(self, msg):
        """
        Stop the agent temporarily.

        Parameters
        ----------
        msg : Int16
            A message from the MESS Occupancy Grid ROS node.
        """

        pass

    def callback_vicon(self, msg):
        """
        Update the onboard states.

        The /vicon_bridge node publishes mm localization data to agents where a calibration is applied before the onboard state is updated.

        Parameters
        ----------
        msg : TransformStamped
            A message consisting of a translation and a rotation.
        """

        self.x_global_curr.translation.x = msg.transform.translation.x
        self.x_global_curr.translation.y = msg.transform.translation.y
        self.x_global_curr.translation.z = msg.transform.translation.z

        q_meas = Quaternion()
        q_meas.x = msg.transform.rotation.x
        q_meas.y = msg.transform.rotation.y
        q_meas.z = msg.transform.rotation.z
        q_meas.w = msg.transform.rotation.w
        q_true = multiply_quats(self.q_diff, q_meas)
        _, _, self.x_global_curr.pose.theta = convert_quat2eul(q_true)

        if self.offset is None:
            self.offset = Point()
            self.offset.x = -msg.transform.translation.x
            self.offset.y = -msg.transform.translation.y
            self.offset.z = -msg.transform.translation.z

        local = PoseStamped()
        local.header.frame_id = "map"
        local.pose.position.x = msg.transform.translation.x - self.offset.x
        local.pose.position.y = msg.transform.translation.y - self.offset.y
        local.pose.position.z = msg.transform.translation.z - self.offset.z
        local.pose.orientation.x = q_true.x
        local.pose.orientation.y = q_true.y
        local.pose.orientation.z = q_true.z
        local.pose.orientation.w = q_true.w
        self.pub_vision_pose.publish(local)

        ts_true = TransformStamped()
        ts_true.transform.translation.x = self.x_global_curr.pose.x
        ts_true.transform.translation.y = self.x_global_curr.pose.y
        ts_true.transform.translation.z = msg.transform.translation.z
        ts_true.transform.rotation.x = q_true.x
        ts_true.transform.rotation.y = q_true.y
        ts_true.transform.rotation.z = q_true.z
        ts_true.transform.rotation.w = q_true.w
        self.pub_vicon_calibrated.publish(ts_true)

    def set_mode(self, mode):
        """
        Sets the flight mode for the MAVROS module.

        This method uses the MAVROS service '/<self.name>/mavros/set_mode' to change
        the flight mode of the MAVROS module. It waits for the service to become available,
        constructs a `SetModeRequest` message with the provided mode, and sends it to the
        service using a `rospy.ServiceProxy`.

        Parameters
        ----------
        mode : str
            The desired flight mode to be set.

        Returns
        -------
        int
            Returns 1 upon successful service call to set the mode.
        """
        
        str1 = f"/{self.name}/mavros/set_mode"
        rospy.wait_for_service(str1, timeout=30)
        try:
            change_mode = SetModeRequest()
            change_mode.custom_mode = mode
            service_caller = rospy.ServiceProxy(str1, SetMode())
            service_caller(change_mode)
            return 1
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def arm(self, status):
        """
        Arms or disarms the MAVROS module.

        This method uses the MAVROS service '/<self.name>/mavros/cmd/arming' to arm or disarm the MAVROS module.
        It waits for the service to become available, constructs a `CommandBoolRequest` message with the provided status,
        and sends it to the service using a `rospy.ServiceProxy`.

        Parameters
        ----------
        status : bool
            True to arm the MAVROS module, False to disarm.

        Returns
        -------
        int
            Returns 1 upon successful arming or disarming.
        """

        str2 = f"/{self.name}/mavros/cmd/arming"
        rospy.wait_for_service(str2, timeout=30)
        try:
            cmd_arm = CommandBoolRequest()
            cmd_arm.value = status
            service_caller = rospy.ServiceProxy(str2, CommandBool)
            service_caller(cmd_arm)
            return 1
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def takeoff(self, altitude):

        str3 = f"/{self.name}/mavros/cmd/takeoff"
        rospy.wait_for_service(str3)
        try:
            cmd_takeoff = CommandTOLRequest
            takeoff_request = cmd_takeoff(altitude=altitude, latitude=0, longitude=0, min_pitch=0, yaw=0)
            service_caller = rospy.ServiceProxy(str3, CommandTOL)
            response = service_caller(takeoff_request)
            #
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
