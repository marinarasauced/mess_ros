
from geometry_msgs.msg import Quaternion

import numpy as np

def multiply_quats(q_1, q_2):
    """
    Multiply two Quaternion ROS message instances.

    Parameters
    ----------
    q_1 : Quaternion ROS message instance
        Difference quaternion used to calibrate measured pose to true pose.
    q_2 : Quaternion ROS message instance
        Measured quaternion from VICON mm localization.

    Returns
    -------
    q_3 : Quaternion ROS message instance
        True quaternion from measured and difference quaterions.
    """

    q_3 = Quaternion()
    q_3.x = (q_1.w * q_2.x) + (q_1.x * q_2.w) + (q_1.y * q_2.z) - (q_1.z * q_2.y)
    q_3.y = (q_1.w * q_2.y) - (q_1.x * q_2.z) + (q_1.y * q_2.w) + (q_1.z * q_2.x)
    q_3.z = (q_1.w * q_2.z) + (q_1.x * q_2.y) - (q_1.y * q_2.x) + (q_1.z * q_2.w)
    q_3.w = (q_1.w * q_2.w) - (q_1.x * q_2.x) - (q_1.y * q_2.y) - (q_1.z * q_2.z)
    return q_3

def convert_quat2eul(q_1):
    """
    Convert a Quaternion ROS message instance to Euler angles.

    Parameters
    ----------
    q_1 : Quaternion ROS message instance
        Quaternion orientation of an agent.

    Returns
    -------
    roll : float
        Roll of the agent.
    pitch : float
        Pitch of the agent.
    yaw : float
        Yaw of the agent.
    """

    roll = roll = np.arctan2(2 * (q_1.w * q_1.x + q_1.y * q_1.z), 1 - 2 * (q_1.x ** 2 + q_1.y ** 2))
    pitch_sign = 2 * (q_1.w * q_1.y - q_1.x * q_1.z)
    if np.abs(pitch_sign) >= 1:
        pitch = np.sign(pitch_sign) * 0.5 * np.pi
    else:
        pitch = -0.5 * np.pi + np.arcsin(pitch_sign)
    yaw = np.arctan2(2 * (q_1.w * q_1.z + q_1.x * q_1.y), 1 - 2 * (q_1.y ** 2 + q_1.z ** 2))
    return roll, pitch, yaw

def convert_eul2quat(roll, pitch, yaw):
    """
    Convert Euler angles to a Quaternion ROS message instance.

    Parameters
    ----------
    roll : float
        Roll of the agent.
    pitch : float
        Pitch of the agent.
    yaw : float
        Yaw of the agent.

    Returns
    -------
    q_1 : Quaternion ROS message instance
        Quaternion orientation of an agent.
    """

    cx = np.cos(0.5 * roll)
    sx = np.sin(0.5 * roll)
    cy = np.cos(0.5 * pitch)
    sy = np.sin(0.5 * pitch)
    cz = np.cos(0.5 * yaw)
    sz = np.sin(0.5 * yaw)
    q_1 = Quaternion()
    q_1.x = sx * cy * cz - cx * sy * sz
    q_1.y = cx * sy * cz + sx * cy * sz
    q_1.z = cx * cy * sz - sx * sy * cz
    q_1.w = cx * cy * cz + sx * sy * sz
    return q_1

def invert_quat(q_1):
    """
    Invert a Quaternion ROS message instance.

    Parameters
    ----------
    q_1 : Quaternion ROS message instance
        Quaternion orientation of an agent.

    Returns
    -------
    q_2 : Quaternion ROS message instance
        Inverse of q_1.
    """

    q_2 = Quaternion()
    q_2.x = -q_1.x
    q_2.y = -q_1.y
    q_2.z = -q_1.z
    q_2.w =  q_1.w
    return q_2

def normalize_quat(q_1):
    """
    Normalize a Quaternion ROS message instance.

    Parameters
    ----------
    q_1 : Quaternion ROS message instance
        The input quaternion to be normalized.

    Returns
    q_2 : Quaternion ROS message instance
        The normalized input quaternion.
    """

    magnitude = (q_1.x ** 2 + q_1.y ** 2 + q_1.z ** 2 + q_1.w ** 2) ** 0.5
    q_2 = Quaternion()
    q_2.x = q_1.x / magnitude
    q_2.y = q_1.y / magnitude
    q_2.z = q_1.z / magnitude
    q_2.w = q_1.w / magnitude
    return q_2

def average_quats(q_1, q_2):
    """
    Average two Quaternion ROS message instances using spherical linear interpolation (slerp).

    Parameters
    ----------
    q_1 : Quaternion ROS message instance
        The first quaternion.
    q_2 : Quaternion ROS message instance
        The second quaternion.

    Returns
    -------
    q_3 : Quaternion ROS message instance
        The average quaternion of q_1 and q_2.
    """

    q_1n = normalize_quat(q_1)
    q_2n = normalize_quat(q_2)
    dot = q_1n.x * q_2n.x + q_1n.y * q_2n.y + q_1n.z * q_2n.z + q_1n.w * q_2n.w
    if dot < 0:
        q_2n.x *= -1
        q_2n.y *= -1
        q_2n.z *= -1
        q_2n.w *= -1
    t = 0.5
    q_3 = Quaternion()
    q_3.x = (1 - t) * q_1n.x + t * q_2n.x
    q_3.y = (1 - t) * q_1n.y + t * q_2n.y
    q_3.z = (1 - t) * q_1n.z + t * q_2n.z
    q_3.w = (1 - t) * q_1n.w + t * q_2n.w
    q_3 = normalize_quat(q_3)
    return q_3
