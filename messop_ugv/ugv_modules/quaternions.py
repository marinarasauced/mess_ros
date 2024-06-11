
from geometry_msgs.msg import Quaternion

import numpy as np

##################################################################################

# Convertes euler angles to equivalent quaternion.
#   Input:  3x euler angle floats in radians
#   Output: 1x ROS1 quaternion message
def convert_eul2quat(Rx, Ry, Rz):

    cx = np.cos(0.5 * Rx)
    sx = np.sin(0.5 * Rx)
    cy = np.cos(0.5 * Ry)
    sy = np.sin(0.5 * Ry)
    cz = np.cos(0.5 * Rz)
    sz = np.sin(0.5 * Rz)

    q_out1 = Quaternion()
    q_out1.x = sx * cy * cz - cx * sy * sz
    q_out1.y = cx * sy * cz + sx * cy * sz
    q_out1.z = cx * cy * sz - sx * sy * cz
    q_out1.w = cx * cy * cz + sx * sy * sz

    return q_out1

# Converts quaternion to equivalent euler angles.
#   Input:  1x ROS1 quaternion message
#   Output: 3x euler angle floats in radians
def convert_quat2eul(q_in1):

    Rx = np.arctan2(2 * (q_in1.w * q_in1.x + q_in1.y * q_in1.z), 1 - 2 * (q_in1.x ** 2 + q_in1.y ** 2))
    Ry = -0.5 * np.pi + 2 * np.arctan2(np.sqrt(1 + 2 * (q_in1.w * q_in1.y - q_in1.x * q_in1.z)), np.sqrt(1 - 2 * (q_in1.w * q_in1.y - q_in1.x * q_in1.z)))
    Rz = np.arctan2(2 * (q_in1.w * q_in1.z + q_in1.x * q_in1.y), 1 - 2 * (q_in1.y ** 2 + q_in1.z ** 2))

    return Rx, Ry, Rz

# Calculates the product of two quaternions.
#   Input:  2x ROS1 quaternion messages
#   Output: 1x ROS1 quaternion message
def multiply_quats(q_in1, q_in2):

    q_out1 = Quaternion()
    q_out1.x = (q_in1.w * q_in2.x) + (q_in1.x * q_in2.w) + (q_in1.y * q_in2.z) - (q_in1.z * q_in2.y)
    q_out1.y = (q_in1.w * q_in2.y) - (q_in1.x * q_in2.z) + (q_in1.y * q_in2.w) + (q_in1.z * q_in2.x)
    q_out1.z = (q_in1.w * q_in2.z) + (q_in1.x * q_in2.y) - (q_in1.y * q_in2.x) + (q_in1.z * q_in2.w)
    q_out1.w = (q_in1.w * q_in2.w) - (q_in1.x * q_in2.x) - (q_in1.y * q_in2.y) - (q_in1.z * q_in2.z)
    return q_out1

# Normalizes a quaternion so that its magnitude equals one.
#   Input:  1x ROS1 quaternion message
#   Output: 1x ROS1 quaternion message
def normalize_quat(q_in1):

    q_in1_norm = np.sqrt(q_in1.x ** 2 + q_in1.y ** 2 + q_in1.z ** 2 + q_in1.w ** 2)
    
    q_out1 = Quaternion()
    q_out1.x = q_in1.x / q_in1_norm
    q_out1.y = q_in1.y / q_in1_norm
    q_out1.z = q_in1.z / q_in1_norm
    q_out1.w = q_in1.w / q_in1_norm
    return q_out1

# Returns the inverse of a quaternion.
#   Input:  1x ROS1 quaternion message
#   Output: 1x ROS1 quaternion message
def invert_quat(q_in1):

    q_out1 = Quaternion()
    q_out1.x = -q_in1.x
    q_out1.y = -q_in1.y
    q_out1.z = -q_in1.z
    q_out1.w =  q_in1.w
