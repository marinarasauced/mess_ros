#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped, Twist
from mess_msgs.msg import CalibrateUGV, StateUGV
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
import numpy as np
import os
import json
import csv

##################################################################################

# Import JSON data:
f = open(os.path.dirname(__file__) + "/../../config/config.json")
fData = json.load(f)

# Object name in VICON Tracker:
UGV_NAME = fData["ugv_name"]

##################################################################################

MSG01 = """
Initializing TurtleBot3 onboard logger

CTRL-C to quit
-----------------------------------------
"""

MSG02 = """
Logging data to .csv files in current directory
"""

MSG03 = """
Data logged to .csv files in current directory
"""

MSG04 = """
Communication ended
"""

##################################################################################

def log_data(name, head, data):
    path = "/home/ubuntu/mess_ros/logs/csv/"
    if not os.path.exists(path):
        os.makedirs(path)
    with open(path + name + ".csv", "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(head)
        rows, cols = data.shape
        row = 0
        while row < rows:
            line = []
            col = 0
            while col < cols:
                append_data(line, str(data[row, col]))
                col += 1
            row += 1
            writer.writerow(line)

def append_data(array, data):
    if hasattr(array, "append"):
        array.append(data)

##################################################################################

def callback_control_input(data):
    append_data(ut, rospy.get_time())
    append_data(uLin, data.linear.x)
    append_data(uAng, data.angular.z)

def callback_imu_raw(data):
    append_data(imut, rospy.get_time())
    append_data(imuQx, data.orientation.x)
    append_data(imuQy, data.orientation.y)
    append_data(imuQz, data.orientation.z)
    append_data(imuQw, data.orientation.w)
    append_data(imuWx, data.angular_velocity.x)
    append_data(imuWy, data.angular_velocity.y)
    append_data(imuWz, data.angular_velocity.z)
    append_data(imuAx, data.linear_acceleration.x)
    append_data(imuAy, data.linear_acceleration.y)
    append_data(imuAz, data.linear_acceleration.z)

def callback_odom_raw(data):
    append_data(ort, rospy.get_time())
    append_data(orTx, data.pose.pose.position.x)
    append_data(orTy, data.pose.pose.position.y)
    append_data(orTz, data.pose.pose.position.z)
    append_data(orQx, data.pose.pose.orientation.x)
    append_data(orQy, data.pose.pose.orientation.y)
    append_data(orQz, data.pose.pose.orientation.z)
    append_data(orQw, data.pose.pose.orientation.w)

def callback_vicon_cal(data):
    append_data(vct, rospy.get_time())
    append_data(vcTx, data.Tx)
    append_data(vcTy, data.Ty)
    append_data(vcRz, data.Rz)

def callback_vicon_raw(data):
    append_data(vrt, rospy.get_time())
    append_data(vrTx, data.transform.translation.x)
    append_data(vrTy, data.transform.translation.y)
    append_data(vrTz, data.transform.translation.z)
    append_data(vrQx, data.transform.rotation.x)
    append_data(vrQy, data.transform.rotation.y)
    append_data(vrQz, data.transform.rotation.z)
    append_data(vrQw, data.transform.rotation.w)

def callback_joints(data):
    append_data(wt, rospy.get_time())
    jointV = data.velocity
    jointE = data.effort
    append_data(wVl, jointV[0])
    append_data(wVr, jointV[1])
    append_data(wEl, jointE[0])
    append_data(wEr, jointE[1])

##################################################################################

if __name__=="__main__":

    # Initialize node:
    rospy.init_node("messlogger_ugv")

    # Control input:
    ut = []
    uLin = []
    uAng = []
    rospy.Subscriber("/" + UGV_NAME + "/cmd_vel", Twist, callback_control_input)

    # IMU raw:
    imut = []
    imuQx = []
    imuQy = []
    imuQz = []
    imuQw = []
    imuWx = []
    imuWy = []
    imuWz = []
    imuAx = []
    imuAy = []
    imuAz = []
    rospy.Subscriber("/" + UGV_NAME + "/imu", Imu, callback_imu_raw)

    # Odometry raw:
    ort = []
    orTx = []
    orTy = []
    orTz = []
    orQx = []
    orQy = []
    orQz = []
    orQw = []
    rospy.Subscriber("/" + UGV_NAME + "/odom", Odometry, callback_odom_raw)

    # VICON calibrated:
    vct = []
    vcTx = []
    vcTy = []
    vcRz = []
    rospy.Subscriber("/" + UGV_NAME + "/messop/logger/vicon", StateUGV, callback_vicon_cal)

    # VICON raw:
    vrt = []
    vrTx = []
    vrTy = []
    vrTz = []
    vrQx = []
    vrQy = []
    vrQz = []
    vrQw = []
    rospy.Subscriber("/vicon/" + UGV_NAME + "/" + UGV_NAME, TransformStamped, callback_vicon_raw)

    # Wheel velocities and torques:
    wt = []
    wVl = []
    wVr = []
    wEl = []
    wEr = []
    rospy.Subscriber("/" + UGV_NAME + "/joint_state", JointState, callback_joints)

    try:

        # Indicate the node is initiated:
        print(MSG01)

        #
        rospy.spin()

        # Indicate loggings is initiated:
        rospy.on_shutdown(lambda: print(MSG02))

        # Retrieve current date:
        # month = time.strftime("%m")
        # day = time.strftime("%d")
        # year = time.strftime("%Y")
        # today = "_" + month + day + year
        today = ""

        # Log calibration coefficients:
        name = "calibration_coefficients" + today
        head = ["time (s)", "c1", "c2", "c3", "c4", "c5", "c6", "c7"]
        data = np.array([ct, c1, c2, c3, c4, c5, c6, c7]).T
        rospy.on_shutdown(lambda: log_data(name, head, data))

        # Log control input:
        name = "control_input" + today
        head = ["time (s)", "v_x (m/s)", "omega_z (rad/s)"]
        data = np.array([ut, uLin, uAng]).T
        rospy.on_shutdown(lambda: log_data(name, head, data))

        # Log IMU raw:
        name = "imu_raw" + today
        data_imu_raw = ["time (s)", "q_x", "q_y", "q_z", "q_w", "omega_x (rad/s)", "omega_y (rad/s)", "omega_z (rad/s)", "a_x (m/s^2)", "a_y (m/s^2)", "a_z (m/s^2)"]
        data = np.array([imut, imuQx, imuQy]).T
        rospy.on_shutdown(lambda: log_data(name, head, data))

        # Log odometry calibrated:
        name = "odometry_calibrated" + today
        head = ["time (s)", "x (m)", "y (m)", "psi (rad)"]
        data = np.array([oct, ocTx, ocTy, ocRz]).T
        rospy.on_shutdown(lambda: log_data(name, head, data))

        # Log odometry raw:
        name = "odometry_raw" + today
        head = ["time (s)", "x (m)", "y (m)", "z (m)", "q_x", "q_y", "q_z", "q_w"]
        data = np.array([ort, orTx, orTy, orTz, orQx, orQy, orQz, orQw]).T
        rospy.on_shutdown(lambda: log_data(name, head, data))

        # Log onboard state:
        name = "state_onboard" + today
        head = ["time (s)", "x (m)", "y (m)", "psi (rad)"]
        data = np.array([sot, soTx, soTy, soRz]).T
        rospy.on_shutdown(lambda: log_data(name, head, data))

        # Log VICON calibrated:
        name = "vicon_calibrated" + today
        head = ["time (s)", "x (m)", "y (m)", "psi (rad)"]
        data = np.array([vct, vcTx, vcTy, vcRz]).T
        rospy.on_shutdown(lambda: log_data(name, head, data))

        # Log VICON raw:
        name = "vicon_raw" + today
        head = ["time (s)", "x (m)", "y (m)", "z (m)", "q_x", "q_y", "q_z", "q_w"]
        data = np.array([vrt, vrTx, vrTy, vrTz, vrQx, vrQy, vrQz, vrQw]).T
        rospy.on_shutdown(lambda: log_data(name, head, data))

        # Log wheel velocities and torques:
        name = "wheel_velocities_and_torques" + today
        head = ["time (s)", "left vel", "right vel", "left effort", "right effort"]
        data = np.array([wt, wVl, wVr, wEl, wEr]).T
        rospy.on_shutdown(lambda: log_data(name, head, data))

        # Indicate logging is complete:
        rospy.on_shutdown(lambda: print(MSG03))

    except rospy.ROSInterruptException:

        # Indicate node is shut down:
        print(MSG04)
