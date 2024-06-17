/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehoon Lim (Darby) */
/* Modified by Marina Nelson */

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/VersionInfo.h>
#include <string>
#include <iostream>

#define SOFTWARE_VERSION "1.2.5"
#define HARDWARE_VERSION "2020.03.16"
#define FIRMWARE_VERSION_MAJOR_NUMBER 1
#define FIRMWARE_VERSION_MINOR_NUMBER 2

ros::Publisher tb3_version_info_pub;
ros::Publisher tb3_diagnostics_pub;

diagnostic_msgs::DiagnosticStatus imu_state;
diagnostic_msgs::DiagnosticStatus motor_state;
diagnostic_msgs::DiagnosticStatus LDS_state;
diagnostic_msgs::DiagnosticStatus battery_state;
diagnostic_msgs::DiagnosticStatus button_state;

typedef struct
{
  int major_number;
  int minor_number;
  int patch_number;
} VERSION;

void setDiagnosisMsg(diagnostic_msgs::DiagnosticStatus *diag, uint8_t level, std::string name, std::string message, std::string hardware_id)
{
  diag->level = level;
  diag->name = name;
  diag->message = message;
  diag->hardware_id = hardware_id;
}

void setIMUDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&imu_state, level, "IMU Sensor", message, "MPU9250");
}

void setMotorDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&motor_state, level, "Actuator", message, "DYNAMIXEL X");
}

void setBatteryDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&battery_state, level, "Power System", message, "Battery");
}

void setLDSDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&LDS_state, level, "Lidar Sensor", message, "HLS-LFCD-LDS");
}

void setButtonDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&button_state, level, "Analog Button", message, "OpenCR Button");
}

void imuMsgCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  setIMUDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}

void LDSMsgCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  setLDSDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}

void sensorStateMsgCallback(const turtlebot3_msgs::SensorState::ConstPtr &msg)
{
  if (msg->battery > 11.0)
    setBatteryDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
  else
    setBatteryDiagnosis(diagnostic_msgs::DiagnosticStatus::WARN, "Charge!!! Charge!!!");

  if (msg->button == turtlebot3_msgs::SensorState::BUTTON0)
    setButtonDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "BUTTON 0 IS PUSHED");
  else if (msg->button == turtlebot3_msgs::SensorState::BUTTON1)
    setButtonDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "BUTTON 1 IS PUSHED");
  else
    setButtonDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Pushed Nothing");

  if (msg->torque == true)
    setMotorDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Torque ON");
  else
    setMotorDiagnosis(diagnostic_msgs::DiagnosticStatus::WARN, "Torque OFF");
}

void firmwareVersionMsgCallback(const turtlebot3_msgs::VersionInfo::ConstPtr &msg)
{
  static bool check_version = false;

  VERSION firmware_version;
  std::vector<std::string> get_version;

  std::string firmware_version_str = msg->firmware;
  std::istringstream ss(firmware_version_str);
  std::string token;

  while (std::getline(ss, token, '.'))
  {
    get_version.push_back(token);
  }

  firmware_version.major_number = std::stoi(get_version[0]);
  firmware_version.minor_number = std::stoi(get_version[1]);
  firmware_version.patch_number = std::stoi(get_version[2]);

  if (check_version == false)
  {
    if (firmware_version.major_number == FIRMWARE_VERSION_MAJOR_NUMBER)
    {
      if (firmware_version.minor_number > FIRMWARE_VERSION_MINOR_NUMBER)
      {
        ROS_WARN("This firmware(v%s) may not compatible with this software (v%s)", msg->firmware.c_str(), SOFTWARE_VERSION);
        ROS_WARN("You can find how to update its in `FAQ` section(turtlebot3.robotis.com)");
      }
    }
    else
    {
      ROS_WARN("This firmware(v%s) may not compatible with this software (v%s)", msg->firmware.c_str(), SOFTWARE_VERSION);
      ROS_WARN("You can find how to update its in `FAQ` section(turtlebot3.robotis.com)");
    }

    check_version = true;
  }

  turtlebot3_msgs::VersionInfo version;
  version.software = SOFTWARE_VERSION;
  version.hardware = HARDWARE_VERSION;
  version.firmware = msg->firmware;

  tb3_version_info_pub.publish(version);
}

void msgPub()
{
  diagnostic_msgs::DiagnosticArray tb3_diagnostics;

  tb3_diagnostics.header.stamp = ros::Time::now();

  tb3_diagnostics.status.clear();
  tb3_diagnostics.status.push_back(imu_state);
  tb3_diagnostics.status.push_back(motor_state);
  tb3_diagnostics.status.push_back(LDS_state);
  tb3_diagnostics.status.push_back(battery_state);
  tb3_diagnostics.status.push_back(button_state);

  tb3_diagnostics_pub.publish(tb3_diagnostics);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot3_diagnostic");

  // Retrieve ugv_name parameter from ROS parameter server
  std::string ugv_name;
  if (!ros::param::get("/ugv_name", ugv_name))
  {
    ROS_ERROR("Failed to get ugv_name parameter");
    return 1;
  }

  // Construct node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_ns("/" + ugv_name);

  // Publishers
  tb3_diagnostics_pub = nh_ns.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
  tb3_version_info_pub = nh_ns.advertise<turtlebot3_msgs::VersionInfo>("version_info", 10);

  // Subscribers
  ros::Subscriber imu = nh_ns.subscribe("imu", 10, imuMsgCallback);
  ros::Subscriber lds = nh_ns.subscribe("scan", 10, LDSMsgCallback);
  ros::Subscriber tb3_sensor = nh_ns.subscribe("sensor_state", 10, sensorStateMsgCallback);
  ros::Subscriber version = nh_ns.subscribe("firmware_version", 10, firmwareVersionMsgCallback);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    msgPub();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
