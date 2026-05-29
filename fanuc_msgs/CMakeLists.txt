# SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(fanuc_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/IOType.msg

  msg/AnalogIO.msg
  msg/AnalogIOCmd.msg
  msg/AnalogIOState.msg

  msg/BoolIO.msg
  msg/IOCmd.msg
  msg/IOState.msg

  msg/NumReg.msg
  msg/NumRegCmd.msg
  msg/NumRegState.msg

  msg/ConnectionStatus.msg
  msg/RobotStatus.msg
  msg/RobotStatusExt.msg
  msg/CollaborativeSpeedScaling.msg

  msg/ForceSensor.msg
)
set(srv_files
  srv/SetAnalogIO.srv
  srv/GetAnalogIO.srv

  srv/SetBoolIO.srv
  srv/GetBoolIO.srv

  srv/SetGroupIO.srv
  srv/GetGroupIO.srv

  srv/SetNumReg.srv
  srv/GetNumReg.srv

  srv/SetPosReg.srv
  srv/GetPosReg.srv

  srv/SetGenOverride.srv
  srv/SetPayloadID.srv
  srv/SetPayloadValue.srv
  srv/SetPayloadComp.srv

  srv/CfgForceSensor.srv

  srv/SwitchControlState.srv
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

ament_package()
