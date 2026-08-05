# SPDX-FileCopyrightText: 2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(rmi_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/Joint.msg
  msg/Cartesian.msg
  msg/Representation.msg
  msg/MotionType.msg
  msg/SpeedType.msg
  msg/TermType.msg
  msg/MotionInstructionOptions.msg
  msg/Command.msg
  msg/CommandOptions.msg
  msg/CallParam.msg
)

set(srv_files
  srv/AddMotionInstruction.srv
  srv/CallCommand.srv
  srv/AddCall.srv
  srv/AddWaitDIN.srv
  srv/AddSetUFrame.srv
  srv/AddSetUTool.srv
  srv/AddWaitTime.srv
  srv/AddSetPayload.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

ament_package()
