// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "rmi/rmi.hpp"

int main()
{
  rmi::RMIConnection rmi_connection("192.168.1.100");
  const rmi::ConnectROS2Packet::Response connect_response = rmi_connection.connect(std::nullopt);
  try
  {
    auto status_response = rmi_connection.getStatus(std::nullopt);
    const rmi::InitializePacket::Response initialize_packet_response =
        rmi_connection.initializeRemoteMotion(std::nullopt);
  }
  catch (const std::runtime_error& e)
  {
    std::cout << "Can't reset and abort" << std::endl;
    rmi_connection.reset(std::nullopt);
    rmi_connection.abort(std::nullopt);
    auto status_response = rmi_connection.getStatus(std::nullopt);
    const rmi::InitializePacket::Response initialize_packet_response =
        rmi_connection.initializeRemoteMotion(std::nullopt);
  }

  auto set_speed_override_packet_response = rmi_connection.setSpeedOverride(100, std::nullopt);

  rmi::JointMotionJRepPacket::Request request_joint_motion;
  request_joint_motion.JointAngle.J1 = 0.0;
  request_joint_motion.JointAngle.J2 = 0.0;
  request_joint_motion.JointAngle.J3 = 0.0;
  request_joint_motion.JointAngle.J4 = 0.0;
  request_joint_motion.JointAngle.J5 = -90;
  request_joint_motion.JointAngle.J6 = 0.0;
  request_joint_motion.SpeedType = "Percent";
  request_joint_motion.Speed = 100;
  request_joint_motion.TermType = "FINE";
  rmi_connection.sendRMIPacket<rmi::JointMotionJRepPacket>(request_joint_motion, std::nullopt);

  // Move the robot by a linear offset
  rmi::LinearRelativePacket::Request request;
  request.Position.Z = -30.0;
  request.Configuration.UToolNumber = 1;
  request.SpeedType = "mmSec";
  request.Speed = 1000;
  request.TermType = "FINE";
  rmi_connection.sendRMIPacket<rmi::LinearRelativePacket>(request, std::nullopt);
  request.Position.Z = -request.Position.Z;
  rmi_connection.sendRMIPacket<rmi::LinearRelativePacket>(request, std::nullopt);

  request_joint_motion.JointAngle.J2 = 10.0;
  rmi_connection.sendRMIPacket<rmi::JointMotionJRepPacket>(request_joint_motion, std::nullopt);

  const auto read_numeric_register_response = rmi_connection.readNumericRegister(2, std::nullopt);
  rmi_connection.writeNumericRegister(5, 7.75f, std::nullopt);

  const auto get_extended_status_response = rmi_connection.getExtendedStatus(std::nullopt);
  const auto read_io_port_response = rmi_connection.readIOPort("DO", 101, std::nullopt);
  rmi_connection.writeIOPort(5, "RO", 1.2f, std::nullopt);

  const auto read_variable_response = rmi_connection.readVariablePacket("$STMO.$COM_INT", std::nullopt);
  std::cout << "Stream Motion's interval: " << std::get<int>(read_variable_response.VariableValue) << " ms"
            << std::endl;

  // TODO: FRC_WriteVariable command is currently not recognized by the robot controller
  // rmi_connection.writeVariablePacket("$STMO.$PHYS_PORT", 2);

  std::string representation;
  representation = "Cartesian";
  rmi::ConfigurationData configuration;
  configuration.Front = 1;
  rmi::PositionData position;
  position.P = 1.5;
  position.W = 2.5;
  rmi::JointAngleData joint_angle;
  const auto write_position_register_packet =
      rmi_connection.writePositionRegister(1, representation, configuration, position, joint_angle, 1);
  const auto read_position_register_packet = rmi_connection.readPositionRegister(1, std::nullopt);

  const auto read_digital_input_port_packet = rmi_connection.readDigitalInputPort(81, std::nullopt);
  const auto write_digital_input_port_packet = rmi_connection.writeDigitalOutputPort(82, true, std::nullopt);

  const auto reset_response = rmi_connection.reset(std::nullopt);
  try
  {
    rmi_connection.setPayloadSchedule(1, std::nullopt);
    rmi_connection.programCallNonBlocking("STREAM_MOTN");
    const auto pause_response_packet = rmi_connection.pause(std::nullopt);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto status_response = rmi_connection.getStatus(std::nullopt);
    const auto continue_response_packet = rmi_connection.resume(std::nullopt);
    status_response = rmi_connection.getStatus(std::nullopt);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    status_response = rmi_connection.getStatus(std::nullopt);
    const auto abort_response = rmi_connection.abort(std::nullopt);
    status_response = rmi_connection.getStatus(std::nullopt);
  }
  catch (const std::runtime_error& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }
  const rmi::DisconnectPacket::Response disconnect_packet = rmi_connection.disconnect(std::nullopt);

  if (auto system_fault_packet = rmi_connection.checkSystemFault())
  {
    std::cout << "System Faulted" << std::endl;
  }
}
