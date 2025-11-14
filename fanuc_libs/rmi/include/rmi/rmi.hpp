// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <list>
#include <memory>
#include <mutex>
#include <optional>

#include "rmi/packets.hpp"

namespace rmi
{
class RMIConnectionInterface
{
public:
  virtual ~RMIConnectionInterface() = default;

  virtual ConnectROS2Packet::Response connect(std::optional<double> timeout) = 0;

  virtual DisconnectPacket::Response disconnect(std::optional<double> timeout) = 0;

  virtual InitializePacket::Response initializeRemoteMotion(std::optional<double> timeout) = 0;

  virtual ProgramCallPacket::Response programCall(const std::string& program_name, std::optional<double> timeout) = 0;

  virtual ProgramCallPacket::Request programCallNonBlocking(const std::string& program_name) = 0;

  virtual StatusRequestPacket::Response getStatus(std::optional<double> timeout) = 0;

  virtual SetSpeedOverridePacket::Response setSpeedOverride(int value, std::optional<double> timeout) = 0;

  virtual AbortPacket::Response abort(std::optional<double> timeout) = 0;

  virtual PausePacket::Response pause(std::optional<double> timeout) = 0;

  virtual ContinuePacket::Response resume(std::optional<double> timeout) = 0;

  virtual ResetRobotPacket::Response reset(std::optional<double> timeout) = 0;

  virtual ReadErrorPacket::Response readError(std::optional<double> timeout) = 0;

  virtual WritePositionRegisterPacket::Response
  writePositionRegister(int register_number, const std::string& representation, const ConfigurationData& configuration,
                        const PositionData& position, const JointAngleData& joint_angle,
                        std::optional<double> timeout) = 0;
  virtual ReadPositionRegisterPacket::Response readPositionRegister(int register_number,
                                                                    std::optional<double> timeout) = 0;

  virtual ReadNumericRegisterPacket::Response readNumericRegister(int register_number,
                                                                  std::optional<double> timeout) = 0;

  virtual WriteNumericRegisterPacket::Response writeNumericRegister(int register_number, std::variant<int, float> value,
                                                                    std::optional<double> timeout) = 0;

  virtual ReadDigitalInputPortPacket::Response readDigitalInputPort(uint16_t port_number,
                                                                    std::optional<double> timeout) = 0;

  virtual WriteDigitalOutputPacket::Response writeDigitalOutputPort(uint16_t port_number, bool port_value,
                                                                    std::optional<double> timeout) = 0;

  virtual ReadIOPortPacket::Response readIOPort(const std::string& port_type, int port_number,
                                                std::optional<double> timeout) = 0;

  virtual WriteIOPortPacket::Response writeIOPort(int port_number, const std::string& port_type,
                                                  std::variant<int, float> port_value,
                                                  std::optional<double> timeout) = 0;

  virtual ReadVariablePacket::Response readVariablePacket(const std::string& variable_name,
                                                          std::optional<double> timeout) = 0;

  virtual WriteVariablePacket::Response writeVariablePacket(const std::string& variable_name,
                                                            std::variant<int, float> value,
                                                            std::optional<double> timeout) = 0;

  virtual GetExtendedStatusPacket::Response getExtendedStatus(std::optional<double> timeout) = 0;

  virtual SetPayloadPacket::Response setPayloadSchedule(uint8_t payload_schedule_number,
                                                        std::optional<double> timeout) = 0;

  virtual SetPayloadValuePacket::Response setPayloadValue(uint8_t payload_schedule_number, float mass, float cg_x,
                                                          float cg_y, float cg_z, bool use_in, float in_x, float in_y,
                                                          float in_z, std::optional<double> timeout) = 0;

  virtual SetPayloadCompPacket::Response setPayloadComp(uint8_t payload_schedule_number, float mass, float cg_x,
                                                        float cg_y, float cg_z, float in_x, float in_y, float in_z,
                                                        std::optional<double> timeout) = 0;

  virtual ReadJointAnglesPacket::Response readJointAngles(const std::optional<uint8_t>& group,
                                                          std::optional<double> timeout) = 0;

  virtual JointMotionJRepPacket::Response sendJointMotion(JointMotionJRepPacket::Request joint_motion_request,
                                                          std::optional<double> timeout) = 0;

  virtual std::optional<SystemFaultPacket> checkSystemFault() = 0;

  virtual std::optional<TimeoutTerminatePacket> checkTimeoutTerminate() = 0;

  virtual std::optional<CommunicationPacket> checkCommunicationPacket() = 0;

  virtual std::optional<UnknownPacket> checkUnknownPacket() = 0;
};

class RMIConnection final : public RMIConnectionInterface
{
public:
  explicit RMIConnection(const std::string& robot_ip_address, uint16_t rmi_port = 16001);

  ~RMIConnection() override;

  RMIConnection(const RMIConnection&) = delete;

  RMIConnection& operator=(const RMIConnection&) = delete;

  ConnectROS2Packet::Response connect(std::optional<double> timeout) override;

  DisconnectPacket::Response disconnect(std::optional<double> timeout) override;

  InitializePacket::Response initializeRemoteMotion(std::optional<double> timeout) override;

  ProgramCallPacket::Response programCall(const std::string& program_name, std::optional<double> timeout) override;

  ProgramCallPacket::Request programCallNonBlocking(const std::string& program_name) override;

  StatusRequestPacket::Response getStatus(std::optional<double> timeout) override;

  SetSpeedOverridePacket::Response setSpeedOverride(int value, std::optional<double> timeout) override;

  AbortPacket::Response abort(std::optional<double> timeout) override;

  PausePacket::Response pause(std::optional<double> timeout) override;

  ContinuePacket::Response resume(std::optional<double> timeout) override;

  ResetRobotPacket::Response reset(std::optional<double> timeout) override;

  ReadErrorPacket::Response readError(std::optional<double> timeout) override;

  WritePositionRegisterPacket::Response writePositionRegister(int register_number, const std::string& representation,
                                                              const ConfigurationData& configuration,
                                                              const PositionData& position,
                                                              const JointAngleData& joint_angle,
                                                              std::optional<double> timeout) override;

  ReadPositionRegisterPacket::Response readPositionRegister(int register_number, std::optional<double> timeout) override;

  ReadNumericRegisterPacket::Response readNumericRegister(int register_number, std::optional<double> timeout) override;

  WriteNumericRegisterPacket::Response writeNumericRegister(int register_number, std::variant<int, float> value,
                                                            std::optional<double> timeout) override;

  ReadDigitalInputPortPacket::Response readDigitalInputPort(uint16_t port_number,
                                                            std::optional<double> timeout) override;

  WriteDigitalOutputPacket::Response writeDigitalOutputPort(uint16_t port_number, bool port_value,
                                                            std::optional<double> timeout) override;

  ReadIOPortPacket::Response readIOPort(const std::string& port_type, int port_number,
                                        std::optional<double> timeout) override;

  WriteIOPortPacket::Response writeIOPort(int port_number, const std::string& port_type,
                                          std::variant<int, float> port_value, std::optional<double> timeout) override;

  ReadVariablePacket::Response readVariablePacket(const std::string& variable_name,
                                                  std::optional<double> timeout) override;

  WriteVariablePacket::Response writeVariablePacket(const std::string& variable_name, std::variant<int, float> value,
                                                    std::optional<double> timeout) override;

  GetExtendedStatusPacket::Response getExtendedStatus(std::optional<double> timeout) override;

  SetPayloadPacket::Response setPayloadSchedule(uint8_t payload_schedule_number, std::optional<double> timeout) override;

  SetPayloadValuePacket::Response setPayloadValue(uint8_t payload_schedule_number, float mass, float cg_x, float cg_y,
                                                  float cg_z, bool use_in, float in_x, float in_y, float in_z,
                                                  std::optional<double> timeout) override;

  SetPayloadCompPacket::Response setPayloadComp(uint8_t payload_schedule_number, float mass, float cg_x, float cg_y,
                                                float cg_z, float in_x, float in_y, float in_z,
                                                std::optional<double> timeout) override;

  ReadJointAnglesPacket::Response readJointAngles(const std::optional<uint8_t>& group,
                                                  std::optional<double> timeout) override;

  JointMotionJRepPacket::Response sendJointMotion(JointMotionJRepPacket::Request joint_motion_request,
                                                  std::optional<double> timeout) override;

  std::optional<SystemFaultPacket> checkSystemFault() override;

  std::optional<TimeoutTerminatePacket> checkTimeoutTerminate() override;

  std::optional<CommunicationPacket> checkCommunicationPacket() override;

  std::optional<UnknownPacket> checkUnknownPacket() override;

  template <typename T>
  typename T::Response sendRMIPacket(typename T::Request& request_packet, std::optional<double> timeout);

private:
  struct PConnectionImpl;

  int32_t getSequenceNumber();

  template <typename T>
  T getResponsePacket(std::optional<double> timeout_optional, const std::string& error_message_prefix,
                      std::optional<int> expected_sequence_id);

  template <typename T>
  std::optional<T> checkPushPacket();

  void drainConnectionBuffer();

  const std::string robot_ip_address_;
  const uint16_t rmi_port_;

  int32_t sequence_number_;
  std::list<std::string> json_responses_;
  mutable std::mutex mutex_;
  mutable std::mutex motion_mutex_;

  const std::unique_ptr<PConnectionImpl> connection_impl_;
};

}  // namespace rmi
// TODO: Add doc comments.
