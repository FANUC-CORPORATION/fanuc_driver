// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <list>
#include <memory>
#include <mutex>
#include <optional>

#include "rmi/packets.hpp"
#include "rmi/struct.hpp"

namespace rmi
{
class RMIConnectionInterface
{
public:
  virtual ~RMIConnectionInterface() = default;

  virtual ConnectROS2Packet::Response connect(std::optional<double> timeout) = 0;

  virtual DisconnectPacket::Response disconnect(std::optional<double> timeout) = 0;

  virtual InitializePacket::Response initializeRemoteMotion(std::optional<double> timeout) = 0;
  virtual InitializePacket::Response initializeRemoteMotion(std::optional<double> timeout,
                                                            const std::optional<uint8_t> groupmask,
                                                            const std::optional<std::string>& rtsa,
                                                            const std::optional<std::string>& pltzmode) = 0;

  virtual ProgramCallPacket::Response programCall(const std::string& program_name, std::optional<double> timeout) = 0;

  virtual ProgramCallPacket::Response programCall(const std::string& program_name, std::optional<double> timeout,
                                                  const std::vector<RMICallParam>& params) = 0;

  virtual ProgramCallPacket::Request programCallNonBlocking(const std::string& program_name) = 0;

  virtual ProgramCallPacket::Request programCallNonBlocking(const std::string& program_name,
                                                            const std::vector<RMICallParam>& params) = 0;

  virtual StatusRequestPacket::Response getStatus(std::optional<double> timeout) = 0;

  virtual SetSpeedOverridePacket::Response setSpeedOverride(int value, std::optional<double> timeout) = 0;

  virtual AbortPacket::Response abort(std::optional<double> timeout) = 0;

  virtual PausePacket::Response pause(std::optional<double> timeout) = 0;

  virtual ContinuePacket::Response resume(std::optional<double> timeout) = 0;

  virtual ResetRobotPacket::Response reset(std::optional<double> timeout) = 0;

  virtual ReadErrorPacket::Response readError(std::optional<double> timeout) = 0;

  virtual ReadErrorPacket::Response readError(std::optional<double> timeout, const std::optional<uint8_t> count) = 0;

  virtual GetUFrameToolFramePacket::Response getUFrameUTool(std::optional<double> timeout,
                                                            const std::optional<uint8_t> group) = 0;
  virtual SetUFrameToolFramePacket::Response setUFrameUTool(const int uframe, const int utool,
                                                            std::optional<double> timeout,
                                                            const std::optional<uint8_t> group) = 0;

  virtual ReadUFrameDataPacket::Response readUFrameData(const int uframe, std::optional<double> timeout,
                                                        const std::optional<uint8_t> group) = 0;

  virtual WriteUFrameDataPacket::Response writeUFrameData(const int uframe, const FrameData data,
                                                          std::optional<double> timeout,
                                                          const std::optional<uint8_t> group) = 0;

  virtual ReadUToolDataPacket::Response readUToolData(const int utool, std::optional<double> timeout,
                                                      const std::optional<uint8_t> group) = 0;

  virtual WriteUToolDataPacket::Response writeUToolData(const int utool, const FrameData data,
                                                        std::optional<double> timeout,
                                                        const std::optional<uint8_t> group) = 0;

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

  virtual GetPayloadPacket::Response getPayloadSchedule(std::optional<double> timeout) = 0;

  virtual GetPayloadValuePacket::Response getPayloadValue(uint16_t payload_schedule_number,
                                                          std::optional<double> timeout) = 0;

  virtual GetPayloadCompPacket::Response getPayloadComp(uint16_t payload_schedule_number,
                                                        std::optional<double> timeout) = 0;

  virtual ReadJointAnglesPacket::Response readJointAngles(const std::optional<uint8_t>& group,
                                                          std::optional<double> timeout) = 0;

  virtual GetCartesianPositionPacket::Response getCartesianPosition(std::optional<double> timeout,
                                                                    const std::optional<uint8_t> group) = 0;

  virtual GetTCPSpeedPacket::Response getTCPSpeed(std::optional<double> timeout, const std::optional<uint8_t> group) = 0;

  virtual JointMotionJRepPacket::Response sendJointMotion(JointMotionJRepPacket::Request joint_motion_request,
                                                          std::optional<double> timeout) = 0;

  virtual std::optional<SystemFaultPacket> checkSystemFault() = 0;

  virtual std::optional<TimeoutTerminatePacket> checkTimeoutTerminate() = 0;

  virtual std::optional<CommunicationPacket> checkCommunicationPacket() = 0;

  virtual std::optional<UnknownPacket> checkUnknownPacket() = 0;

  virtual std::optional<InstructionResponse> getLastInstructionResponse() = 0;

  virtual void sendRMIPacketNonBlocking(JointMotionPacket::Request& packet) = 0;
  virtual void sendRMIPacketNonBlocking(JointMotionJRepPacket::Request& packet) = 0;
  virtual void sendRMIPacketNonBlocking(JointRelativePacket::Request& packet) = 0;
  virtual void sendRMIPacketNonBlocking(JointRelativeJRepPacket::Request& packet) = 0;

  virtual void sendRMIPacketNonBlocking(LinearMotionPacket::Request& packet) = 0;
  virtual void sendRMIPacketNonBlocking(LinearMotionJRepPacket::Request& packet) = 0;
  virtual void sendRMIPacketNonBlocking(LinearRelativePacket::Request& packet) = 0;
  virtual void sendRMIPacketNonBlocking(LinearRelativeJRepPacket::Request& packet) = 0;

  virtual void sendRMIPacketNonBlocking(CircularMotionPacket::Request& packet) = 0;
  virtual void sendRMIPacketNonBlocking(CircularRelativePacket::Request& packet) = 0;

  virtual void sendRMIPacketNonBlocking(SplineMotionPacket::Request& packet) = 0;
  virtual void sendRMIPacketNonBlocking(SplineMotionJRepPacket::Request& packet) = 0;

  virtual void sendRMIPacketNonBlocking(WaitForDINPacket::Request& packet) = 0;
  virtual void sendRMIPacketNonBlocking(SetUFramePacket::Request& packet) = 0;
  virtual void sendRMIPacketNonBlocking(SetToolFramePacket::Request& packet) = 0;
  virtual void sendRMIPacketNonBlocking(WaitForTimePacket::Request& packet) = 0;
  virtual void sendRMIPacketNonBlocking(SetPayloadInstructionPacket::Request& packet) = 0;

  virtual std::string getErrorMessageString(const uint32_t error_code) = 0;

  virtual int32_t getRemainingBuffuerSize() = 0;
  virtual void setEncoding(const std::string& encoding) = 0;
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
  InitializePacket::Response initializeRemoteMotion(std::optional<double> timeout,
                                                    const std::optional<uint8_t> groupmask,
                                                    const std::optional<std::string>& rtsa,
                                                    const std::optional<std::string>& pltzmode) override;

  ProgramCallPacket::Response programCall(const std::string& program_name, std::optional<double> timeout) override;

  ProgramCallPacket::Response programCall(const std::string& program_name, std::optional<double> timeout,
                                          const std::vector<RMICallParam>& params) override;

  ProgramCallPacket::Request programCallNonBlocking(const std::string& program_name) override;

  ProgramCallPacket::Request programCallNonBlocking(const std::string& program_name,
                                                    const std::vector<RMICallParam>& params) override;

  StatusRequestPacket::Response getStatus(std::optional<double> timeout) override;

  SetSpeedOverridePacket::Response setSpeedOverride(int value, std::optional<double> timeout) override;

  AbortPacket::Response abort(std::optional<double> timeout) override;

  PausePacket::Response pause(std::optional<double> timeout) override;

  ContinuePacket::Response resume(std::optional<double> timeout) override;

  ResetRobotPacket::Response reset(std::optional<double> timeout) override;

  ReadErrorPacket::Response readError(std::optional<double> timeout) override;

  ReadErrorPacket::Response readError(std::optional<double> timeout, const std::optional<uint8_t> count) override;

  GetUFrameToolFramePacket::Response getUFrameUTool(std::optional<double> timeout,
                                                    const std::optional<uint8_t> group) override;

  SetUFrameToolFramePacket::Response setUFrameUTool(const int uframe, const int utool, std::optional<double> timeout,
                                                    const std::optional<uint8_t> group) override;

  ReadUFrameDataPacket::Response readUFrameData(const int uframe, std::optional<double> timeout,
                                                const std::optional<uint8_t> group) override;

  WriteUFrameDataPacket::Response writeUFrameData(const int uframe, const FrameData data, std::optional<double> timeout,
                                                  const std::optional<uint8_t> group) override;

  ReadUToolDataPacket::Response readUToolData(const int utool, std::optional<double> timeout,
                                              const std::optional<uint8_t> group) override;

  WriteUToolDataPacket::Response writeUToolData(const int utool, const FrameData data, std::optional<double> timeout,
                                                const std::optional<uint8_t> group) override;

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

  GetPayloadPacket::Response getPayloadSchedule(std::optional<double> timeout) override;

  GetPayloadValuePacket::Response getPayloadValue(uint16_t payload_schedule_number,
                                                  std::optional<double> timeout) override;

  GetPayloadCompPacket::Response getPayloadComp(uint16_t payload_schedule_number,
                                                std::optional<double> timeout) override;

  ReadJointAnglesPacket::Response readJointAngles(const std::optional<uint8_t>& group,
                                                  std::optional<double> timeout) override;
  GetCartesianPositionPacket::Response getCartesianPosition(std::optional<double> timeout,
                                                            const std::optional<uint8_t> group) override;
  GetTCPSpeedPacket::Response getTCPSpeed(std::optional<double> timeout, const std::optional<uint8_t> group) override;

  JointMotionJRepPacket::Response sendJointMotion(JointMotionJRepPacket::Request joint_motion_request,
                                                  std::optional<double> timeout) override;

  void sendRMIPacketNonBlocking(JointMotionPacket::Request& packet) override;
  void sendRMIPacketNonBlocking(JointMotionJRepPacket::Request& packet) override;
  void sendRMIPacketNonBlocking(JointRelativePacket::Request& packet) override;
  void sendRMIPacketNonBlocking(JointRelativeJRepPacket::Request& packet) override;

  void sendRMIPacketNonBlocking(LinearMotionPacket::Request& packet) override;
  void sendRMIPacketNonBlocking(LinearMotionJRepPacket::Request& packet) override;
  void sendRMIPacketNonBlocking(LinearRelativePacket::Request& packet) override;
  void sendRMIPacketNonBlocking(LinearRelativeJRepPacket::Request& packet) override;

  void sendRMIPacketNonBlocking(CircularMotionPacket::Request& packet) override;
  void sendRMIPacketNonBlocking(CircularRelativePacket::Request& packet) override;

  void sendRMIPacketNonBlocking(SplineMotionPacket::Request& packet) override;
  void sendRMIPacketNonBlocking(SplineMotionJRepPacket::Request& packet) override;

  void sendRMIPacketNonBlocking(WaitForDINPacket::Request& packet) override;
  void sendRMIPacketNonBlocking(SetUFramePacket::Request& packet) override;
  void sendRMIPacketNonBlocking(SetToolFramePacket::Request& packet) override;
  void sendRMIPacketNonBlocking(WaitForTimePacket::Request& packet) override;
  void sendRMIPacketNonBlocking(SetPayloadInstructionPacket::Request& packet) override;

  std::string getErrorMessageString(const uint32_t error_code) override;

  std::optional<SystemFaultPacket> checkSystemFault() override;

  std::optional<TimeoutTerminatePacket> checkTimeoutTerminate() override;

  std::optional<CommunicationPacket> checkCommunicationPacket() override;

  std::optional<UnknownPacket> checkUnknownPacket() override;

  std::optional<InstructionResponse> getLastInstructionResponse() override;

  int32_t getRemainingBuffuerSize() override;

  void setEncoding(const std::string& encoding) override;

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
  bool processInstructionResponse(std::string& json_response);
  template <typename T>
  std::optional<T> checkForPacketInJSONResponses(std::list<std::string>::iterator& it,
                                                 std::list<std::string>& json_responses);

  template <typename T>
  void sendRMIPacketNonBlockingImpl(typename T::Request& request_packet);

  const std::string robot_ip_address_;
  const uint16_t rmi_port_;

  std::string encoding_;
  int32_t sequence_number_;
  std::list<std::string> json_responses_;
  mutable std::mutex mutex_;
  mutable std::mutex motion_mutex_;

  const std::unique_ptr<PConnectionImpl> connection_impl_;

  std::optional<InstructionResponse> last_instruction_response_;
  mutable std::mutex instruction_mutex_;
};

}  // namespace rmi
// TODO: Add doc comments.
