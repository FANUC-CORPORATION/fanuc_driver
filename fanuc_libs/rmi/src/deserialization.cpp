// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "rmi/serialization.hpp"

#include "rfl/json.hpp"
#include "rmi/packets.hpp"

namespace rmi
{
template <typename T>
std::optional<T> FromJSON(const std::string& json)
{
  const auto result = rfl::json::read<T>(json);
  bool valid_result = false;
  if constexpr (requires(T t) { t.Command; })
  {
    valid_result = result.has_value() && result.value().Command == T().Command;
  }
  if constexpr (requires(T t) { t.Communication; })
  {
    valid_result = result.has_value() && result.value().Communication == T().Communication;
  }
  if constexpr (requires(T t) { t.Instruction; })
  {
    valid_result = result.has_value() && result.value().Instruction == T().Instruction;
  }

  return valid_result ? std::optional<T>(result.value()) : std::nullopt;
}

template std::optional<WriteDigitalOutputPacket::Response>
FromJSON<WriteDigitalOutputPacket::Response>(const std::string&);
template std::optional<ConnectPacket::Response> FromJSON<ConnectPacket::Response>(const std::string&);
template std::optional<DisconnectPacket::Response> FromJSON<DisconnectPacket::Response>(const std::string&);
template std::optional<InitializePacket::Response> FromJSON<InitializePacket::Response>(const std::string&);
template std::optional<ResetRobotPacket::Response> FromJSON<ResetRobotPacket::Response>(const std::string&);
template std::optional<StatusRequestPacket::Response> FromJSON<StatusRequestPacket::Response>(const std::string&);
template std::optional<SetSpeedOverridePacket::Response> FromJSON<SetSpeedOverridePacket::Response>(const std::string&);
template std::optional<ProgramCallPacket::Response> FromJSON<ProgramCallPacket::Response>(const std::string&);
template std::optional<AbortPacket::Response> FromJSON<AbortPacket::Response>(const std::string&);
template std::optional<PausePacket::Response> FromJSON<PausePacket::Response>(const std::string&);
template std::optional<ContinuePacket::Response> FromJSON<ContinuePacket::Response>(const std::string&);
template std::optional<ReadErrorPacket::Response> FromJSON<ReadErrorPacket::Response>(const std::string&);
template std::optional<WritePositionRegisterPacket::Response>
FromJSON<WritePositionRegisterPacket::Response>(const std::string&);
template std::optional<ReadPositionRegisterPacket::Response>
FromJSON<ReadPositionRegisterPacket::Response>(const std::string&);
template std::optional<ReadDigitalInputPortPacket::Response>
FromJSON<ReadDigitalInputPortPacket::Response>(const std::string&);
template std::optional<SetPayloadPacket::Response> FromJSON<SetPayloadPacket::Response>(const std::string&);
template std::optional<SetPayloadValuePacket::Response> FromJSON<SetPayloadValuePacket::Response>(const std::string&);
template std::optional<SetPayloadCompPacket::Response> FromJSON<SetPayloadCompPacket::Response>(const std::string&);
template std::optional<ReadNumericRegisterPacket::Response>
FromJSON<ReadNumericRegisterPacket::Response>(const std::string&);
template std::optional<WriteNumericRegisterPacket::Response>
FromJSON<WriteNumericRegisterPacket::Response>(const std::string&);
template std::optional<ReadIOPortPacket::Response> FromJSON<ReadIOPortPacket::Response>(const std::string&);
template std::optional<WriteIOPortPacket::Response> FromJSON<WriteIOPortPacket::Response>(const std::string&);
template std::optional<ReadVariablePacket::Response> FromJSON<ReadVariablePacket::Response>(const std::string&);
template std::optional<WriteVariablePacket::Response> FromJSON<WriteVariablePacket::Response>(const std::string&);
template std::optional<GetExtendedStatusPacket::Response>
FromJSON<GetExtendedStatusPacket::Response>(const std::string&);
template std::optional<LinearRelativeJRepPacket::Response>
FromJSON<LinearRelativeJRepPacket::Response>(const std::string&);
template std::optional<LinearRelativePacket::Response> FromJSON<LinearRelativePacket::Response>(const std::string&);
template std::optional<ReadJointAnglesPacket::Response> FromJSON<ReadJointAnglesPacket::Response>(const std::string&);
template std::optional<ReadUFrameDataPacket::Response> FromJSON<ReadUFrameDataPacket::Response>(const std::string&);
template std::optional<JointMotionJRepPacket::Response> FromJSON<JointMotionJRepPacket::Response>(const std::string&);
template std::optional<CircularRelativePacket::Response> FromJSON<CircularRelativePacket::Response>(const std::string&);
template std::optional<LinearMotionJRepPacket::Response> FromJSON<LinearMotionJRepPacket::Response>(const std::string&);
template std::optional<WaitForDINPacket::Response> FromJSON<WaitForDINPacket::Response>(const std::string&);
template std::optional<CircularMotionPacket::Response> FromJSON<CircularMotionPacket::Response>(const std::string&);
template std::optional<SetToolFramePacket::Response> FromJSON<SetToolFramePacket::Response>(const std::string&);
template std::optional<SetUFramePacket::Response> FromJSON<SetUFramePacket::Response>(const std::string&);
template std::optional<JointRelativeJRepPacket::Response>
FromJSON<JointRelativeJRepPacket::Response>(const std::string&);
template std::optional<SplineMotionJRepPacket::Response> FromJSON<SplineMotionJRepPacket::Response>(const std::string&);
template std::optional<GetTCPSpeedPacket::Response> FromJSON<GetTCPSpeedPacket::Response>(const std::string&);
template std::optional<WaitForTimePacket::Response> FromJSON<WaitForTimePacket::Response>(const std::string&);
template std::optional<JointMotionPacket::Response> FromJSON<JointMotionPacket::Response>(const std::string&);
template std::optional<JointRelativePacket::Response> FromJSON<JointRelativePacket::Response>(const std::string&);
template std::optional<SplineMotionPacket::Response> FromJSON<SplineMotionPacket::Response>(const std::string&);
template std::optional<WriteUToolDataPacket::Response> FromJSON<WriteUToolDataPacket::Response>(const std::string&);
template std::optional<LinearMotionPacket::Response> FromJSON<LinearMotionPacket::Response>(const std::string&);
template std::optional<ReadUToolDataPacket::Response> FromJSON<ReadUToolDataPacket::Response>(const std::string&);
template std::optional<GetUFrameToolFramePacket::Response>
FromJSON<GetUFrameToolFramePacket::Response>(const std::string&);
template std::optional<WriteUFrameDataPacket::Response> FromJSON<WriteUFrameDataPacket::Response>(const std::string&);
template std::optional<SetUFrameToolFramePacket::Response>
FromJSON<SetUFrameToolFramePacket::Response>(const std::string&);
template std::optional<ConnectROS2Packet::Response> FromJSON<ConnectROS2Packet::Response>(const std::string&);

template std::optional<CommunicationPacket> FromJSON<CommunicationPacket>(const std::string&);
template std::optional<SystemFaultPacket> FromJSON<SystemFaultPacket>(const std::string&);
template std::optional<TimeoutTerminatePacket> FromJSON<TimeoutTerminatePacket>(const std::string&);
template std::optional<UnknownPacket> FromJSON<UnknownPacket>(const std::string&);

}  // namespace rmi
