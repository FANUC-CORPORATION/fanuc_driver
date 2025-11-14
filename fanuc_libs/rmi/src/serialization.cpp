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
std::string ToJSON(const T& data)
{
  return rfl::json::write(data);
}

template std::string ToJSON<ConnectPacket::Request>(const ConnectPacket::Request&);
template std::string ToJSON<DisconnectPacket::Request>(const DisconnectPacket::Request&);
template std::string ToJSON<InitializePacket::Request>(const InitializePacket::Request&);
template std::string ToJSON<ResetRobotPacket::Request>(const ResetRobotPacket::Request&);
template std::string ToJSON<StatusRequestPacket::Request>(const StatusRequestPacket::Request&);
template std::string ToJSON<SetSpeedOverridePacket::Request>(const SetSpeedOverridePacket::Request&);
template std::string ToJSON<ProgramCallPacket::Request>(const ProgramCallPacket::Request&);
template std::string ToJSON<AbortPacket::Request>(const AbortPacket::Request&);
template std::string ToJSON<PausePacket::Request>(const PausePacket::Request&);
template std::string ToJSON<ContinuePacket::Request>(const ContinuePacket::Request&);
template std::string ToJSON<ReadErrorPacket::Request>(const ReadErrorPacket::Request&);
template std::string ToJSON<WritePositionRegisterPacket::Request>(const WritePositionRegisterPacket::Request&);
template std::string ToJSON<ReadPositionRegisterPacket::Request>(const ReadPositionRegisterPacket::Request&);
template std::string ToJSON<ReadDigitalInputPortPacket::Request>(const ReadDigitalInputPortPacket::Request&);
template std::string ToJSON<WriteDigitalOutputPacket::Request>(const WriteDigitalOutputPacket::Request&);
template std::string ToJSON<SetPayloadPacket::Request>(const SetPayloadPacket::Request&);
template std::string ToJSON<SetPayloadValuePacket::Request>(const SetPayloadValuePacket::Request&);
template std::string ToJSON<SetPayloadCompPacket::Request>(const SetPayloadCompPacket::Request&);
template std::string ToJSON<ReadNumericRegisterPacket::Request>(const ReadNumericRegisterPacket::Request&);
template std::string ToJSON<WriteNumericRegisterPacket::Request>(const WriteNumericRegisterPacket::Request&);
template std::string ToJSON<WriteIOPortPacket::Request>(const WriteIOPortPacket::Request&);
template std::string ToJSON<ReadIOPortPacket::Request>(const ReadIOPortPacket::Request&);
template std::string ToJSON<ReadVariablePacket::Request>(const ReadVariablePacket::Request&);
template std::string ToJSON<WriteVariablePacket::Request>(const WriteVariablePacket::Request&);
template std::string ToJSON<GetExtendedStatusPacket::Request>(const GetExtendedStatusPacket::Request&);
template std::string ToJSON<WaitForDINPacket::Request>(const WaitForDINPacket::Request&);
template std::string ToJSON<LinearMotionJRepPacket::Request>(const LinearMotionJRepPacket::Request&);
template std::string ToJSON<SetToolFramePacket::Request>(const SetToolFramePacket::Request&);
template std::string ToJSON<JointMotionPacket::Request>(const JointMotionPacket::Request&);
template std::string ToJSON<SplineMotionJRepPacket::Request>(const SplineMotionJRepPacket::Request&);
template std::string ToJSON<WaitForTimePacket::Request>(const WaitForTimePacket::Request&);
template std::string ToJSON<SetUFramePacket::Request>(const SetUFramePacket::Request&);
template std::string ToJSON<ReadUToolDataPacket::Request>(const ReadUToolDataPacket::Request&);
template std::string ToJSON<ReadUFrameDataPacket::Request>(const ReadUFrameDataPacket::Request&);
template std::string ToJSON<JointMotionJRepPacket::Request>(const JointMotionJRepPacket::Request&);
template std::string ToJSON<JointRelativeJRepPacket::Request>(const JointRelativeJRepPacket::Request&);
template std::string ToJSON<SetUFrameToolFramePacket::Request>(const SetUFrameToolFramePacket::Request&);
template std::string ToJSON<LinearRelativeJRepPacket::Request>(const LinearRelativeJRepPacket::Request&);
template std::string ToJSON<WriteUToolDataPacket::Request>(const WriteUToolDataPacket::Request&);
template std::string ToJSON<SplineMotionPacket::Request>(const SplineMotionPacket::Request&);
template std::string ToJSON<CircularMotionPacket::Request>(const CircularMotionPacket::Request&);
template std::string ToJSON<GetTCPSpeedPacket::Request>(const GetTCPSpeedPacket::Request&);
template std::string ToJSON<WriteUFrameDataPacket::Request>(const WriteUFrameDataPacket::Request&);
template std::string ToJSON<LinearMotionPacket::Request>(const LinearMotionPacket::Request&);
template std::string ToJSON<ReadJointAnglesPacket::Request>(const ReadJointAnglesPacket::Request&);
template std::string ToJSON<CircularRelativePacket::Request>(const CircularRelativePacket::Request&);
template std::string ToJSON<GetUFrameToolFramePacket::Request>(const GetUFrameToolFramePacket::Request&);
template std::string ToJSON<JointRelativePacket::Request>(const JointRelativePacket::Request&);
template std::string ToJSON<LinearRelativePacket::Request>(const LinearRelativePacket::Request&);
template std::string ToJSON<ConnectROS2Packet::Request>(const ConnectROS2Packet::Request&);

}  // namespace rmi
