// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "rmi/rmi.hpp"
#include "rmi/serialization.hpp"

#include <array>
#include <iostream>
#include <mutex>
#include <sstream>

#include "rmi/packets.hpp"
#include "sockpp/inet_address.h"
#include "sockpp/tcp_connector.h"

namespace rmi
{

namespace
{
constexpr auto kEndChars = "\r\n";

constexpr std::array<std::pair<uint32_t, std::string_view>, 55> kErrorCodes = {
  { { 2556929, "Internal System Error." },
    { 2556930, "Invalid UTool Number." },
    { 2556931, "Invalid UFrame Number." },
    { 2556932, "Invalid Position Register." },
    { 2556933, "Invalid Speed Override." },
    { 2556934, "Cannot Execute TP program." },
    { 2556935, "Controller Servo is Off." },
    { 2556936, "Cannot Execute TP program." },
    { 2556937, "RMI is Not Running." },
    { 2556938, "TP Program is Not Paused." },
    { 2556939, "Cannot Resume TP Program." },
    { 2556940, "Cannot Reset Controller." },
    { 2556941, "Invalid RMI Command." },
    { 2556942, "RMI Command Fail." },
    { 2556943, "Invalid Controller State." },
    { 2556944, "Please Cycle Power." },
    { 2556945, "Invalid Payload Schedule." },
    { 2556946, "Invalid Motion Option." },
    { 2556947, "Invalid Vision Register." },
    { 2556948, "Invalid RMI Instruction." },
    { 2556949, "Invalid Value." },
    { 2556950, "Invalid Text String" },
    { 2556951, "Invalid Position Data" },
    { 2556952, "RMI is In HOLD State" },
    { 2556953, "Remote Device Disconnected." },
    { 2556954, "Robot is Already Connected." },
    { 2556955, "Wait for Command Done." },
    { 2556956, "Wait for Instruction Done." },
    { 2556957, "Invalid sequence ID number." },
    { 2556958, "Invalid Speed Type." },
    { 2556959, "Invalid Speed Value." },
    { 2556960, "Invalid Term Type." },
    { 2556961, "Invalid Term Value." },
    { 2556962, "Invalid LCB Port Type." },
    { 2556963, "Invalid ACC Value." },
    { 2556964, "Invalid Destination Position" },
    { 2556965, "Invalid VIA Position." },
    { 2556966, "Invalid Port Number." },
    { 2556967, "Invalid Group Number" },
    { 2556968, "Invalid Group Mask" },
    { 2556969, "Joint motion with COORD" },
    { 2556970, "Incremental motn with COORD" },
    { 2556971, "Robot in Single Step Mode" },
    { 2556972, "Invalid Position Data Type" },
    { 2556973, "Ready for ASCII Packet" },
    { 2556974, "ASCII Conversion Failed" },
    { 2556975, "Invalid ASCII Instruction" },
    { 2556976, "Invalid Number of Groups" },
    { 2556977, "Invalid Instruction packet" },
    { 2556978, "Invalid ASCII String packet" },
    { 2556979, "Invalid ASCII string size" },
    { 2556980, "Invalid Application Tool" },
    { 2556981, "Invalid Call Program Name" },
    { 7015, "RMI_MOVE is the selected TP program. Select a different TP program and re-run RMI PC code." },
    { 7004, "The specific program is in use." } }
};

std::string LookupErrorCode(const uint32_t error_code)
{
  if (const auto it = std::ranges::find_if(kErrorCodes,
                                           [error_code](const std::pair<uint32_t, std::string_view>& error) {
                                             return error.first == error_code;
                                           });
      it != kErrorCodes.end())
  {
    return std::string(it->second);
  }
  return "Unknown Error Code: " + std::to_string(error_code);
}

template <typename T>
std::optional<T> CheckForPacketInJSONResponses(std::list<std::string>::iterator& it,
                                               std::list<std::string>& json_responses)
{
  std::optional<T> packet_response;
  for (; it != json_responses.end(); ++it)
  {
    if (auto packet_response_maybe = FromJSON<T>(*it); packet_response_maybe.has_value())
    {
      packet_response = packet_response_maybe.value();
      it = json_responses.erase(it);
      break;
    }
  }
  return packet_response;
}

template <typename T>
bool CheckSequenceIDIfPresent(T& packet, int expected_sequence_id)
{
  if constexpr (requires(typename T::Response t) { t.SequenceID; })
  {
    return packet.SequenceID == expected_sequence_id;
  }
  return true;
}

}  // namespace

struct RMIConnection::PConnectionImpl
{
  PConnectionImpl(const std::string& robot_ip_address, const uint16_t robot_port)
  {
    createNewConnection(robot_ip_address, robot_port);
  }

  template <typename T>
  void write(const T& value)
  {
    const std::string json_string = ToJSON(value) + kEndChars;
    std::scoped_lock lock(mutex_);
    conn.write(json_string);
  }

  void createNewConnection(const std::string& robot_ip_address, const uint16_t robot_port)
  {
    const sockpp::inet_address server_address(robot_ip_address, robot_port);
    std::scoped_lock lock(mutex_);
    if (!conn.connect(server_address))
    {
      throw std::runtime_error("Failed to create TCP connection at: " + robot_ip_address);
    }
    conn.set_non_blocking(true);
  }

  std::vector<std::string> read()
  {
    char buf[2048];
    mutex_.lock();
    const sockpp::result<size_t> res = conn.read(buf, sizeof(buf));
    mutex_.unlock();

    std::vector<std::string> json_responses;
    std::string_view buf_view(buf, res.value());
    size_t pos = 0;
    while (pos < buf_view.size())
    {
      const size_t end_pos = buf_view.find(kEndChars, pos);
      if (end_pos == std::string_view::npos)
      {
        break;
      }
      json_responses.emplace_back(buf_view.substr(pos, end_pos - pos));
      pos = end_pos + strlen(kEndChars);
    }

    return json_responses;
  }

  sockpp::tcp_connector conn;

private:
  mutable std::mutex mutex_;
};

RMIConnection::RMIConnection(const std::string& robot_ip_address, const uint16_t rmi_port)
  : RMIConnectionInterface()
  , robot_ip_address_{ robot_ip_address }
  , rmi_port_{ rmi_port }
  , sequence_number_{ 1 }
  , connection_impl_{ std::make_unique<PConnectionImpl>(robot_ip_address, rmi_port) }
{
}

RMIConnection::~RMIConnection() = default;

void RMIConnection::drainConnectionBuffer()
{
  std::vector<std::string> json_responses = connection_impl_->read();
  while (!json_responses.empty())
  {
    {
      std::scoped_lock lock(mutex_);
      json_responses_.insert(json_responses_.end(), json_responses.begin(), json_responses.end());
    }
    json_responses = connection_impl_->read();
  }
}

template <typename T>
std::optional<T> RMIConnection::checkPushPacket()
{
  drainConnectionBuffer();

  std::optional<T> packet;
  auto it = json_responses_.begin();
  auto possible_packet = CheckForPacketInJSONResponses<T>(it, json_responses_);
  while (packet.has_value())
  {
    packet = possible_packet;
    possible_packet = CheckForPacketInJSONResponses<T>(it, json_responses_);
  }

  return packet;
}

std::optional<SystemFaultPacket> RMIConnection::checkSystemFault()
{
  return checkPushPacket<SystemFaultPacket>();
}

std::optional<TimeoutTerminatePacket> RMIConnection::checkTimeoutTerminate()
{
  return checkPushPacket<TimeoutTerminatePacket>();
}

std::optional<CommunicationPacket> RMIConnection::checkCommunicationPacket()
{
  return checkPushPacket<CommunicationPacket>();
}

std::optional<UnknownPacket> RMIConnection::checkUnknownPacket()
{
  return checkPushPacket<UnknownPacket>();
}

template <typename T>
T RMIConnection::getResponsePacket(const std::optional<double> timeout_optional,
                                   const std::string& error_message_prefix, std::optional<int> expected_sequence_id)
{
  if constexpr (requires(T t) { t.SequenceID; })
  {
    if (!expected_sequence_id.has_value())
    {
      throw std::logic_error("expected_sequence_id must be provided for the packet type T with the SequenceID field.");
    }
  }

  const double timeout = timeout_optional.has_value() ? timeout_optional.value() : 1.0;
  std::optional<T> packet_response;

  const auto start_time = std::chrono::steady_clock::now();
  while (!packet_response.has_value() ||
         (expected_sequence_id.has_value() &&
          !CheckSequenceIDIfPresent(packet_response.value(), expected_sequence_id.value())))
  {
    if (const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time);
        timeout >= 0.0 && elapsed.count() >= timeout)
    {
      std::stringstream ss;
      ss << error_message_prefix << "Timeout waiting for response. ";
      std::scoped_lock lock(mutex_);
      if (!json_responses_.empty())
      {
        ss << " The current unhandled json response packets are:\n";
        for (const auto& json_response : json_responses_)
        {
          ss << json_response << "\n";
        }
        ss << "You may have incompatible packet definitions for the current RMI version.";
      }
      throw std::runtime_error(ss.str());
    }

    drainConnectionBuffer();
    std::scoped_lock lock(mutex_);
    if (auto it = json_responses_.begin(); !json_responses_.empty())
    {
      packet_response = CheckForPacketInJSONResponses<T>(it, json_responses_);
    }
  }

  if (packet_response.value().ErrorID != 0)
  {
    throw std::runtime_error(error_message_prefix + "Error: " + LookupErrorCode(packet_response.value().ErrorID));
  }

  return packet_response.value();
}

int32_t RMIConnection::getSequenceNumber()
{
  return sequence_number_++;
}

ConnectROS2Packet::Response RMIConnection::connect(const std::optional<double> timeout)
{
  connection_impl_->write(ConnectROS2Packet::Request());
  auto connect_response =
      getResponsePacket<ConnectROS2Packet::Response>(timeout, "Failed to connect to RMI server. ", std::nullopt);
  // Recreate a TCP to the new port number provided by the server
  connection_impl_->createNewConnection(robot_ip_address_, connect_response.PortNumber);
  return connect_response;
}

DisconnectPacket::Response RMIConnection::disconnect(const std::optional<double> timeout)
{
  connection_impl_->write(DisconnectPacket::Request());
  return getResponsePacket<DisconnectPacket::Response>(timeout, "Failed to disconnect from RMI server. ", std::nullopt);
}

InitializePacket::Response RMIConnection::initializeRemoteMotion(const std::optional<double> timeout)
{
  {
    std::scoped_lock lock(mutex_);
    sequence_number_ = 1;  // Reset sequence number for a new session
  }
  connection_impl_->write(InitializePacket::Request());
  return getResponsePacket<InitializePacket::Response>(timeout, "Failed to initialize RMI. ", std::nullopt);
}

ProgramCallPacket::Request RMIConnection::programCallNonBlocking(const std::string& program_name)
{
  ProgramCallPacket::Request program_call_packet;
  program_call_packet.ProgramName = program_name;
  std::scoped_lock lock(motion_mutex_);
  program_call_packet.SequenceID = getSequenceNumber();
  connection_impl_->write(program_call_packet);
  return program_call_packet;
}

ProgramCallPacket::Response RMIConnection::programCall(const std::string& program_name,
                                                       const std::optional<double> timeout)
{
  ProgramCallPacket::Request program_call_packet = programCallNonBlocking(program_name);
  return getResponsePacket<ProgramCallPacket::Response>(
      timeout, "Failed to call program named `" + program_name + "`. ", program_call_packet.SequenceID);
}

SetSpeedOverridePacket::Response RMIConnection::setSpeedOverride(const int value, const std::optional<double> timeout)
{
  SetSpeedOverridePacket::Request set_speed_override_packet;
  set_speed_override_packet.Value = value;
  connection_impl_->write(set_speed_override_packet);
  return getResponsePacket<SetSpeedOverridePacket::Response>(timeout, "Failed to set the override speed. ",
                                                             std::nullopt);
}

StatusRequestPacket::Response RMIConnection::getStatus(const std::optional<double> timeout)
{
  connection_impl_->write(StatusRequestPacket::Request());
  return getResponsePacket<StatusRequestPacket::Response>(timeout, "Failed to get status from RMI server. ",
                                                          std::nullopt);
}

AbortPacket::Response RMIConnection::abort(const std::optional<double> timeout)
{
  connection_impl_->write(AbortPacket::Request());
  return getResponsePacket<AbortPacket::Response>(timeout, "Failed to abort. ", std::nullopt);
}

PausePacket::Response RMIConnection::pause(const std::optional<double> timeout)
{
  connection_impl_->write(PausePacket::Request());
  return getResponsePacket<PausePacket::Response>(timeout, "Failed to pause. ", std::nullopt);
}

ContinuePacket::Response RMIConnection::resume(const std::optional<double> timeout)
{
  connection_impl_->write(ContinuePacket::Request());
  return getResponsePacket<ContinuePacket::Response>(timeout, "Failed to continue. ", std::nullopt);
}

ResetRobotPacket::Response RMIConnection::reset(const std::optional<double> timeout)
{
  connection_impl_->write(ResetRobotPacket::Request());
  return getResponsePacket<ResetRobotPacket::Response>(timeout, "Failed to reset. ", std::nullopt);
}

ReadErrorPacket::Response RMIConnection::readError(const std::optional<double> timeout)
{
  connection_impl_->write(ReadErrorPacket::Request());
  return getResponsePacket<ReadErrorPacket::Response>(timeout, "Failed to read error. ", std::nullopt);
}

WritePositionRegisterPacket::Response
RMIConnection::writePositionRegister(const int register_number, const std::string& representation,
                                     const ConfigurationData& configuration, const PositionData& position,
                                     const JointAngleData& joint_angle, const std::optional<double> timeout)
{
  if (register_number < 1 || register_number > 100)
  {
    throw std::runtime_error("Invalid position register number: " + std::to_string(register_number) +
                             ". Valid range is 1 to 100.");
  }
  WritePositionRegisterPacket::Request write_position_register_packet;
  write_position_register_packet.RegisterNumber = register_number;
  write_position_register_packet.Representation = representation;
  std::string temp_upper_rep = representation;
  std::transform(temp_upper_rep.begin(), temp_upper_rep.end(), temp_upper_rep.begin(), ::toupper);
  if (temp_upper_rep == "JOINT")
  {
    write_position_register_packet.JointAngle = joint_angle;
  }
  else
  {
    write_position_register_packet.Configuration = configuration;
    write_position_register_packet.Position = position;
  }
  connection_impl_->write(write_position_register_packet);
  return getResponsePacket<WritePositionRegisterPacket::Response>(timeout, "Failed to write to position register. ",
                                                                  std::nullopt);
}

ReadPositionRegisterPacket::Response RMIConnection::readPositionRegister(const int register_number,
                                                                         const std::optional<double> timeout)
{
  if (register_number < 1 || register_number > 100)
  {
    throw std::runtime_error("Invalid position register number: " + std::to_string(register_number) +
                             ". Valid range is 1 to 100.");
  }
  ReadPositionRegisterPacket::Request read_position_register_packet;
  read_position_register_packet.RegisterNumber = register_number;
  connection_impl_->write(read_position_register_packet);
  return getResponsePacket<ReadPositionRegisterPacket::Response>(timeout, "Failed to read from position register. ",
                                                                 std::nullopt);
}

ReadNumericRegisterPacket::Response RMIConnection::readNumericRegister(const int register_number,
                                                                       const std::optional<double> timeout)
{
  if (register_number < 1 || register_number > 200)
  {
    throw std::runtime_error("Invalid position register number: " + std::to_string(register_number) +
                             ". Valid range is 1 to 200.");
  }
  ReadNumericRegisterPacket::Request read_numeric_register_packet;
  read_numeric_register_packet.RegisterNumber = register_number;
  connection_impl_->write(read_numeric_register_packet);
  return getResponsePacket<ReadNumericRegisterPacket::Response>(timeout, "Failed to read from numeric register. ",
                                                                std::nullopt);
}

WriteNumericRegisterPacket::Response RMIConnection::writeNumericRegister(const int register_number,
                                                                         const std::variant<int, float> value,
                                                                         const std::optional<double> timeout)
{
  if (register_number < 1 || register_number > 200)
  {
    throw std::runtime_error("Invalid position register number: " + std::to_string(register_number) +
                             ". Valid range is 1 to 200.");
  }
  WriteNumericRegisterPacket::Request write_numeric_register_packet;
  write_numeric_register_packet.RegisterNumber = register_number;
  write_numeric_register_packet.RegisterValue = value;
  write_numeric_register_packet.DataType = std::holds_alternative<float>(value) ? "float" : "integer";
  connection_impl_->write(write_numeric_register_packet);
  return getResponsePacket<WriteNumericRegisterPacket::Response>(timeout, "Failed to write to numeric register. ",
                                                                 std::nullopt);
}

ReadDigitalInputPortPacket::Response RMIConnection::readDigitalInputPort(const uint16_t port_number,
                                                                         const std::optional<double> timeout)
{
  ReadDigitalInputPortPacket::Request read_digital_input_port_packet;
  read_digital_input_port_packet.PortNumber = port_number;
  connection_impl_->write(read_digital_input_port_packet);
  return getResponsePacket<ReadDigitalInputPortPacket::Response>(timeout, "Failed to read digital input port. ",
                                                                 std::nullopt);
}

WriteDigitalOutputPacket::Response RMIConnection::writeDigitalOutputPort(const uint16_t port_number,
                                                                         const bool port_value,
                                                                         const std::optional<double> timeout)
{
  WriteDigitalOutputPacket::Request write_digital_output_packet;
  write_digital_output_packet.PortNumber = port_number;
  write_digital_output_packet.PortValue = port_value ? "ON" : "OFF";
  connection_impl_->write(write_digital_output_packet);
  return getResponsePacket<WriteDigitalOutputPacket::Response>(timeout, "Failed to write to digital output port. ",
                                                               std::nullopt);
}

ReadIOPortPacket::Response RMIConnection::readIOPort(const std::string& port_type, const int port_number,
                                                     const std::optional<double> timeout)
{
  ReadIOPortPacket::Request read_io_port_packet;
  read_io_port_packet.PortNumber = port_number;
  read_io_port_packet.PortType = port_type;
  connection_impl_->write(read_io_port_packet);
  return getResponsePacket<ReadIOPortPacket::Response>(timeout, "Failed to read IO port. ", std::nullopt);
}

WriteIOPortPacket::Response RMIConnection::writeIOPort(const int port_number, const std::string& port_type,
                                                       const std::variant<int, float> port_value,
                                                       const std::optional<double> timeout)
{
  WriteIOPortPacket::Request write_io_port_packet;
  write_io_port_packet.PortNumber = port_number;
  write_io_port_packet.PortType = port_type;
  write_io_port_packet.PortValue = port_value;
  connection_impl_->write(write_io_port_packet);
  return getResponsePacket<WriteIOPortPacket::Response>(timeout, "Failed to read IO port. ", std::nullopt);
}

ReadVariablePacket::Response RMIConnection::readVariablePacket(const std::string& variable_name,
                                                               const std::optional<double> timeout)
{
  ReadVariablePacket::Request read_variable_packet;
  read_variable_packet.VariableName = variable_name;
  connection_impl_->write(read_variable_packet);
  return getResponsePacket<ReadVariablePacket::Response>(timeout, "Failed to read variable `" + variable_name + "`. ",
                                                         std::nullopt);
}

WriteVariablePacket::Response RMIConnection::writeVariablePacket(const std::string& variable_name,
                                                                 const std::variant<int, float> value,
                                                                 const std::optional<double> timeout)
{
  WriteVariablePacket::Request write_variable_packet;
  write_variable_packet.VariableName = variable_name;
  write_variable_packet.VariableValue = value;
  write_variable_packet.VariableType = std::holds_alternative<float>(value) ? "float" : "integer";
  connection_impl_->write(write_variable_packet);
  return getResponsePacket<WriteVariablePacket::Response>(timeout, "Failed to write variable `" + variable_name + "`. ",
                                                          std::nullopt);
}

GetExtendedStatusPacket::Response RMIConnection::getExtendedStatus(const std::optional<double> timeout)
{
  connection_impl_->write(GetExtendedStatusPacket::Request());
  return getResponsePacket<GetExtendedStatusPacket::Response>(timeout, "Failed to get extended status. ", std::nullopt);
}

SetPayloadPacket::Response RMIConnection::setPayloadSchedule(const uint8_t payload_schedule_number,
                                                             const std::optional<double> timeout)
{
  SetPayloadPacket::Request set_payload_packet;
  set_payload_packet.ScheduleNumber = payload_schedule_number;
  std::scoped_lock lock(motion_mutex_);
  connection_impl_->write(set_payload_packet);
  return getResponsePacket<SetPayloadPacket::Response>(timeout, "Failed to set the payload schedule. ", std::nullopt);
}

SetPayloadValuePacket::Response RMIConnection::setPayloadValue(const uint8_t payload_schedule_number, const float mass,
                                                               const float cg_x, const float cg_y, const float cg_z,
                                                               const bool use_in, const float in_x, const float in_y,
                                                               const float in_z, const std::optional<double> timeout)
{
  SetPayloadValuePacket::Request set_payload_value_packet;
  set_payload_value_packet.ScheduleNumber = payload_schedule_number;
  set_payload_value_packet.Mass = mass;
  set_payload_value_packet.CG_X = cg_x * 100;  // unit conversion: m -> cm (rmi)
  set_payload_value_packet.CG_Y = cg_y * 100;  // unit conversion: m -> cm (rmi)
  set_payload_value_packet.CG_Z = cg_z * 100;  // unit conversion: m -> cm (rmi)
  if (use_in)
  {
    set_payload_value_packet.IN_X = in_x * 10000;  // unit conversion: kgm^2 -> kgcm^2 (rmi)
    set_payload_value_packet.IN_Y = in_y * 10000;  // unit conversion: kgm^2 -> kgcm^2 (rmi)
    set_payload_value_packet.IN_Z = in_z * 10000;  // unit conversion: kgm^2 -> kgcm^2 (rmi)
  }
  std::scoped_lock lock(motion_mutex_);
  connection_impl_->write(set_payload_value_packet);
  return getResponsePacket<SetPayloadValuePacket::Response>(timeout, "Failed to set the payload value. ", std::nullopt);
}

SetPayloadCompPacket::Response RMIConnection::setPayloadComp(const uint8_t payload_schedule_number, const float mass,
                                                             const float cg_x, const float cg_y, const float cg_z,
                                                             const float in_x, const float in_y, const float in_z,
                                                             const std::optional<double> timeout)
{
  SetPayloadCompPacket::Request set_payload_comp_packet;
  set_payload_comp_packet.ScheduleNumber = payload_schedule_number;
  set_payload_comp_packet.Mass = mass;
  set_payload_comp_packet.CG_X = cg_x * 100;    // unit conversion: m -> cm (rmi)
  set_payload_comp_packet.CG_Y = cg_y * 100;    // unit conversion: m -> cm (rmi)
  set_payload_comp_packet.CG_Z = cg_z * 100;    // unit conversion: m -> cm (rmi)
  set_payload_comp_packet.IN_X = in_x * 10000;  // unit conversion: kgm^2 -> kgcm^2 (rmi)
  set_payload_comp_packet.IN_Y = in_y * 10000;  // unit conversion: kgm^2 -> kgcm^2 (rmi)
  set_payload_comp_packet.IN_Z = in_z * 10000;  // unit conversion: kgm^2 -> kgcm^2 (rmi)
  std::scoped_lock lock(motion_mutex_);
  connection_impl_->write(set_payload_comp_packet);
  return getResponsePacket<SetPayloadCompPacket::Response>(timeout, "Failed to set the payload compensation. ",
                                                           std::nullopt);
}

ReadJointAnglesPacket::Response RMIConnection::readJointAngles(const std::optional<uint8_t>& group,
                                                               const std::optional<double> timeout)
{
  ReadJointAnglesPacket::Request read_joint_angles_packet;
  read_joint_angles_packet.Group = group;
  connection_impl_->write(read_joint_angles_packet);
  return getResponsePacket<ReadJointAnglesPacket::Response>(timeout, "Failed to set the payload schedule. ",
                                                            std::nullopt);
}

JointMotionJRepPacket::Response RMIConnection::sendJointMotion(JointMotionJRepPacket::Request joint_motion_request,
                                                               const std::optional<double> timeout)
{
  joint_motion_request.SequenceID = getSequenceNumber();
  connection_impl_->write(joint_motion_request);
  return getResponsePacket<JointMotionJRepPacket::Response>(timeout, "Failed to set the payload schedule. ",
                                                            joint_motion_request.SequenceID);
}

template <typename T>
typename T::Response RMIConnection::sendRMIPacket(typename T::Request& request_packet,
                                                  const std::optional<double> timeout)
{
  if constexpr (requires(typename T::Response t) { t.SequenceID; })
  {
    std::scoped_lock lock(motion_mutex_);
    request_packet.SequenceID = getSequenceNumber();
    connection_impl_->write(request_packet);
    return getResponsePacket<typename T::Response>(timeout, "Failed to send packet. ", request_packet.SequenceID);
  }

  connection_impl_->write(request_packet);
  return getResponsePacket<typename T::Response>(timeout, "Failed to send packet. ", std::nullopt);
}

template SetUFrameToolFramePacket::Response
RMIConnection::sendRMIPacket<SetUFrameToolFramePacket>(SetUFrameToolFramePacket::Request&, std::optional<double>);
template ReadUFrameDataPacket::Response
RMIConnection::sendRMIPacket<ReadUFrameDataPacket>(ReadUFrameDataPacket::Request&, std::optional<double>);
template WriteUFrameDataPacket::Response
RMIConnection::sendRMIPacket<WriteUFrameDataPacket>(WriteUFrameDataPacket::Request&, std::optional<double>);
template ReadUToolDataPacket::Response RMIConnection::sendRMIPacket<ReadUToolDataPacket>(ReadUToolDataPacket::Request&,
                                                                                         std::optional<double>);
template WriteUToolDataPacket::Response
RMIConnection::sendRMIPacket<WriteUToolDataPacket>(WriteUToolDataPacket::Request&, std::optional<double>);
template WriteDigitalOutputPacket::Response
RMIConnection::sendRMIPacket<WriteDigitalOutputPacket>(WriteDigitalOutputPacket::Request&, std::optional<double>);
template ReadJointAnglesPacket::Response
RMIConnection::sendRMIPacket<ReadJointAnglesPacket>(ReadJointAnglesPacket::Request&, std::optional<double>);
template GetUFrameToolFramePacket::Response
RMIConnection::sendRMIPacket<GetUFrameToolFramePacket>(GetUFrameToolFramePacket::Request&, std::optional<double>);
template GetTCPSpeedPacket::Response RMIConnection::sendRMIPacket<GetTCPSpeedPacket>(GetTCPSpeedPacket::Request&,
                                                                                     std::optional<double>);
template WaitForDINPacket::Response RMIConnection::sendRMIPacket<WaitForDINPacket>(WaitForDINPacket::Request&,
                                                                                   std::optional<double>);
template SetUFramePacket::Response RMIConnection::sendRMIPacket<SetUFramePacket>(SetUFramePacket::Request&,
                                                                                 std::optional<double>);
template SetToolFramePacket::Response RMIConnection::sendRMIPacket<SetToolFramePacket>(SetToolFramePacket::Request&,
                                                                                       std::optional<double>);
template WaitForTimePacket::Response RMIConnection::sendRMIPacket<WaitForTimePacket>(WaitForTimePacket::Request&,
                                                                                     std::optional<double>);
template LinearMotionPacket::Response RMIConnection::sendRMIPacket<LinearMotionPacket>(LinearMotionPacket::Request&,
                                                                                       std::optional<double>);
template LinearRelativePacket::Response
RMIConnection::sendRMIPacket<LinearRelativePacket>(LinearRelativePacket::Request&, std::optional<double>);
template JointMotionPacket::Response RMIConnection::sendRMIPacket<JointMotionPacket>(JointMotionPacket::Request&,
                                                                                     std::optional<double>);
template JointRelativePacket::Response RMIConnection::sendRMIPacket<JointRelativePacket>(JointRelativePacket::Request&,
                                                                                         std::optional<double>);
template CircularMotionPacket::Response
RMIConnection::sendRMIPacket<CircularMotionPacket>(CircularMotionPacket::Request&, std::optional<double>);
template CircularRelativePacket::Response
RMIConnection::sendRMIPacket<CircularRelativePacket>(CircularRelativePacket::Request&, std::optional<double>);
template JointMotionJRepPacket::Response
RMIConnection::sendRMIPacket<JointMotionJRepPacket>(JointMotionJRepPacket::Request&, std::optional<double>);
template JointRelativeJRepPacket::Response
RMIConnection::sendRMIPacket<JointRelativeJRepPacket>(JointRelativeJRepPacket::Request&, std::optional<double>);
template LinearMotionJRepPacket::Response
RMIConnection::sendRMIPacket<LinearMotionJRepPacket>(LinearMotionJRepPacket::Request&, std::optional<double>);
template LinearRelativeJRepPacket::Response
RMIConnection::sendRMIPacket<LinearRelativeJRepPacket>(LinearRelativeJRepPacket::Request&, std::optional<double>);
template SplineMotionPacket::Response RMIConnection::sendRMIPacket<SplineMotionPacket>(SplineMotionPacket::Request&,
                                                                                       std::optional<double>);
template SplineMotionJRepPacket::Response
RMIConnection::sendRMIPacket<SplineMotionJRepPacket>(SplineMotionJRepPacket::Request&, std::optional<double>);
template ConnectROS2Packet::Response RMIConnection::sendRMIPacket<ConnectROS2Packet>(ConnectROS2Packet::Request&,
                                                                                     std::optional<double>);

}  // namespace rmi
