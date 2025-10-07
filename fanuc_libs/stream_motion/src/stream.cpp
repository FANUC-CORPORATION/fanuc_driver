// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "stream_motion/stream.hpp"

#include <cmath>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <vector>

#include "sockpp/inet_address.h"
#include "sockpp/udp_socket.h"
#include "stream_motion/byte_ops.hpp"
#include "stream_motion/packets.hpp"

namespace stream_motion
{
namespace
{
constexpr int32_t kVersion = 2;
constexpr int32_t kStopPacketType = 2;
constexpr int32_t kStartPacketType = 200;
constexpr uint32_t kJerkLimitCommandPacketType = 201;
constexpr uint16_t kCommandPacketUnused = 0xFFFF;
constexpr int kThresholdPayloadLength = 20;

bool IsReadGPIOConfig(const GPIOControlConfig& config)
{
  return config.command_type == GPIOCommandType::IOState || config.command_type == GPIOCommandType::NumRegState;
}

bool IsWriteGPIOConfig(const GPIOControlConfig& config)
{
  return config.command_type == GPIOCommandType::IOCmd || config.command_type == GPIOCommandType::NumRegCmd;
}

void CheckOverlapping(const std::vector<GPIOControlConfig>& gpio_config)
{
  std::unordered_map<uint64_t, std::vector<std::reference_wrapper<const GPIOControlConfig>>> configs_by_type;
  for (const auto& config : gpio_config)
  {
    const uint64_t hash = (static_cast<uint64_t>(config.command_type) << 32) | config.gpio_type;
    configs_by_type[hash].push_back(config);
  }

  for (auto& [_, sorted_gpio_config] : configs_by_type)
  {
    std::sort(sorted_gpio_config.begin(), sorted_gpio_config.end(),
              [](const GPIOControlConfig& a, const GPIOControlConfig& b) { return a.start < b.start; });
    for (size_t i = 0; i + 1 < sorted_gpio_config.size(); ++i)
    {
      const auto& current = sorted_gpio_config[i];
      if (const auto& next = sorted_gpio_config[i + 1]; current.get().start + current.get().length > next.get().start)
      {
        throw std::invalid_argument("The GPIO configurations has overlapping ranges which is not supported.");
      }
    }
  }
}

// Returns the number of GPIO values that can be packed in 4 bytes based on their type.
int32_t CalculateNumPackedValues(const GPIOControlConfig& gpio_config)
{
  switch (gpio_config.command_type)
  {
    case GPIOCommandType::IOState:
      [[fallthrough]];
    case GPIOCommandType::IOCmd:
    {
      if (gpio_config.gpio_type == static_cast<uint32_t>(IOType::AO) ||
          gpio_config.gpio_type == static_cast<uint32_t>(IOType::AI))
      {
        return 2;
      }
      return 32;
    }
    case GPIOCommandType::NumRegState:
      [[fallthrough]];
    case GPIOCommandType::NumRegCmd:
      return 1;  // For numeric registers, we assume a float size of 4 bytes
    default:
      return 0;  // No bits for None command type
  }
}

// Returns the number of bytes needed for a config with 4 byte alignment.
int32_t CalculateNumBytesConfig(const GPIOControlConfig& gpio_config)
{
  const int32_t denominator = CalculateNumPackedValues(gpio_config);
  if (denominator == 0)
  {
    return 0;
  }
  return std::max(static_cast<int32_t>(1 + (gpio_config.length - 1) / denominator) * 4, 4);
}
}  // namespace

struct StreamMotionConnection::PSocketImpl
{
  PSocketImpl(const std::string& robot_ip_address, const uint16_t robot_port, const double timeout)
    : server_address{ robot_ip_address, robot_port }, timeout{ timeout }
  {
    sock.connect(server_address);
    sock.set_non_blocking(true);
    std::cout << "Created UDP socket at: " << sock.address() << std::endl;
  }

  template <typename T>
  bool send(const T& value)
  {
    const auto buf = reinterpret_cast<const void*>(&value);
    constexpr size_t kPacketNumBytes = sizeof(T);

    if (const auto res = sock.send(buf, kPacketNumBytes); res != kPacketNumBytes)
    {
      std::cerr << "Error writing to the UDP socket: " << res.error_message() << std::endl;
      return false;
    }
    return true;
  }

  template <typename T>
  bool receive(T& value)
  {
    // Clear the status packet before receiving new data
    value = T();

    void* buf = &value;
    const auto start_time = std::chrono::steady_clock::now();
    while (true)
    {
      constexpr size_t kPacketNumBytes = sizeof(T);
      sockpp::result<size_t> res = sock.recv(buf, kPacketNumBytes);
      if (res != kPacketNumBytes &&
          std::chrono::steady_clock::now() - start_time > std::chrono::duration<double>(timeout))
      {
        std::cerr << "Timeout while reading from UDP socket." << std::endl;
        return false;
      }
      if (res == kPacketNumBytes)
      {
        break;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    return true;
  }

  sockpp::udp_socket sock;
  sockpp::inet_address server_address;
  double timeout;
};

StreamMotionConnection::~StreamMotionConnection() = default;

StreamMotionConnection::StreamMotionConnection(const std::string& robot_ip_address, const double timeout,
                                               uint16_t robot_port)
  : StreamMotionInterface(), socket_impl_{ std::make_unique<PSocketImpl>(robot_ip_address, robot_port, timeout) }
{
}

bool StreamMotionConnection::getRobotLimits(const uint32_t axis_number, RobotThresholdPacket& robot_threshold_velocity,
                                            RobotThresholdPacket& robot_threshold_acceleration,
                                            RobotThresholdPacket& robot_threshold_jerk) const
{
  if (axis_number < 1 || axis_number > kMaxAxisNumber)
  {
    throw std::out_of_range("Axis number must be between 1 and 9.");
  }
  ThresholdPacket threshold_packet{};
  threshold_packet.packet_type = swapBytesIfNeeded(3);
  threshold_packet.version_no = swapBytesIfNeeded(1);
  threshold_packet.axis_number = swapBytesIfNeeded(axis_number);
  // Velocity limits
  threshold_packet.threshold_type = swapBytesIfNeeded(0);
  socket_impl_->send(threshold_packet);
  if (!socket_impl_->receive(robot_threshold_velocity))
  {
    return false;
  }
  swapRobotThresholdPacketBytes(robot_threshold_velocity);
  // Acceleration limits
  threshold_packet.threshold_type = swapBytesIfNeeded(1);
  socket_impl_->send(threshold_packet);
  if (!socket_impl_->receive(robot_threshold_acceleration))
  {
    return false;
  }
  swapRobotThresholdPacketBytes(robot_threshold_acceleration);
  // Jerk limits
  threshold_packet.threshold_type = swapBytesIfNeeded(2);
  socket_impl_->send(threshold_packet);
  if (!socket_impl_->receive(robot_threshold_jerk))
  {
    return false;
  }
  swapRobotThresholdPacketBytes(robot_threshold_jerk);

  return true;
}

bool StreamMotionConnection::configureGPIO(const GPIOConfiguration& config) const
{
  ValidateGPIOConfig(config);

  GPIOConfigPacket gpio_config_packet{};
  gpio_config_packet.packet_type = 203;
  gpio_config_packet.version_no = 3;
  gpio_config_packet.gpio_configuration = config;
  swapGPIOConfigPacketBytes(gpio_config_packet);

  socket_impl_->send(gpio_config_packet);

  GPIOConfigResultPacket gpio_config_result_packet{};
  if (!socket_impl_->receive(gpio_config_result_packet))
  {
    std::cerr << "Failed to get response from IO configuration." << std::endl;
    return false;
  }
  gpio_config_result_packet.packet_type = swapBytesIfNeeded(gpio_config_result_packet.packet_type);
  gpio_config_result_packet.result = swapBytesIfNeeded(gpio_config_result_packet.result);
  gpio_config_result_packet.ptf = swapBytesIfNeeded(gpio_config_result_packet.ptf);
  if (gpio_config_result_packet.result != 0)
  {
    std::cerr << "IO configuration failed with error code: " << gpio_config_result_packet.result << std::endl;
    return false;
  }
  return true;
}

bool StreamMotionConnection::getControllerCapability(ControllerCapabilityResultPacket& controller_capability) const
{
  ControllerCapabilityPacket controller_capability_packet{};
  controller_capability_packet.packet_type = 7;  // Read capability
  controller_capability_packet.version_no = 3;
  swapControllerCapabilityBytes(controller_capability_packet);
  socket_impl_->send(controller_capability_packet);
  controller_capability = ControllerCapabilityResultPacket{};
  if (!socket_impl_->receive(controller_capability))
  {
    std::cerr << "Failed to get response from IO configuration." << std::endl;
    return false;
  }
  swapControllerCapabilityResponseBytes(controller_capability);

  return true;
}

void StreamMotionConnection::sendStartPacket() const
{
  StartPacket start_packet{};
  start_packet.packet_type = swapBytesIfNeeded(kStartPacketType);
  start_packet.version_no = swapBytesIfNeeded(kVersion);
  socket_impl_->send(start_packet);
}

void StreamMotionConnection::sendStopPacket() const
{
  StopPacket stop_packet{};
  stop_packet.packet_type = swapBytesIfNeeded(kStopPacketType);
  stop_packet.version_no = swapBytesIfNeeded(kVersion);
  socket_impl_->send(stop_packet);
}

void swapCommandPacketBytes(CommandPacket& command)
{
  command.packet_type = swapBytesIfNeeded(command.packet_type);
  command.version_no = swapBytesIfNeeded(command.version_no);
  command.unused = swapBytesIfNeeded(command.unused);
  command.sequence_no = swapBytesIfNeeded(command.sequence_no);
  for (double& pos : command.command_pos)
  {
    pos = swapBytesIfNeeded(pos);
  }
  // Skip io_command since this is always expected to be little endian.
}

void swapRobotStatusPacketBytes(RobotStatusPacket& status)
{
  status.packet_type = swapBytesIfNeeded(status.packet_type);
  status.version_no = swapBytesIfNeeded(status.version_no);
  status.sequence_no = swapBytesIfNeeded(status.sequence_no);
  status.time_stamp = swapBytesIfNeeded(status.time_stamp);
  status.safety_scale = swapBytesIfNeeded(status.safety_scale);
  for (int idx = 0; idx < kMaxAxisNumber; idx++)
  {
    status.joint_angle[idx] = swapBytesIfNeeded(status.joint_angle[idx]);
    status.position[idx] = swapBytesIfNeeded(status.position[idx]);
    status.current[idx] = swapBytesIfNeeded(status.current[idx]);
  }
  // Skip io_status since this is always expected to be little endian.
}

void swapRobotThresholdPacketBytes(RobotThresholdPacket& threshold_packet)
{
  threshold_packet.packet_type = swapBytesIfNeeded(threshold_packet.packet_type);
  threshold_packet.version_no = swapBytesIfNeeded(threshold_packet.version_no);
  threshold_packet.axis_number = swapBytesIfNeeded(threshold_packet.axis_number);
  threshold_packet.threshold_type = swapBytesIfNeeded(threshold_packet.threshold_type);
  threshold_packet.max_cartesian_speed = swapBytesIfNeeded(threshold_packet.max_cartesian_speed);
  threshold_packet.interval = swapBytesIfNeeded(threshold_packet.interval);

  for (int i = 0; i < kThresholdPayloadLength; ++i)
  {
    threshold_packet.no_payload[i] = swapBytesIfNeeded(threshold_packet.no_payload[i]);
    threshold_packet.full_payload[i] = swapBytesIfNeeded(threshold_packet.full_payload[i]);
  }
}

void swapGPIOConfigPacketBytes(GPIOConfigPacket& gpio_config_packet)
{
  gpio_config_packet.packet_type = swapBytesIfNeeded(gpio_config_packet.packet_type);
  gpio_config_packet.version_no = swapBytesIfNeeded(gpio_config_packet.version_no);
  for (auto& [command_type, gpio_type, start, length] : gpio_config_packet.gpio_configuration)
  {
    command_type = static_cast<GPIOCommandType>(swapBytesIfNeeded(static_cast<uint32_t>(command_type)));
    gpio_type = swapBytesIfNeeded(gpio_type);
    start = swapBytesIfNeeded(start);
    length = swapBytesIfNeeded(length);
  }
}

void swapControllerCapabilityBytes(ControllerCapabilityPacket& controller_capability_packet)
{
  controller_capability_packet.packet_type = swapBytesIfNeeded(controller_capability_packet.packet_type);
  controller_capability_packet.version_no = swapBytesIfNeeded(controller_capability_packet.version_no);
  controller_capability_packet.id = swapBytesIfNeeded(controller_capability_packet.id);
  controller_capability_packet.sampling_rate = swapBytesIfNeeded(controller_capability_packet.sampling_rate);
  controller_capability_packet.start_move = swapBytesIfNeeded(controller_capability_packet.start_move);
  controller_capability_packet.available_version = swapBytesIfNeeded(controller_capability_packet.available_version);
  controller_capability_packet.rob_status_use_tcp = swapBytesIfNeeded(controller_capability_packet.rob_status_use_tcp);
}

void swapControllerCapabilityResponseBytes(ControllerCapabilityResultPacket& controller_capability_result_packet)
{
  controller_capability_result_packet.packet_type = swapBytesIfNeeded(controller_capability_result_packet.packet_type);
  controller_capability_result_packet.version_no = swapBytesIfNeeded(controller_capability_result_packet.version_no);
  controller_capability_result_packet.id = swapBytesIfNeeded(controller_capability_result_packet.id);
  controller_capability_result_packet.sampling_rate =
      swapBytesIfNeeded(controller_capability_result_packet.sampling_rate);
  controller_capability_result_packet.start_move = swapBytesIfNeeded(controller_capability_result_packet.start_move);
  controller_capability_result_packet.available_version =
      swapBytesIfNeeded(controller_capability_result_packet.available_version);
  controller_capability_result_packet.rob_status_use_tcp =
      swapBytesIfNeeded(controller_capability_result_packet.rob_status_use_tcp);
}

void StreamMotionConnection::sendCommand(const std::array<double, kMaxAxisNumber>& command_pos,
                                         const bool is_last_command, const std::array<uint8_t, 256>& io_command) const
{
  CommandPacket command{};
  command.command_pos = command_pos;
  command.packet_type = kJerkLimitCommandPacketType;
  command.version_no = kVersion;
  command.sequence_no = sequence_no_;
  command.is_last_command = is_last_command;
  command.do_motn_ctrl = 1;
  command.unused = kCommandPacketUnused;
  command.io_command = io_command;
  swapCommandPacketBytes(command);
  socket_impl_->send(command);
}

bool StreamMotionConnection::getStatusPacket(RobotStatusPacket& status)
{
  status = RobotStatusPacket{};
  if (!socket_impl_->receive(status))
  {
    return false;
  }
  // Swap the bits of the received status packet
  swapRobotStatusPacketBytes(status);
  if (sequence_no_ + 1 != status.sequence_no)
  {
    return false;
  }
  sequence_no_ = status.sequence_no;

  return true;
}

void ValidateGPIOConfig(const std::array<GPIOControlConfig, 32>& gpio_config)
{
  for (const auto& config : gpio_config)
  {
    if (config.command_type != GPIOCommandType::None && config.length == 0)
    {
      throw std::invalid_argument("The GPIO configuration length must be greater than 0.");
    }
  }

  for (const auto& config : gpio_config)
  {
    if ((config.command_type == GPIOCommandType::IOState || config.command_type == GPIOCommandType::IOCmd) &&
        config.gpio_type > static_cast<uint32_t>(IOType::F))
    {
      throw std::invalid_argument(
          "If the command type of the GPIO configuration is IO, the GPIO type must one of: DO, DI, RO, RI, AO, AI, F.");
    }
    if ((config.command_type == GPIOCommandType::NumRegState || config.command_type == GPIOCommandType::NumRegCmd) &&
        config.gpio_type != static_cast<uint32_t>(NumRegType::Float))
    {
      throw std::invalid_argument(
          "If the command type of the GPIO configuration is a numeric register, the GPIO type must one of: Float.");
    }
  }

  std::vector<GPIOControlConfig> read_configs;
  std::copy_if(gpio_config.begin(), gpio_config.end(), std::back_inserter(read_configs),
               [](const GPIOControlConfig& config) { return IsReadGPIOConfig(config); });
  CheckOverlapping(read_configs);

  int32_t remaining_read_bytes = 256;
  for (const auto& config : read_configs)
  {
    remaining_read_bytes -= CalculateNumBytesConfig(config);
  }
  if (remaining_read_bytes < 0)
  {
    throw std::invalid_argument("The GPIO configuration is invalid. The total amount of IO read data exceeds 256 "
                                "bytes. The provided config would take `" +
                                std::to_string(256 - remaining_read_bytes) + "` bytes.");
  }

  std::vector<GPIOControlConfig> write_configs;
  std::copy_if(gpio_config.begin(), gpio_config.end(), std::back_inserter(write_configs),
               [](const GPIOControlConfig& config) { return IsWriteGPIOConfig(config); });
  CheckOverlapping(write_configs);

  int32_t remaining_write_bytes = 256;
  for (const auto& config : write_configs)
  {
    remaining_write_bytes -= CalculateNumBytesConfig(config);
  }
  if (remaining_write_bytes < 0)
  {
    throw std::invalid_argument("The GPIO configuration is invalid. The total amount of IO write data exceeds 256 "
                                "bytes. The provided config would take `" +
                                std::to_string(256 - remaining_write_bytes) + "` bytes.");
  }
}

}  // namespace stream_motion
