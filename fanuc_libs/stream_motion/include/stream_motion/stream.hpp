// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <memory>
#include <string>

#include "stream_motion/packets.hpp"

namespace stream_motion
{
class StreamMotionInterface
{
public:
  StreamMotionInterface() = default;
  virtual ~StreamMotionInterface() = default;

  /**
   * @brief Sends a start packet to initiate the stream motion connection.
   */
  virtual void sendStartPacket() const = 0;

  /**
   * @brief Sends a stop packet to terminate the stream motion connection.
   */
  virtual void sendStopPacket() const = 0;

  /**
   * @brief Sends a command packet with the specified positions.
   * @param command_pos The target positions for each axis.
   * @param io_command IO data to write. For IO types, each bit is mapped to an IO port on the robot controller. For
   * numerical types, the values are encoded in little endian format.
   * @param is_last_command Indicates if this is the last command in the sequence.
   */
  virtual void sendCommand(const std::array<double, kMaxAxisNumber>& command_pos, bool is_last_command,
                           const std::array<uint8_t, 256>& io_command) const = 0;

  /**
   * @brief Receives a robot status packet from the robot. This method blocks until a status packet is received or a
   * timeout occurs.
   * @param status Reference to a RobotStatusPacket to store the received status.
   * @return true if the status packet was received successfully, false is a timeout occurs.
   */
  virtual bool getStatusPacket(RobotStatusPacket& status) = 0;

  /**
   * @brief Gets the dynamic limits via the stream connection.
   *
   * This method is designed to be called before streaming motion begins.
   *
   * @return true if the limits were received successfully, false is a timeout occurs.
   */
  virtual bool getRobotLimits(uint32_t axis_number, RobotThresholdPacket& robot_threshold_velocity,
                              RobotThresholdPacket& robot_threshold_acceleration,
                              RobotThresholdPacket& robot_threshold_jerk) const = 0;

  /**
   * @brief Configures the format of the command and status IO. **/
  virtual bool configureGPIO(const GPIOConfiguration& config) const = 0;

  virtual bool getControllerCapability(ControllerCapabilityResultPacket& controller_capability) const = 0;
};

class StreamMotionConnection final : public StreamMotionInterface
{
public:
  StreamMotionConnection() = delete;
  [[maybe_unused]] explicit StreamMotionConnection(const std::string& robot_ip_address, double timeout = 1.0,
                                                   uint16_t robot_port = 60015);
  ~StreamMotionConnection() override;

  StreamMotionConnection(const StreamMotionConnection&) = delete;

  StreamMotionConnection& operator=(const StreamMotionConnection&) = delete;

  void sendStartPacket() const override;

  void sendStopPacket() const override;

  void sendCommand(const std::array<double, kMaxAxisNumber>& command_pos, bool is_last_command,
                   const std::array<uint8_t, 256>& io_command) const override;

  bool getStatusPacket(RobotStatusPacket& status) override;

  bool getRobotLimits(uint32_t axis_number, RobotThresholdPacket& robot_threshold_velocity,
                      RobotThresholdPacket& robot_threshold_acceleration,
                      RobotThresholdPacket& robot_threshold_jerk) const override;

  bool configureGPIO(const GPIOConfiguration& config) const override;

  bool getControllerCapability(ControllerCapabilityResultPacket& controller_capability) const override;

private:
  uint32_t status_sequence_no_ = 0;
  uint32_t command_sequence_no_ = 0;

  struct PSocketImpl;
  std::unique_ptr<PSocketImpl> socket_impl_;
};

void swapCommandPacketBytes(CommandPacket& command);

void swapRobotStatusPacketBytes(RobotStatusPacket& status);

void swapRobotThresholdPacketBytes(RobotThresholdPacket& threshold_packet);

void swapGPIOConfigPacketBytes(GPIOConfigPacket& gpio_config_packet);

void swapControllerCapabilityBytes(ControllerCapabilityPacket& controller_capability_packet);

void swapControllerCapabilityResponseBytes(ControllerCapabilityResultPacket& controller_capability_result_packet);

void ValidateGPIOConfig(const GPIOConfiguration& gpio_config);

}  // namespace stream_motion
