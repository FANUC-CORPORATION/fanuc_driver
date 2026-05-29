// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <iostream>

#include "stream_motion/stream.hpp"

void printStatus(const stream_motion::RobotStatusPacket& status)
{
  std::cout << "packet_type: " << status.packet_type << std::endl;
  std::cout << "version_no: " << status.version_no << std::endl;
  std::cout << "sequence_no: " << status.sequence_no << std::endl;
  std::cout << "status: " << static_cast<int>(status.status) << std::endl;
  std::cout << "robot_status: " << static_cast<int>(status.robot_status) << std::endl;
  std::cout << "contact_stop_status: " << static_cast<int>(status.contact_stop_status) << std::endl;
  std::cout << "time_stamp: " << status.time_stamp << std::endl;
  for (int i = 0; i < status.position.size(); ++i)
  {
    std::cout << "position[" << i << "]: " << status.position[i] << std::endl;
  }
  for (int i = 0; i < status.joint_angle.size(); ++i)
  {
    std::cout << "joint_angle[" << i << "]: " << status.joint_angle[i] << std::endl;
  }
  for (int i = 0; i < status.current.size(); ++i)
  {
    std::cout << "current[" << i << "]: " << status.current[i] << std::endl;
  }
  std::cout << "status.io_status[1]: " << (int)status.io_status[1] << std::endl;
}

void printJointLimits(const stream_motion::RobotThresholdPacket& robot_threshold_velocity,
                      const stream_motion::RobotThresholdPacket& robot_threshold_acceleration,
                      const stream_motion::RobotThresholdPacket& robot_threshold_jerk)
{
  std::cout << "Velocity limits:" << std::endl;
  for (size_t i = 0; i < 20; ++i)
  {
    std::cout << "  joint[" << i << "]: " << robot_threshold_velocity.no_payload[i] << std::endl;
  }
  std::cout << "Acceleration limits:" << std::endl;
  for (size_t i = 0; i < 20; ++i)
  {
    std::cout << "  joint[" << i << "]: " << robot_threshold_acceleration.no_payload[i] << std::endl;
  }
  std::cout << "Jerk limits:" << std::endl;
  for (size_t i = 0; i < 20; ++i)
  {
    std::cout << "  joint[" << i << "]: " << robot_threshold_jerk.no_payload[i] << std::endl;
  }
}
int main()
{
  // Crete IO config for reading and writing
  std::array<uint8_t, 256> io_command{};
  for (int i = 0; i < 256; ++i)
  {
    io_command[i] = 0xFF;
  }

  std::array<stream_motion::GPIOControlConfig, 32> gpio_config{};
  gpio_config[0].command_type = stream_motion::GPIOCommandType::IOCmd;
  gpio_config[0].gpio_type = static_cast<uint32_t>(stream_motion::IOType::F);
  gpio_config[0].start = 1;
  gpio_config[0].length = 32;

  gpio_config[1].command_type = stream_motion::GPIOCommandType::IOState;
  gpio_config[1].gpio_type = static_cast<uint32_t>(stream_motion::IOType::F);
  gpio_config[1].start = 1;
  gpio_config[1].length = 32;

  stream_motion::StreamMotionConnection connection("192.168.1.100");
  stream_motion::RobotThresholdPacket robot_threshold_velocity;
  stream_motion::RobotThresholdPacket robot_threshold_acceleration;
  stream_motion::RobotThresholdPacket robot_threshold_jerk;

  // Get the robot limits for axis 1 and print them out
  connection.getRobotLimits(1, robot_threshold_velocity, robot_threshold_acceleration, robot_threshold_jerk);
  printJointLimits(robot_threshold_velocity, robot_threshold_acceleration, robot_threshold_jerk);

  // Setup GPIO
  connection.configureGPIO(gpio_config);

  // Start the streaming protocol
  connection.sendStartPacket();
  stream_motion::RobotStatusPacket status{};
  connection.getStatusPacket(status);

  // Print the initial status packet
  printStatus(status);

  std::array<double, stream_motion::kMaxAxisNumber> initial_command{};
  std::transform(status.joint_angle.begin(), status.joint_angle.end(), initial_command.begin(),
                 [](const float angle) { return static_cast<double>(angle); });
  std::array<double, stream_motion::kMaxAxisNumber> current_command = initial_command;

  // Generate a sine wave motion for 5 seconds
  constexpr int period = 5000;
  constexpr int num_cycles = 1;
  constexpr int num_steps = num_cycles * (period / 8);
  const double dt = 0.008;
  for (int i = 0; i < num_steps; ++i)
  {
    // If we fail to get the status packet due to timeout, we need to send the stop packet to clean up the connection
    if (!connection.getStatusPacket(status))
    {
      break;
    }
    const double time = i * dt;  // 8ms per iteration
    for (int j = 0; j < status.joint_angle.size(); ++j)
    {
      const double omega = (2 * M_PI) / (static_cast<double>(period) / 1000);
      current_command[j] = initial_command[j] + 20.0 * (1 + -cos(time * omega));
    }
    // Send the command to the robot
    connection.sendCommand(current_command, false, io_command, 1);
  }

  if (connection.getStatusPacket(status))
  {
    connection.sendCommand(initial_command, true, io_command, 1);
  }

  // checking motion completed
  bool wait_for_completed = true;
  while (wait_for_completed)
  {
    connection.getStatusPacket(status);
    wait_for_completed = status.status & 1 || status.status & 8;
  }

  // Terminate the streaming protocol
  connection.sendStopPacket();

  // Print the final status packet
  printStatus(status);
}
