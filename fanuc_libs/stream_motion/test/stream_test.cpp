// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include <cmath>

#include <gtest/gtest.h>
#include <thread>

#include "sockpp/udp_socket.h"
#include "stream_motion/byte_ops.hpp"
#include "stream_motion/stream.hpp"

namespace
{
std::array<float, stream_motion::kMaxAxisNumber> kDefaultVector = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };
std::array<double, stream_motion::kMaxAxisNumber> kDefaultCommandVectorD = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
}  // namespace

class FakeRobot
{
public:
  const std::string ip_address = "127.0.0.1";
  const uint16_t port = 60015;

  FakeRobot()
  {
    sock_.set_non_blocking(false);
    sock_.bind(sockpp::inet_address(ip_address, port));
    if (!sock_)
    {
      throw std::runtime_error("Failed to bind socket to port in test");
    }
  }
  static stream_motion::RobotStatusPacket createStatusPacket(const int sequence_no)
  {
    stream_motion::RobotStatusPacket status;
    status.status = 4;
    status.packet_type = 0;
    status.version_no = 3;
    status.position = kDefaultVector;
    status.joint_angle = kDefaultVector;
    status.current = kDefaultVector;
    status.sequence_no = sequence_no;
    return status;
  }

  std::tuple<stream_motion::StartPacket, sockpp::inet_address> receiveStartPacket()
  {
    stream_motion::StartPacket start_packet{};
    sockpp::inet_address sender_address;
    const sockpp::result<size_t> res = sock_.recv_from(&start_packet, sizeof(start_packet), &sender_address);
    assert(res.value() == sizeof(start_packet));
    start_packet.version_no = stream_motion::swapBytesIfNeeded(start_packet.version_no);
    start_packet.packet_type = stream_motion::swapBytesIfNeeded(start_packet.packet_type);
    EXPECT_EQ(start_packet.version_no, 2);
    EXPECT_EQ(start_packet.packet_type, 200);
    return std::make_tuple(start_packet, sender_address);
  }

  std::tuple<stream_motion::CommandPacket, sockpp::inet_address> receiveCommandPacket()
  {
    stream_motion::CommandPacket command_packet{};
    sockpp::inet_address sender_address;
    const sockpp::result<size_t> res = sock_.recv_from(&command_packet, sizeof(command_packet), &sender_address);
    assert(res.value() == sizeof(command_packet));
    stream_motion::swapCommandPacketBytes(command_packet);
    return std::make_tuple(command_packet, sender_address);
  }

  void sendStatusPacket(const sockpp::inet_address& address, const int sequence_no)
  {
    auto status = createStatusPacket(sequence_no);
    stream_motion::swapRobotStatusPacketBytes(status);
    sock_.send_to(&status, sizeof(status), address);
  }

  void respondStartPacket(const int sequence_no)
  {
    auto [start_packet, sender_address] = receiveStartPacket();
    ASSERT_EQ(start_packet.version_no, 2);
    ASSERT_EQ(start_packet.packet_type, 200);
    sendStatusPacket(sender_address, sequence_no);
  }

  void respondIOConfigPacket()
  {
    stream_motion::GPIOConfigPacket gpio_config_packet{};
    sockpp::inet_address sender_address;
    const sockpp::result<size_t> res =
        sock_.recv_from(&gpio_config_packet, sizeof(gpio_config_packet), &sender_address);
    assert(res.value() == sizeof(gpio_config_packet));
    swapGPIOConfigPacketBytes(gpio_config_packet);
    ASSERT_EQ(gpio_config_packet.packet_type, 203);
    const stream_motion::GPIOConfigResultPacket gpio_config_result{};
    sock_.send_to(&gpio_config_result, sizeof(gpio_config_result), sender_address);
  }

  void receiveStopPacket()
  {
    stream_motion::StopPacket stop_packet{};
    sockpp::inet_address sender_address;
    const sockpp::result<size_t> res = sock_.recv_from(&stop_packet, sizeof(stop_packet), &sender_address);
    assert(res.value() == sizeof(stop_packet));
    stop_packet.version_no = stream_motion::swapBytesIfNeeded(stop_packet.version_no);
    stop_packet.packet_type = stream_motion::swapBytesIfNeeded(stop_packet.packet_type);
    ASSERT_EQ(stop_packet.version_no, 2);
    ASSERT_EQ(stop_packet.packet_type, 2);
  }

  void respondCommandPacket(const int sequence_no)
  {
    auto [command_packet, sender_address] = receiveCommandPacket();
    ASSERT_EQ(command_packet.sequence_no, 1);
    ASSERT_EQ(command_packet.is_last_command, 0);
    ASSERT_EQ(command_packet.command_pos, kDefaultCommandVectorD);
    sendStatusPacket(sender_address, sequence_no);
  }

  std::tuple<stream_motion::ThresholdPacket, sockpp::inet_address> receiveThresholdPacket()
  {
    stream_motion::ThresholdPacket threshold_packet{};
    sockpp::inet_address sender_address;
    const sockpp::result<size_t> res = sock_.recv_from(&threshold_packet, sizeof(threshold_packet), &sender_address);
    assert(res.value() == sizeof(threshold_packet));
    threshold_packet.version_no = stream_motion::swapBytesIfNeeded(threshold_packet.version_no);
    threshold_packet.packet_type = stream_motion::swapBytesIfNeeded(threshold_packet.packet_type);
    threshold_packet.axis_number = stream_motion::swapBytesIfNeeded(threshold_packet.axis_number);
    threshold_packet.threshold_type = stream_motion::swapBytesIfNeeded(threshold_packet.threshold_type);
    return std::make_tuple(threshold_packet, sender_address);
  }

  static stream_motion::RobotThresholdPacket
  createRobotThresholdPacket(const stream_motion::ThresholdPacket& threshold_packet, const float no_payload,
                             const float full_payload)
  {
    stream_motion::RobotThresholdPacket robot_threshold_packet{};
    robot_threshold_packet.threshold_type = threshold_packet.threshold_type;
    robot_threshold_packet.axis_number = threshold_packet.axis_number;
    robot_threshold_packet.version_no = threshold_packet.version_no;
    robot_threshold_packet.packet_type = threshold_packet.packet_type;
    robot_threshold_packet.max_cartesian_speed = 2000;
    robot_threshold_packet.interval = 8;
    for (int i = 0; i < 20; ++i)
    {
      robot_threshold_packet.no_payload[i] = no_payload;
      robot_threshold_packet.full_payload[i] = full_payload;
    }

    return robot_threshold_packet;
  }

  void respondGetJointLimits()
  {
    auto [threshold_packet, address] = receiveThresholdPacket();
    ASSERT_EQ(threshold_packet.threshold_type, 0);

    // Respond to velocity
    stream_motion::RobotThresholdPacket robot_threshold_packet = createRobotThresholdPacket(threshold_packet, 100, 200);
    stream_motion::swapRobotThresholdPacketBytes(robot_threshold_packet);
    sock_.send_to(&robot_threshold_packet, sizeof(robot_threshold_packet), address);
    // Respond to acceleration
    robot_threshold_packet = createRobotThresholdPacket(threshold_packet, 1000, 2000);
    stream_motion::swapRobotThresholdPacketBytes(robot_threshold_packet);
    sock_.send_to(&robot_threshold_packet, sizeof(robot_threshold_packet), address);
    // Respond to jerk
    robot_threshold_packet = createRobotThresholdPacket(threshold_packet, 10000, 20000);
    stream_motion::swapRobotThresholdPacketBytes(robot_threshold_packet);
    sock_.send_to(&robot_threshold_packet, sizeof(robot_threshold_packet), address);
  }

private:
  sockpp::udp_socket sock_;
};

TEST(StreamMotionConnectionTest, TestSuccessfulConnectionCycle)
{
  // Create mock robot
  auto robot = FakeRobot();

  // Create stream motion connection
  stream_motion::StreamMotionConnection connection(robot.ip_address, robot.port);

  // Send start packet and expect a status packet back
  connection.sendStartPacket();
  std::thread respond_start_packet([&robot] { robot.respondStartPacket(1); });
  stream_motion::RobotStatusPacket status{};
  ASSERT_TRUE(connection.getStatusPacket(status));

  EXPECT_EQ((status.status & 0x4) >> 2, 1);
  EXPECT_EQ(status.packet_type, 0);
  EXPECT_EQ(status.version_no, 3);
  EXPECT_EQ(status.joint_angle, kDefaultVector);
  EXPECT_EQ(status.position, kDefaultVector);
  EXPECT_EQ(status.current, kDefaultVector);
  EXPECT_EQ(status.sequence_no, 1);

  connection.sendCommand(kDefaultCommandVectorD, false, {});
  std::thread respond_status_packet([&robot] { robot.respondCommandPacket(2); });
  connection.getStatusPacket(status);
  EXPECT_EQ(status.sequence_no, 2);

  std::thread respond_stop_packet([&robot] { robot.receiveStopPacket(); });
  connection.sendStopPacket();

  respond_start_packet.join();
  respond_status_packet.join();
  respond_stop_packet.join();
}

TEST(StreamMotionConnectionTest, TestGetJointLimits)
{
  // Create mock robot
  auto robot = FakeRobot();

  // Create stream motion connection
  const stream_motion::StreamMotionConnection connection(robot.ip_address, robot.port);

  stream_motion::RobotThresholdPacket robot_threshold_velocity{};
  stream_motion::RobotThresholdPacket robot_threshold_acceleration{};
  stream_motion::RobotThresholdPacket robot_threshold_jerk{};

  std::thread respond([&robot] { robot.respondGetJointLimits(); });
  ASSERT_TRUE(
      connection.getRobotLimits(1, robot_threshold_velocity, robot_threshold_acceleration, robot_threshold_jerk));
  EXPECT_EQ(robot_threshold_velocity.axis_number, 1);
  EXPECT_EQ(robot_threshold_velocity.full_payload[0], 200);
  EXPECT_EQ(robot_threshold_velocity.no_payload[0], 100);
  EXPECT_EQ(robot_threshold_acceleration.axis_number, 1);
  EXPECT_EQ(robot_threshold_acceleration.full_payload[0], 2000);
  EXPECT_EQ(robot_threshold_acceleration.no_payload[0], 1000);
  EXPECT_EQ(robot_threshold_jerk.axis_number, 1);
  EXPECT_EQ(robot_threshold_jerk.full_payload[0], 20000);
  EXPECT_EQ(robot_threshold_jerk.no_payload[0], 10000);

  // Test with an invalid axis number
  ASSERT_THROW(connection.getRobotLimits(0, robot_threshold_velocity, robot_threshold_acceleration,
                                         robot_threshold_jerk),
               std::out_of_range);

  respond.join();
}

TEST(StreamMotionConnectionTest, TestConfigureGPIO)
{
  // Create mock robot
  auto robot = FakeRobot();

  // Create stream motion connection
  const stream_motion::StreamMotionConnection connection(robot.ip_address, robot.port);

  std::thread respond([&robot] { robot.respondIOConfigPacket(); });
  stream_motion::GPIOConfiguration gpio_config;

  EXPECT_TRUE(connection.configureGPIO(gpio_config));

  respond.join();
}

TEST(ByteOrder, Conversions)
{
  // Test swapBytesIfNeeded (swapBytesIfNeeded) for all supported types
  if constexpr (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
  {
    EXPECT_EQ(stream_motion::swapBytesIfNeeded(uint16_t{ 0x1234 }), uint16_t{ 0x3412 });
    EXPECT_EQ(stream_motion::swapBytesIfNeeded(uint32_t{ 0x12345678 }), uint32_t{ 0x78563412 });
    EXPECT_EQ(stream_motion::swapBytesIfNeeded(int16_t{ 0x1234 }), int16_t{ 0x3412 });
    EXPECT_EQ(stream_motion::swapBytesIfNeeded(int32_t{ 0x12345678 }), int32_t{ 0x78563412 });
    EXPECT_EQ(stream_motion::swapBytesIfNeeded(uint64_t{ 0x0123456789ABCDEF }), uint64_t{ 0xEFCDAB8967452301 });
  }

  EXPECT_EQ(stream_motion::swapBytesIfNeeded(uint16_t{ 0x1234 }, true), uint16_t{ 0x3412 });
  EXPECT_EQ(stream_motion::swapBytesIfNeeded(uint32_t{ 0x12345678 }, true), uint32_t{ 0x78563412 });
  EXPECT_EQ(stream_motion::swapBytesIfNeeded(int16_t{ 0x1234 }, true), int16_t{ 0x3412 });
  EXPECT_EQ(stream_motion::swapBytesIfNeeded(int32_t{ 0x12345678 }, true), int32_t{ 0x78563412 });
  EXPECT_EQ(stream_motion::swapBytesIfNeeded(uint64_t{ 0x0123456789ABCDEF }, true), uint64_t{ 0xEFCDAB8967452301 });

  EXPECT_EQ(stream_motion::swapBytesIfNeeded(uint16_t{ 0x1234 }, false), uint16_t{ 0x1234 });
  EXPECT_EQ(stream_motion::swapBytesIfNeeded(uint32_t{ 0x12345678 }, false), uint32_t{ 0x12345678 });
  EXPECT_EQ(stream_motion::swapBytesIfNeeded(int16_t{ 0x1234 }, false), int16_t{ 0x1234 });
  EXPECT_EQ(stream_motion::swapBytesIfNeeded(int32_t{ 0x12345678 }, false), int32_t{ 0x12345678 });
  EXPECT_EQ(stream_motion::swapBytesIfNeeded(uint64_t{ 0x0123456789ABCDEF }, false), uint64_t{ 0x0123456789ABCDEF });
}

TEST(Packets, CommandPacketSize)
{
  EXPECT_EQ(sizeof(stream_motion::CommandPacket), 344);
  EXPECT_EQ(sizeof(stream_motion::RobotStatusPacket), 388);
  EXPECT_EQ(sizeof(stream_motion::StartPacket), 8);
  EXPECT_EQ(sizeof(stream_motion::StopPacket), 8);
  EXPECT_EQ(sizeof(stream_motion::ThresholdPacket), 16);
  EXPECT_EQ(sizeof(stream_motion::RobotThresholdPacket), 184);
}

TEST(ValidateGPIOConfig, ValidConfig)
{
  std::array<stream_motion::GPIOControlConfig, 32> gpio_config{};
  gpio_config[0].command_type = stream_motion::GPIOCommandType::IOCmd;
  gpio_config[0].gpio_type = static_cast<uint32_t>(stream_motion::IOType::F);
  gpio_config[0].start = 1;
  gpio_config[0].length = 32;
  gpio_config[1].command_type = stream_motion::GPIOCommandType::IOState;
  gpio_config[1].gpio_type = static_cast<uint32_t>(stream_motion::IOType::F);
  gpio_config[1].start = 1;
  gpio_config[1].length = 32;
  EXPECT_NO_THROW(stream_motion::ValidateGPIOConfig(gpio_config));
  const std::array<stream_motion::GPIOControlConfig, 32> empty_gpio_config{};
  EXPECT_NO_THROW(stream_motion::ValidateGPIOConfig(empty_gpio_config));
}

TEST(ValidateGPIOConfig, OverlappingIndicesFail)
{
  std::array<stream_motion::GPIOControlConfig, 32> gpio_config{};
  gpio_config[0].command_type = stream_motion::GPIOCommandType::IOState;
  gpio_config[0].gpio_type = static_cast<uint32_t>(stream_motion::IOType::F);
  gpio_config[0].start = 16;
  gpio_config[0].length = 32;
  gpio_config[1].command_type = stream_motion::GPIOCommandType::IOState;
  gpio_config[1].gpio_type = static_cast<uint32_t>(stream_motion::IOType::F);
  gpio_config[1].start = 1;  // Overlaps with the first command
  gpio_config[1].length = 32;
  EXPECT_THROW(stream_motion::ValidateGPIOConfig(gpio_config), std::invalid_argument);
}

TEST(ValidateGPIOConfig, OverlappingIndicesDifferentTypesSuccess)
{
  std::array<stream_motion::GPIOControlConfig, 32> gpio_config{};
  gpio_config[0].command_type = stream_motion::GPIOCommandType::IOState;
  gpio_config[0].gpio_type = static_cast<uint32_t>(stream_motion::IOType::DI);
  gpio_config[0].start = 16;
  gpio_config[0].length = 32;
  gpio_config[1].command_type = stream_motion::GPIOCommandType::IOState;
  gpio_config[1].gpio_type = static_cast<uint32_t>(stream_motion::IOType::F);
  gpio_config[1].start = 1;  // Overlaps with the first command
  gpio_config[1].length = 32;
  stream_motion::ValidateGPIOConfig(gpio_config);
}

TEST(ValidateGPIOConfig, ZeroLengthConfigFail)
{
  std::array<stream_motion::GPIOControlConfig, 32> gpio_config{};
  gpio_config[0].command_type = stream_motion::GPIOCommandType::IOState;
  gpio_config[0].gpio_type = static_cast<uint32_t>(stream_motion::IOType::F);
  gpio_config[0].start = 1;
  gpio_config[0].length = 0;
  EXPECT_THROW(stream_motion::ValidateGPIOConfig(gpio_config), std::invalid_argument);
}

TEST(ValidateGPIOConfig, TooManyBytesFail)
{
  std::array<stream_motion::GPIOControlConfig, 32> gpio_config{};
  gpio_config[0].command_type = stream_motion::GPIOCommandType::IOState;
  gpio_config[0].gpio_type = static_cast<uint32_t>(stream_motion::IOType::F);
  gpio_config[0].start = 1;
  gpio_config[0].length = 1 + 256 * 8;
  EXPECT_THROW(stream_motion::ValidateGPIOConfig(gpio_config), std::invalid_argument);

  gpio_config[0].command_type = stream_motion::GPIOCommandType::NumRegCmd;
  gpio_config[0].gpio_type = static_cast<uint32_t>(stream_motion::NumRegType::Float);
  gpio_config[0].start = 1;
  gpio_config[0].length = 1 + 256 / 4;
  EXPECT_THROW(stream_motion::ValidateGPIOConfig(gpio_config), std::invalid_argument);

  gpio_config[0].command_type = stream_motion::GPIOCommandType::NumRegState;
  gpio_config[0].gpio_type = static_cast<uint32_t>(stream_motion::NumRegType::Float);
  gpio_config[0].start = 1;
  gpio_config[0].length = 1 + 256 / 4;
  EXPECT_THROW(stream_motion::ValidateGPIOConfig(gpio_config), std::invalid_argument);

  gpio_config[0].command_type = stream_motion::GPIOCommandType::IOState;
  gpio_config[0].gpio_type = static_cast<uint32_t>(stream_motion::IOType::AO);
  gpio_config[0].start = 1;
  gpio_config[0].length = 1 + 256 / 2;
  EXPECT_THROW(stream_motion::ValidateGPIOConfig(gpio_config), std::invalid_argument);
}
