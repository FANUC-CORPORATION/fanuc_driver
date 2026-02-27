// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstdint>

namespace stream_motion
{
constexpr uint32_t kVersion = 3;  // base version for client
constexpr uint32_t kStopPacketType = 2;
constexpr uint32_t kThresholdPacketType = 3;
constexpr uint32_t kGetCapabilityPacketType = 7;
constexpr uint32_t kSetCapabilityPacketType = 8;
constexpr uint32_t kStartPacketType = 200;
constexpr uint32_t kCommandPacketType = 201;
constexpr uint32_t kGPIOConfigPacketType = 203;
constexpr uint32_t kForceSensorConfigPacketType = 205;

constexpr int kMaxAxisNumber = 9;  // data file can have either 9 axis (or xyzwpr ext) data per each position
constexpr int kMaxGPIOConfigs = 32;
constexpr int kMaxIOSize = 256;  // max 256 bytes of IO in command and status packets

#pragma pack(push, 1)

enum class IOType : uint32_t
{
  DO = 0,
  DI = 1,
  RO = 2,
  RI = 3,
  AO = 4,
  AI = 5,
  F = 6,
};

enum class GPIOCommandType : uint32_t
{
  None = 0,
  IOCmd = 1,
  IOState = 2,
  NumRegCmd = 3,
  NumRegState = 4
};

enum class NumRegType : uint32_t
{
  Float = 100,
};

struct GPIOControlConfig
{
  GPIOCommandType command_type = GPIOCommandType::None;
  uint32_t gpio_type{};  // Depending on command_type, set this to the appropriate IOType or NumRegType
  uint32_t start = 1;    // 1-based index
  uint32_t length = 0;   // Number of indices
};

// Table 3.3(a) : Status output start packet (External Device → Robot)
struct StartPacket
{
  uint32_t packet_type = kStartPacketType;
  uint32_t version_no = kVersion;
};

enum class ContactStopStatus : uint8_t
{
  None = 0,
  SAFE = 1,
  STOP = 2,
  DSBL = 3,
  ESCP = 4,
};

// Table 3.3(b) : Status packet (Robot → External Device)
struct RobotStatusPacket  // Newest status packet structure
{
  uint32_t packet_type{};  // 204
  uint32_t version_no{};
  uint32_t sequence_no{};
  uint8_t status{};
  uint8_t robot_status{};
  ContactStopStatus contact_stop_status{};
  uint8_t unused{};
  uint32_t time_stamp{};
  std::array<float, kMaxAxisNumber> position{};
  std::array<float, kMaxAxisNumber> joint_angle{};
  std::array<float, kMaxAxisNumber> current{};
  float safety_scale{};
  float force_x{};     // [N]
  float force_y{};     // [N]
  float force_z{};     // [N]
  float moment_x{};    // [Nm]
  float moment_y{};    // [Nm]
  float moment_z{};    // [Nm]
  uint32_t fs_type{};  // Force sensor type 0: Unselected, 1: EMBEDDED, 2: EXTERNAL, 0xFFFFFFFF: Unavailable
  // Note: io_status always comes in little endian. When parsing this array, least-significant bytes comes first.
  std::array<uint8_t, 256> io_status{};  // 256 bytes
};

// Old robot status packet structures to keep backward compatibility
struct V3RobotStatusPacket  // V3: no force data
{
  uint32_t packet_type{};  // 202
  uint32_t version_no{};
  uint32_t sequence_no{};
  uint8_t status{};
  uint8_t robot_status{};
  ContactStopStatus contact_stop_status{};
  uint8_t unused{};
  uint32_t time_stamp{};
  std::array<float, kMaxAxisNumber> position{};
  std::array<float, kMaxAxisNumber> joint_angle{};
  std::array<float, kMaxAxisNumber> current{};
  float safety_scale{};
  // Note: io_status always comes in little endian. When parsing this array, least-significant bytes comes first.
  std::array<uint8_t, 256> io_status{};  // 256 bytes
};

struct CommandPacket
{
  uint32_t packet_type = kCommandPacketType;
  uint32_t version_no = kVersion;
  uint32_t sequence_no{};
  uint8_t is_last_command{};
  uint8_t do_motn_ctrl{};  // 1 for motion, 0 for i/o only
  uint16_t unused{};
  std::array<double, kMaxAxisNumber> command_pos{};
  // Note: io_command expects values in little endian. When packing this array, least-significant bytes comes first.
  std::array<uint8_t, 256> io_command{};
  // could be either cartesian position or joint angle, based on dataStyle
};

// Table 3.3(d) : Status output stop packet (External Device → Robot)
struct StopPacket
{
  uint32_t packet_type = kStopPacketType;
  uint32_t version_no = kVersion;
};

// Table B.3(a): Request packet for allowable upper limit table (External Device → Robot)
struct ThresholdPacket
{
  uint32_t packet_type = kThresholdPacketType;
  uint32_t version_no = kVersion;
  uint32_t axis_number{};    /* Specify one of the integers 1-9. */
  uint32_t threshold_type{}; /* Specify one of 0:velocity [deg/s], 1:acceleration [deg/s^2], 2:jerk [deg/s^3]. */
};

// Table B.3(b): Response packet for allowable upper limit table (Robot → External Device)
struct RobotThresholdPacket
{
  uint32_t packet_type{};
  uint32_t version_no{};
  uint32_t axis_number{};
  uint32_t threshold_type{};
  uint32_t max_cartesian_speed{};
  uint32_t interval{};
  float no_payload[20]{};
  float full_payload[20]{};
};

// Table 1.1 GPIO configuration packet (External Device → Robot)
using GPIOConfiguration = std::array<GPIOControlConfig, kMaxGPIOConfigs>;
struct GPIOConfigPacket
{
  uint32_t packet_type = kGPIOConfigPacketType;
  uint32_t version_no = kVersion;
  GPIOConfiguration gpio_configuration{};  // 32 configs, each 16 bytes
};

struct GPIOConfigResultPacket
{
  uint32_t packet_type{};  // 203
  uint32_t result{};       // 0: success, other: error codes
  uint32_t ptf{};          // Process Time Factor
};

// Table 1.3 Get/Set controller capability request packet (External Device → Robot)
struct ControllerCapabilityPacket
{
  uint32_t packet_type{};  // 7 (get) or 8 (set)
  uint32_t version_no = kVersion;
  uint32_t id = 1;
  uint32_t sampling_rate{};       // Not used (Read Only parameter)
  uint32_t start_move{};          // 0-9, buffer size to start motion
  uint32_t available_version{};   // Not used (Read Only parameter)
  uint32_t rob_status_use_tcp{};  // 0: face plate position, 1: TCP position
};

// Table 1.4 Get/Set controller capability result packet (Robot → External Device)
struct ControllerCapabilityResultPacket
{
  uint32_t packet_type{};  // 7 (get) or 8 (set)
  uint32_t version_no{};
  uint32_t id{};
  uint32_t sampling_rate{};       // Sampling rate of Stream Motion [msec]
  uint32_t start_move{};          // The current setting
  uint32_t available_version{};   // Available version of Stream Motion
  uint32_t rob_status_use_tcp{};  // The current setting
};

struct ForceSensorConfigPacket
{
  uint32_t packet_type = kForceSensorConfigPacketType;
  uint32_t version_no = kVersion;
  uint32_t do_reset{};  // 0: do nothing, 1: reset force
  uint32_t fs_type{};   // 1: EMBEDDED, 2: EXTERNAL
};

#pragma pack(pop)

}  // namespace stream_motion
