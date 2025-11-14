// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "fanuc_robot_driver/hardware_interface.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "fanuc_client/gpio_buffer.hpp"
#include "fanuc_robot_driver/constants.hpp"
#include "gpio_config/gpio_config.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace fanuc_robot_driver
{
namespace
{
using CommandGPIOTypes = ::fanuc_client::GPIOBuffer::CommandGPIOTypes;
using StatusGPIOTypes = ::fanuc_client::GPIOBuffer::StatusGPIOTypes;

constexpr auto kFRHWInterface = "FR_HW_Interface";
constexpr int kNumberConnectionAttempts = 5;

int StringToInt(const std::string& param_name, const std::string& param_value)
{
  try
  {
    return std::stoi(param_value);
  }
  catch (const std::invalid_argument& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRHWInterface), "Invalid integer parameter `%s` for parameter named `%s`",
                 param_value.c_str(), param_name.c_str());
    throw;
  }
  catch (const std::out_of_range& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRHWInterface), "Integer parameter out of range: `%s` for parameter named `%s`",
                 param_value.c_str(), param_name.c_str());
    throw;
  }
}
}  // namespace

struct IOCommandInterface
{
  // Get the state interface name;
  virtual std::string name() const = 0;
  // Update internals based on value.
  virtual void updateBuffer() = 0;
  // The GPIO index.
  uint32_t index;
  // The value as a double (set by ROS 2 control).
  double value = 0;
};

template <CommandGPIOTypes type, typename T>
struct IOCommand : IOCommandInterface
{
  IOCommand(int i, fanuc_client::GPIOBuffer::CommandBlock<type, T>& b) : block(b)
  {
    index = i;
  }

  std::string name() const override
  {
    return std::string(block.type());
  }

  void updateBuffer() override
  {
    block.set(index, static_cast<T>(value));
  }

  // The block in the GPIO command buffer that this index belongs to.
  fanuc_client::GPIOBuffer::CommandBlock<type, T>& block;
};

struct IOStateInterface
{
  virtual std::string name() const = 0;
  // Update value based on internal sources.
  virtual void updateValue() = 0;
  // The GPIO index.
  uint32_t index;
  // The value as a double (read by ROS 2 control).
  double value = 0;
};

template <StatusGPIOTypes type, typename T>
struct IOState : IOStateInterface
{
  IOState(int i, fanuc_client::GPIOBuffer::StatusBlock<type, T>& b) : block(b)
  {
    index = i;
  }

  std::string name() const override
  {
    return std::string(block.type());
  }

  void updateValue() override
  {
    value = static_cast<double>(block.get(index));
  }

  // The block in the GPIO status buffer that this index belongs to.
  fanuc_client::GPIOBuffer::StatusBlock<type, T>& block;
};

struct IOInterfaces
{
  std::vector<std::unique_ptr<IOCommandInterface>> commands;
  std::vector<std::unique_ptr<IOStateInterface>> states;
  fanuc_client::GPIOBuffer buffer;
};

namespace
{

template <CommandGPIOTypes type>
void AppendBoolCmdInterfaces(uint32_t start, uint32_t length, fanuc_client::GPIOBuffer::Builder& builder,
                             std::vector<std::unique_ptr<IOCommandInterface>>& interfaces)
{
  auto& block = builder.addCommandConfig<type, bool>(start, length);
  for (uint32_t index = start; index < start + length; ++index)
  {
    interfaces.emplace_back(std::make_unique<IOCommand<type, bool>>(index, block));
  }
}

template <StatusGPIOTypes type>
void AppendBoolStateInterfaces(uint32_t start, uint32_t length, fanuc_client::GPIOBuffer::Builder& builder,
                               std::vector<std::unique_ptr<IOStateInterface>>& interfaces)
{
  auto& block = builder.addStatusConfig<type, bool>(start, length);
  for (uint32_t index = start; index < start + length; ++index)
  {
    interfaces.emplace_back(std::make_unique<IOState<type, bool>>(index, block));
  }
}

template <CommandGPIOTypes type>
void AppendAnalogCmdInterfaces(uint32_t start, uint32_t length, fanuc_client::GPIOBuffer::Builder& builder,
                               std::vector<std::unique_ptr<IOCommandInterface>>& interfaces)
{
  auto& block = builder.addCommandConfig<type, uint16_t>(start, length);
  for (uint32_t index = start; index < start + length; ++index)
  {
    interfaces.emplace_back(std::make_unique<IOCommand<type, uint16_t>>(index, block));
  }
}

template <StatusGPIOTypes type>
void AppendAnalogStateInterfaces(uint32_t start, uint32_t length, fanuc_client::GPIOBuffer::Builder& builder,
                                 std::vector<std::unique_ptr<IOStateInterface>>& interfaces)
{
  auto& block = builder.addStatusConfig<type, uint16_t>(start, length);
  for (uint32_t index = start; index < start + length; ++index)
  {
    interfaces.emplace_back(std::make_unique<IOState<type, uint16_t>>(index, block));
  }
}

IOInterfaces GPIOConfigToInterfaces(const gpio_config::GPIOTopicConfig& config)
{
  std::vector<std::unique_ptr<IOCommandInterface>> commands;
  std::vector<std::unique_ptr<IOStateInterface>> states;
  fanuc_client::GPIOBuffer::Builder buf_builder{};

  using ::gpio_config::BoolIOCmdType;
  if (config.io_cmd.has_value())
  {
    for (const gpio_config::BoolIOCmdConfig& bool_cmd : *config.io_cmd)
    {
      switch (bool_cmd.type)
      {
        case BoolIOCmdType::DO:
          AppendBoolCmdInterfaces<CommandGPIOTypes::DO>(bool_cmd.start, bool_cmd.length, buf_builder, commands);
          break;
        case BoolIOCmdType::RO:
          AppendBoolCmdInterfaces<CommandGPIOTypes::RO>(bool_cmd.start, bool_cmd.length, buf_builder, commands);
          break;
        case BoolIOCmdType::F:
          AppendBoolCmdInterfaces<CommandGPIOTypes::F>(bool_cmd.start, bool_cmd.length, buf_builder, commands);
          break;
      }
    }
  }

  using ::gpio_config::BoolIOStateType;
  if (config.io_state.has_value())
  {
    for (const gpio_config::BoolIOStateConfig& bool_state : *config.io_state)
    {
      switch (bool_state.type)
      {
        case BoolIOStateType::DO:
          AppendBoolStateInterfaces<StatusGPIOTypes::DO>(bool_state.start, bool_state.length, buf_builder, states);
          break;
        case BoolIOStateType::DI:
          AppendBoolStateInterfaces<StatusGPIOTypes::DI>(bool_state.start, bool_state.length, buf_builder, states);
          break;
        case BoolIOStateType::RO:
          AppendBoolStateInterfaces<StatusGPIOTypes::RO>(bool_state.start, bool_state.length, buf_builder, states);
          break;
        case BoolIOStateType::RI:
          AppendBoolStateInterfaces<StatusGPIOTypes::RI>(bool_state.start, bool_state.length, buf_builder, states);
          break;
        case BoolIOStateType::F:
          AppendBoolStateInterfaces<StatusGPIOTypes::F>(bool_state.start, bool_state.length, buf_builder, states);
          break;
      }
    }
  }

  using ::gpio_config::AnalogIOCmdType;
  if (config.analog_io_cmd.has_value())
  {
    for (const gpio_config::AnalogIOCmdConfig& analog_cmd : *config.analog_io_cmd)
    {
      if (analog_cmd.type == AnalogIOCmdType::AO)
      {
        AppendAnalogCmdInterfaces<CommandGPIOTypes::AO>(analog_cmd.start, analog_cmd.length, buf_builder, commands);
      }
    }
  }

  using ::gpio_config::AnalogIOStateType;
  if (config.analog_io_state.has_value())
  {
    for (const gpio_config::AnalogIOStateConfig& analog_state : *config.analog_io_state)
    {
      switch (analog_state.type)
      {
        case AnalogIOStateType::AO:
          AppendAnalogStateInterfaces<StatusGPIOTypes::AO>(analog_state.start, analog_state.length, buf_builder, states);
          break;
        case AnalogIOStateType::AI:
          AppendAnalogStateInterfaces<StatusGPIOTypes::AI>(analog_state.start, analog_state.length, buf_builder, states);
          break;
      }
    }
  }

  if (config.num_reg_cmd.has_value())
  {
    for (const gpio_config::NumRegConfig& num_reg_cmd : *config.num_reg_cmd)
    {
      const uint32_t start = num_reg_cmd.start;
      const uint32_t length = num_reg_cmd.length;
      auto& block = buf_builder.addCommandConfig<CommandGPIOTypes::FloatReg, float>(start, length);
      for (uint32_t index = start; index < start + length; ++index)
      {
        commands.emplace_back(std::make_unique<IOCommand<CommandGPIOTypes::FloatReg, float>>(index, block));
      }
    }
  }

  if (config.num_reg_state.has_value())
  {
    for (const gpio_config::NumRegConfig& num_reg_state : *config.num_reg_state)
    {
      const uint32_t start = num_reg_state.start;
      const uint32_t length = num_reg_state.length;
      auto& block = buf_builder.addStatusConfig<StatusGPIOTypes::FloatReg, float>(start, length);
      for (uint32_t index = start; index < start + length; ++index)
      {
        states.emplace_back(std::make_unique<IOState<StatusGPIOTypes::FloatReg, float>>(index, block));
      }
    }
  }

  return { std::move(commands), std::move(states), buf_builder.build() };
}

}  // namespace

FanucHardwareInterface::FanucHardwareInterface()
  : fr_joint_pos_{ Eigen::VectorXd::Zero(9) }
  , fr_prev_joint_pos_{ Eigen::VectorXd::Zero(9) }
  , fr_joint_vel_{ Eigen::VectorXd::Zero(9) }
  , joint_targets_{ Eigen::VectorXd::Zero(9) }
  , joint_targets_degrees_{ Eigen::VectorXd::Zero(9) }
  , stream_motion_port_(60015)
  , rmi_port_(1600)
{
}

FanucHardwareInterface::~FanucHardwareInterface() = default;

hardware_interface::CallbackReturn FanucHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  info_ = info;

  // Parse and configure cyclic GPIO from the yaml config.
  const auto config_path_it = info_.hardware_parameters.find("gpio_configuration");
  if (config_path_it != info_.hardware_parameters.end() && !config_path_it->second.empty())
  {
    gpio_config::GPIOConfig gpio_config;
    RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface),
                       "Loading GPIO configuration file: " << config_path_it->second);
    try
    {
      gpio_config = gpio_config::ParseGPIOConfig(std::filesystem::path(config_path_it->second));
    }
    catch (std::runtime_error& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(kFRHWInterface), "Failed to parse gpio_config: %s", e.what());
      return CallbackReturn::ERROR;
    }

    IOInterfaces interfaces = GPIOConfigToInterfaces(gpio_config.gpio_topic_config);
    io_commands_ = std::move(interfaces.commands);
    io_state_ = std::move(interfaces.states);
    gpio_buffer_ = std::make_shared<fanuc_client::GPIOBuffer>(std::move(interfaces.buffer));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FanucHardwareInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "FANUC ROS2 HW interface v3");
  ip_address_ = info_.hardware_parameters["robot_ip"];
  try
  {
    rmi_port_ = StringToInt("rmi_port", info_.hardware_parameters["rmi_port"]);
    stream_motion_port_ = StringToInt("stream_motion_port", info_.hardware_parameters["stream_motion_port"]);
    payload_schedule_ = StringToInt("payload_schedule", info_.hardware_parameters["payload_schedule"]);
    out_cmd_interp_buff_target_ =
        StringToInt("out_cmd_interp_buff_target", info_.hardware_parameters["out_cmd_interp_buff_target"]);
  }
  catch (const std::exception& e)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "payload_schedule: " << payload_schedule_);
  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "Starting RMI with: " << ip_address_);

  // Initialize the driver client
  for (int i = 0; i < kNumberConnectionAttempts; i++)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "Connecting to the robot: attempt: " << i);
    try
    {
      fanuc_client_.reset();
      fanuc_client_ = std::make_unique<fanuc_client::FanucClient>(ip_address_, stream_motion_port_, rmi_port_);
      fanuc_client_->setOutCmdInterpBuffTarget(out_cmd_interp_buff_target_);
      fanuc_client_->startRMI();
      fanuc_client_->setPayloadSchedule(payload_schedule_);
      fanuc_client_->validateGPIOBuffer(gpio_buffer_);
      RCLCPP_INFO(rclcpp::get_logger(kFRHWInterface), "Successfully connected to the robot.");
      return CallbackReturn::SUCCESS;
    }
    catch (std::runtime_error& e)
    {
      RCLCPP_WARN(rclcpp::get_logger(kFRHWInterface), "%s", e.what());
      rclcpp::sleep_for(std::chrono::milliseconds(3000));
    }
  }

  RCLCPP_ERROR(rclcpp::get_logger(kFRHWInterface), "Failed to connect to the robot.");
  return CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn FanucHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger(kFRHWInterface), "activating hardware interface");

  fanuc_client_->startRealtimeStream(gpio_buffer_);
  joint_targets_degrees_ = fanuc_client_->readJointAngles();
  joint_targets_.array() = M_PI / 180.0 * joint_targets_degrees_.array();

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FanucHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "deactivating stream motion");
  fanuc_client_->stopRealtimeStream();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FanucHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  fanuc_client_.reset();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FanucHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(2 * info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &fr_joint_pos_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &fr_joint_vel_[i]);
  }

  for (const auto& io_state : io_state_)
  {
    state_interfaces.emplace_back(io_state->name(), std::to_string(io_state->index), &io_state->value);
  }

  state_interfaces.emplace_back(kRobotStatusInterfaceName, kStatusInErrorType, &robot_status_.in_error);
  state_interfaces.emplace_back(kRobotStatusInterfaceName, kStatusTPEnabledType, &robot_status_.tp_enabled);
  state_interfaces.emplace_back(kRobotStatusInterfaceName, kStatusEStoppedType, &robot_status_.e_stopped);
  state_interfaces.emplace_back(kRobotStatusInterfaceName, kStatusMotionPossibleType, &robot_status_.motion_possible);
  state_interfaces.emplace_back(kRobotStatusInterfaceName, kStatusContactStopModeType, &robot_status_.contact_stop_mode);
  state_interfaces.emplace_back(kRobotStatusInterfaceName, kStatusCollaborativeSpeedScalingType,
                                &robot_status_.collaborative_speed_scaling);

  state_interfaces.emplace_back(kConnectionStatusName, kIsConnectedType, &robot_status_.is_connected);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FanucHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(info_.joints.size()); ++i)
  {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_targets_[i]);
  }

  for (const auto& io_command : io_commands_)
  {
    command_interfaces.emplace_back(io_command->name(), std::to_string(io_command->index), &io_command->value);
  }

  return command_interfaces;
}

hardware_interface::return_type FanucHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& period)
{
  robot_status_.is_connected = fanuc_client_ != nullptr && fanuc_client_->isStreaming();
  if (!robot_status_.is_connected)
  {
    fanuc_client_->stopRealtimeStream();
    RCLCPP_ERROR(rclcpp::get_logger(kFRHWInterface), "FANUC ROS2 HW no longer streaming.");
    return hardware_interface::return_type::ERROR;
  }

  fr_prev_joint_pos_ = fr_joint_pos_;
  const Eigen::Ref<const Eigen::VectorXd> joint_angles = fanuc_client_->readJointAngles();
  for (Eigen::Index i = 0; i < joint_angles.size(); ++i)
  {
    fr_joint_pos_[i] = M_PI / 180.0 * joint_angles[i];
  }
  if ((fr_prev_joint_pos_.array() != fr_joint_pos_.array()).any())
  {
    const double dt = static_cast<double>(fanuc_client_->getControlPeriod()) / 1000.0;
    fr_joint_vel_ = (fr_joint_pos_ - fr_prev_joint_pos_) / dt;
  }

  for (const auto& io_state : io_state_)
  {
    io_state->updateValue();
  }

  robot_status_.in_error = fanuc_client_->robot_status().in_error;
  robot_status_.tp_enabled = fanuc_client_->robot_status().tp_enabled;
  robot_status_.e_stopped = fanuc_client_->robot_status().e_stopped;
  robot_status_.motion_possible = fanuc_client_->robot_status().motion_possible;
  robot_status_.contact_stop_mode = static_cast<double>(fanuc_client_->robot_status().contact_stop_mode);
  robot_status_.collaborative_speed_scaling = static_cast<double>(fanuc_client_->robot_status().safety_scale);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FanucHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  robot_status_.is_connected = fanuc_client_ != nullptr && fanuc_client_->isStreaming();
  if (!robot_status_.is_connected)
  {
    fanuc_client_->stopRealtimeStream();
    RCLCPP_ERROR(rclcpp::get_logger(kFRHWInterface), "FANUC ROS2 HW no longer streaming.");
    return hardware_interface::return_type::ERROR;
  }
  joint_targets_degrees_.array() = 180.0 / M_PI * joint_targets_.array();
  fanuc_client_->writeJointTarget(joint_targets_degrees_);

  for (const auto& io_command : io_commands_)
  {
    io_command->updateBuffer();
  }
  fanuc_client_->sendIOCommand();

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn FanucHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}
}  // namespace fanuc_robot_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fanuc_robot_driver::FanucHardwareInterface, hardware_interface::SystemInterface)
