// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "fanuc_robot_driver/hardware_interface.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "fanuc_client/gpio_buffer.hpp"
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
  , motion_command_type_(MotionCommandType::InitialState)
  , motion_command_type_dbl_(0.0)
  , hw_active_(false)
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
  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "Preparing FANUC ROS2 HW interface");
  hw_active_.store(false);
  ip_address_ = info_.hardware_parameters["robot_ip"];
  bool initial_motion_control = true;
  try
  {
    rmi_port_ = StringToInt("rmi_port", info_.hardware_parameters["rmi_port"]);
    stream_motion_port_ = StringToInt("stream_motion_port", info_.hardware_parameters["stream_motion_port"]);
    payload_schedule_ = StringToInt("payload_schedule", info_.hardware_parameters["payload_schedule"]);
    out_cmd_interp_buff_target_ =
        StringToInt("out_cmd_interp_buff_target", info_.hardware_parameters["out_cmd_interp_buff_target"]);
    force_sensor_type_ = StringToInt("force_sensor_type", info_.hardware_parameters["force_sensor_type"]);
    initial_motion_control = (StringToInt("motion_control", info_.hardware_parameters["motion_control"]) == 1);
    auto it = info_.hardware_parameters.find("encoding");
    if (it != info_.hardware_parameters.end())
    {
      encoding_ = info_.hardware_parameters["encoding"];
    }
    else
    {
      encoding_ = "UTF-8";
    }
  }
  catch (const std::exception& e)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "payload_schedule: " << payload_schedule_);
  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "Starting RMI with: " << ip_address_);
  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "Initial Motion Control Mode: " << initial_motion_control);
  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "Encoding: " << encoding_);

  // Initialize the driver client
  for (int i = 0; i < kNumberConnectionAttempts; i++)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "Connecting to the robot: attempt: " << i);
    try
    {
      fanuc_client_.reset();
      fanuc_client_ = std::make_unique<fanuc_client::FanucClient>(ip_address_, stream_motion_port_, rmi_port_, nullptr,
                                                                  nullptr, encoding_);
      fanuc_client_->setDoMotnCtrl(initial_motion_control);
      fanuc_client_->setOutCmdInterpBuffTarget(out_cmd_interp_buff_target_);
      fanuc_client_->setForceSensorType(force_sensor_type_);
      if (initial_motion_control)
      {
        fanuc_client_->startRMI();
      }
      fanuc_client_->setPayloadSchedule(payload_schedule_);
      fanuc_client_->validateGPIOBuffer(gpio_buffer_);
      RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "Successfully connected to the robot.");
      RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface),
                         "FANUC ROS2 HW interface is ready with client version: " << fanuc_client_->getClientVersion());
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
  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "activating hardware interface");

  fanuc_client_->startRealtimeStream(gpio_buffer_);
  joint_targets_degrees_ = fanuc_client_->readJointAngles();
  joint_targets_.array() = M_PI / 180.0 * joint_targets_degrees_.array();

  hw_active_.store(true);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FanucHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger(kFRHWInterface), "deactivating stream motion");
  hw_active_.store(false);
  fanuc_client_->stopRealtimeStream();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FanucHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  hw_active_.store(false);
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
  state_interfaces.emplace_back(kConnectionStatusName, kMotionCommandType, &motion_command_type_dbl_);

  state_interfaces.emplace_back(kForceInterfaceName, kForceXType, &force_sensor_.force_x);
  state_interfaces.emplace_back(kForceInterfaceName, kForceYType, &force_sensor_.force_y);
  state_interfaces.emplace_back(kForceInterfaceName, kForceZType, &force_sensor_.force_z);
  state_interfaces.emplace_back(kForceInterfaceName, kMomentXType, &force_sensor_.moment_x);
  state_interfaces.emplace_back(kForceInterfaceName, kMomentYType, &force_sensor_.moment_y);
  state_interfaces.emplace_back(kForceInterfaceName, kMomentZType, &force_sensor_.moment_z);
  state_interfaces.emplace_back(kForceInterfaceName, kForceSensorType, &force_sensor_.fs_type);

  // For force_torque_sensor_broadcaster (geometry_msgs/WrenchStamped)
  state_interfaces.emplace_back("ft_sensor", "force.x", &force_sensor_.force_x);
  state_interfaces.emplace_back("ft_sensor", "force.y", &force_sensor_.force_y);
  state_interfaces.emplace_back("ft_sensor", "force.z", &force_sensor_.force_z);
  state_interfaces.emplace_back("ft_sensor", "torque.x", &force_sensor_.moment_x);
  state_interfaces.emplace_back("ft_sensor", "torque.y", &force_sensor_.moment_y);
  state_interfaces.emplace_back("ft_sensor", "torque.z", &force_sensor_.moment_z);

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

  command_interfaces.emplace_back(kRMIInterfaceName, kRMICommandName, &dummy_rmi_command_);

  return command_interfaces;
}

hardware_interface::return_type
FanucHardwareInterface::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                    const std::vector<std::string>& stop_interfaces)
{
  bool start_rmi_control = false;
  bool stop_rmi_control = false;
  bool rmi_control_is_active = false;

  bool start_position_control = false;
  bool stop_position_control = false;
  bool position_control_is_active = false;
  int start_position_control_num = 0;

  int joints_size = info_.joints.size();
  std::vector<std::string> position_interface_names(joints_size);

  // List position interface names
  for (int ii = 0; ii < joints_size; ii++)
  {
    position_interface_names[ii] = info_.joints[ii].name + "/" + hardware_interface::HW_IF_POSITION;
  }

  // Check stop interfaces
  for (const std::string& command : stop_interfaces)
  {
    if (std::any_of(position_interface_names.begin(), position_interface_names.end(),
                    [&command](std::string& value) { return (value == command); }))
    {
      stop_position_control = true;
    }
    if (command == std::string(kRMIInterfaceName) + "/" + std::string(kRMICommandName))
    {
      stop_rmi_control = true;
    }
  }

  // Check requested interfaces
  for (const std::string& command : start_interfaces)
  {
    if (std::any_of(position_interface_names.begin(), position_interface_names.end(),
                    [&command](std::string& value) { return (value == command); }))
    {
      start_position_control_num += 1;
    }
    if (command == std::string(kRMIInterfaceName) + "/" + std::string(kRMICommandName))
    {
      start_rmi_control = true;
    }
  }

  // Check joint number
  if (start_position_control_num != 0)
  {
    if (start_position_control_num != joints_size)
    {
      RCLCPP_ERROR(rclcpp::get_logger(kFRHWInterface),
                   "Requested position command number does not match the joint size. %d %d", start_position_control_num,
                   joints_size);
      return hardware_interface::return_type::ERROR;
    }
    else
    {
      start_position_control = true;
    }
  }

  if (!(start_position_control || stop_position_control || start_rmi_control || stop_rmi_control))
  {
    RCLCPP_INFO(rclcpp::get_logger(kFRHWInterface), "There is no motion commands in this switching.");
    return hardware_interface::return_type::OK;
  }
  rmi_control_is_active = (motion_command_type_ == MotionCommandType::RMI);
  position_control_is_active = (motion_command_type_ == MotionCommandType::Position);
  RCLCPP_INFO(rclcpp::get_logger(kFRHWInterface), "RMI: start %d stop %d active %d", start_rmi_control,
              stop_rmi_control, rmi_control_is_active);
  RCLCPP_INFO(rclcpp::get_logger(kFRHWInterface), "POSITION: start %d stop %d active %d", start_position_control,
              stop_position_control, position_control_is_active);
  if ((start_rmi_control || (rmi_control_is_active && !stop_rmi_control)) &&
      (start_position_control || (position_control_is_active && !stop_position_control)))
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRHWInterface), "Position and RMI commands are requested simultaneously.");
    return hardware_interface::return_type::ERROR;
  }

  // Stop commands
  if (stop_position_control || (start_rmi_control && (motion_command_type_ == MotionCommandType::InitialState)))
  {
    if (fanuc_client_ != nullptr)
    {
      // Stop STMO
      auto const motion_control = fanuc_client_->getDoMotnCtrl();
      try
      {
        fanuc_client_->stopMotionControl();  // motion_control_ is set to false internally.
      }
      catch (const std::runtime_error& e)
      {
        // just continue.
      }
      fanuc_client_->setDoMotnCtrl(motion_control);  // restore motion_control after stopMotionControl;
    }
  }

  // Start commands
  if (start_position_control)
  {
    if (motion_command_type_ != MotionCommandType::InitialState)
    {
      if (fanuc_client_->getDoMotnCtrl())
      {
        fanuc_client_->startMotionControl();
      }
    }
  }

  // Save command type
  if (start_position_control)
  {
    motion_command_type_ = MotionCommandType::Position;
  }
  else if (start_rmi_control)
  {
    motion_command_type_ = MotionCommandType::RMI;
  }
  else
  {
    if (stop_position_control || stop_rmi_control)
    {
      motion_command_type_ = MotionCommandType::None;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
FanucHardwareInterface::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                    const std::vector<std::string>& stop_interfaces)
{
  // nothing to do
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FanucHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& period)
{
  motion_command_type_dbl_ = static_cast<double>(static_cast<int>(motion_command_type_.load()));
  robot_status_.is_connected = fanuc_client_ != nullptr && fanuc_client_->isStreaming();
  if (!hw_active_.load())
  {
    // is_connected should be updated beforehand.
    // Do nothing
    return hardware_interface::return_type::OK;
  }
  if (!robot_status_.is_connected)
  {
    if (fanuc_client_ != nullptr)
    {
      try
      {
        fanuc_client_->stopRealtimeStream();
      }
      catch (const std::runtime_error& e)
      {
        RCLCPP_DEBUG(rclcpp::get_logger(kFRHWInterface), "Stream already stopped: %s", e.what());
      }
      catch (...)
      {
        // Catch any other exceptions during shutdown
        RCLCPP_DEBUG(rclcpp::get_logger(kFRHWInterface), "Exception during stream shutdown (likely normal)");
      }
    }

    static auto last_log_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() > 5000)
    {
      RCLCPP_WARN(rclcpp::get_logger(kFRHWInterface),
                  "FANUC ROS2 HW no longer streaming (this is normal during shutdown).");
      last_log_time = now;
    }
    return hardware_interface::return_type::ERROR;
  }

  try
  {
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

    force_sensor_.force_x = static_cast<double>(fanuc_client_->force_sensor().force_x);
    force_sensor_.force_y = static_cast<double>(fanuc_client_->force_sensor().force_y);
    force_sensor_.force_z = static_cast<double>(fanuc_client_->force_sensor().force_z);
    force_sensor_.moment_x = static_cast<double>(fanuc_client_->force_sensor().moment_x);
    force_sensor_.moment_y = static_cast<double>(fanuc_client_->force_sensor().moment_y);
    force_sensor_.moment_z = static_cast<double>(fanuc_client_->force_sensor().moment_z);
    force_sensor_.fs_type = static_cast<double>(fanuc_client_->force_sensor().fs_type);
  }
  catch (const std::exception& e)
  {
    RCLCPP_DEBUG(rclcpp::get_logger(kFRHWInterface), "Exception during read (likely shutdown): %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  catch (...)
  {
    RCLCPP_DEBUG(rclcpp::get_logger(kFRHWInterface), "Unknown exception during read (likely shutdown)");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FanucHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  robot_status_.is_connected = fanuc_client_ != nullptr && fanuc_client_->isStreaming();
  if (!hw_active_.load())
  {
    // Do nothing
    return hardware_interface::return_type::OK;
  }
  if (!robot_status_.is_connected)
  {
    if (fanuc_client_ != nullptr)
    {
      try
      {
        fanuc_client_->stopRealtimeStream();
      }
      catch (const std::runtime_error& e)
      {
        RCLCPP_DEBUG(rclcpp::get_logger(kFRHWInterface), "Stream already stopped: %s", e.what());
      }
      catch (...)
      {
        RCLCPP_DEBUG(rclcpp::get_logger(kFRHWInterface), "Exception during stream shutdown (likely normal)");
      }
    }
    // Throttle to avoid spam during shutdown
    static auto last_log_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() > 5000)
    {
      RCLCPP_WARN(rclcpp::get_logger(kFRHWInterface),
                  "FANUC ROS2 HW no longer streaming (this is normal during shutdown).");
      last_log_time = now;
    }
    return hardware_interface::return_type::ERROR;
  }

  try
  {
    joint_targets_degrees_.array() = 180.0 / M_PI * joint_targets_.array();
    fanuc_client_->writeJointTarget(joint_targets_degrees_);

    for (const auto& io_command : io_commands_)
    {
      io_command->updateBuffer();
    }
    fanuc_client_->sendIOCommand();
  }
  catch (const std::exception& e)
  {
    // During shutdown, operations may fail - log but don't crash
    RCLCPP_DEBUG(rclcpp::get_logger(kFRHWInterface), "Exception during write (likely shutdown): %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  catch (...)
  {
    // Catch any other exceptions during shutdown
    RCLCPP_DEBUG(rclcpp::get_logger(kFRHWInterface), "Unknown exception during write (likely shutdown)");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn FanucHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}
}  // namespace fanuc_robot_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fanuc_robot_driver::FanucHardwareInterface, hardware_interface::SystemInterface)
