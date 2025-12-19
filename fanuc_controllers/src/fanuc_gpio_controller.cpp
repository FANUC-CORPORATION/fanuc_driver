// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "fanuc_controllers/fanuc_gpio_controller.hpp"

#include <algorithm>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "controller_interface/controller_interface_base.hpp"
#include "fanuc_client/fanuc_client.hpp"
#include "fanuc_robot_driver/constants.hpp"
#include "gpio_config/gpio_config.hpp"

namespace fanuc_controllers
{
constexpr auto kFRGPIOController = "FR_GPIO_Controller";

namespace
{
template <typename T>
void WriteMessageCallback(const T& msg, realtime_tools::RealtimeBuffer<T>& rt_buffer)
{
  rt_buffer.writeFromNonRT(msg);
}

std::shared_ptr<rmi::RMIConnectionInterface> getRMIInstance()
{
  return fanuc_client::RMISingleton::getRMIInstance();
}

void GetAnalogIO(const std::shared_ptr<fanuc_msgs::srv::GetAnalogIO::Request>& request,
                 const std::shared_ptr<fanuc_msgs::srv::GetAnalogIO::Response>& response)
{
  if (request->io_type.type != fanuc_msgs::msg::IOType::AO && request->io_type.type != fanuc_msgs::msg::IOType::AI)
  {
    response->result = 1;
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController),
                 std::string("GetAnalogIO does not support the type: ").append(request->io_type.type).c_str());
    return;
  }
  try
  {
    const rmi::ReadIOPortPacket::Response rmi_response =
        getRMIInstance()->readIOPort(request->io_type.type, request->index, 1.0);
    response->value = std::holds_alternative<float>(rmi_response.PortValue) ?
                          std::get<float>(rmi_response.PortValue) :
                          static_cast<float>(std::get<int>(rmi_response.PortValue));
    response->result = rmi_response.ErrorID;
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController), e.what());
    response->result = 1;
  }
}

void GetBoolIO(const std::shared_ptr<fanuc_msgs::srv::GetBoolIO::Request>& request,
               const std::shared_ptr<fanuc_msgs::srv::GetBoolIO::Response>& response)
{
  std::vector supported_types = { fanuc_msgs::msg::IOType::DI, fanuc_msgs::msg::IOType::RI, fanuc_msgs::msg::IOType::DO,
                                  fanuc_msgs::msg::IOType::RO, fanuc_msgs::msg::IOType::F };
  if (std::find(supported_types.begin(), supported_types.end(), request->io_type.type) == supported_types.end())
  {
    response->result = 1;
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController),
                 std::string("GetBoolIO does not support the type: ").append(request->io_type.type).c_str());
    return;
  }
  try
  {
    if (request->io_type.type == fanuc_msgs::msg::IOType::DI)
    {
      const rmi::ReadDigitalInputPortPacket::Response rmi_response =
          getRMIInstance()->readDigitalInputPort(request->index, 1.0);
      response->result = rmi_response.ErrorID;
      response->value = rmi_response.PortValue;
    }
    else
    {
      const std::string type = request->io_type.type == fanuc_msgs::msg::IOType::F ? "FLAG" : request->io_type.type;
      const rmi::ReadIOPortPacket::Response rmi_response = getRMIInstance()->readIOPort(type, request->index, 1.0);
      response->value = std::holds_alternative<int>(rmi_response.PortValue) ?
                            std::get<int>(rmi_response.PortValue) :
                            static_cast<int>(std::get<float>(rmi_response.PortValue));
      response->result = rmi_response.ErrorID;
    }
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController), e.what());
    response->result = 1;
  }
}

void GetNumReg(const std::shared_ptr<fanuc_msgs::srv::GetNumReg::Request>& request,
               const std::shared_ptr<fanuc_msgs::srv::GetNumReg::Response>& response)
{
  try
  {
    response->result = 1;
    rmi::ReadNumericRegisterPacket::Response rmi_response = getRMIInstance()->readNumericRegister(request->index, 1.0);
    response->value = std::holds_alternative<float>(rmi_response.RegisterValue) ?
                          std::get<float>(rmi_response.RegisterValue) :
                          static_cast<float>(std::get<int>(rmi_response.RegisterValue));
    response->result = rmi_response.ErrorID;
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController), e.what());
    response->result = 1;
  }
}

void GetPosReg(const std::shared_ptr<fanuc_msgs::srv::GetPosReg::Request>& request,
               const std::shared_ptr<fanuc_msgs::srv::GetPosReg::Response>& response)
{
  try
  {
    response->result = 1;
    rmi::ReadPositionRegisterPacket::Response rmi_response =
        getRMIInstance()->readPositionRegister(request->index, 1.0);
    response->representation = rmi_response.Representation;
    if (rmi_response.Configuration.has_value())
    {
      response->utool = rmi_response.Configuration->UToolNumber;
      response->uframe = rmi_response.Configuration->UFrameNumber;
      response->front = rmi_response.Configuration->Front;
      response->up = rmi_response.Configuration->Up;
      response->left = rmi_response.Configuration->Left;
      response->flip = rmi_response.Configuration->Flip;
      response->turn4 = rmi_response.Configuration->Turn4;
      response->turn5 = rmi_response.Configuration->Turn5;
      response->turn6 = rmi_response.Configuration->Turn6;
    }
    if (rmi_response.Position.has_value())
    {
      response->x = rmi_response.Position->X;
      response->y = rmi_response.Position->Y;
      response->z = rmi_response.Position->Z;
      response->w = rmi_response.Position->W;
      response->p = rmi_response.Position->P;
      response->r = rmi_response.Position->R;
      response->ext1 = rmi_response.Position->Ext1;
      response->ext2 = rmi_response.Position->Ext2;
      response->ext3 = rmi_response.Position->Ext3;
    }
    if (rmi_response.JointAngle.has_value())
    {
      response->j1 = rmi_response.JointAngle->J1;
      response->j2 = rmi_response.JointAngle->J2;
      response->j3 = rmi_response.JointAngle->J3;
      response->j4 = rmi_response.JointAngle->J4;
      response->j5 = rmi_response.JointAngle->J5;
      response->j6 = rmi_response.JointAngle->J6;
      response->j7 = rmi_response.JointAngle->J7;
      response->j8 = rmi_response.JointAngle->J8;
      response->j9 = rmi_response.JointAngle->J9;
    }
    response->result = rmi_response.ErrorID;
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController), e.what());
    response->result = 1;
  }
}

void GetGroupIO(const std::shared_ptr<fanuc_msgs::srv::GetGroupIO::Request>& request,
                const std::shared_ptr<fanuc_msgs::srv::GetGroupIO::Response>& response)
{
  try
  {
    if (request->io_type.type != fanuc_msgs::msg::IOType::GI && request->io_type.type != fanuc_msgs::msg::IOType::GO)
    {
      response->result = 1;
      RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController),
                   std::string("GetGroupIO does not support the type: ").append(request->io_type.type).c_str());
      return;
    }
    const rmi::ReadIOPortPacket::Response rmi_response =
        getRMIInstance()->readIOPort(request->io_type.type, request->index, 1.0);
    response->value = std::holds_alternative<float>(rmi_response.PortValue) ?
                          static_cast<uint16_t>(std::get<float>(rmi_response.PortValue)) :
                          static_cast<uint16_t>(std::get<int>(rmi_response.PortValue));
    response->result = rmi_response.ErrorID;
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController), e.what());
    response->result = 1;
  }
}

void SetAnalogIO(const std::shared_ptr<fanuc_msgs::srv::SetAnalogIO::Request>& request,
                 const std::shared_ptr<fanuc_msgs::srv::SetAnalogIO::Response>& response)
{
  if (request->io_type.type != fanuc_msgs::msg::IOType::AO)
  {
    response->result = 1;
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController),
                 std::string("SetAnalogIO does not support the type: ").append(request->io_type.type).c_str());
    return;
  }
  try
  {
    rmi::WriteIOPortPacket::Response rmi_response =
        getRMIInstance()->writeIOPort(request->index, request->io_type.type, request->value, 1.0);
    response->result = rmi_response.ErrorID;
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController), e.what());
    response->result = 1;
  }
}

void SetBoolIO(const std::shared_ptr<fanuc_msgs::srv::SetBoolIO::Request>& request,
               const std::shared_ptr<fanuc_msgs::srv::SetBoolIO::Response>& response)
{
  std::vector supported_types = { fanuc_msgs::msg::IOType::DO, fanuc_msgs::msg::IOType::RO, fanuc_msgs::msg::IOType::F };
  if (std::find(supported_types.begin(), supported_types.end(), request->io_type.type) == supported_types.end())
  {
    response->result = 1;
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController),
                 std::string("SetBoolIO does not support the type: ").append(request->io_type.type).c_str());
    return;
  }
  try
  {
    if (request->io_type.type == fanuc_msgs::msg::IOType::DI)
    {
      rmi::WriteDigitalOutputPacket::Response rmi_response =
          getRMIInstance()->writeDigitalOutputPort(request->index, request->value, 1.0);
      response->result = rmi_response.ErrorID;
    }
    else
    {
      const std::string type = request->io_type.type == fanuc_msgs::msg::IOType::F ? "FLAG" : request->io_type.type;
      rmi::WriteIOPortPacket::Response rmi_response =
          getRMIInstance()->writeIOPort(request->index, type, request->value, 1.0);
      response->result = rmi_response.ErrorID;
    }
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController), e.what());
    response->result = 1;
  }
}

void SetGenOverride(const std::shared_ptr<fanuc_msgs::srv::SetGenOverride::Request>& request,
                    const std::shared_ptr<fanuc_msgs::srv::SetGenOverride::Response>& response)
{
  try
  {
    rmi::SetSpeedOverridePacket::Response rmi_response = getRMIInstance()->setSpeedOverride(request->value, 1.0);
    response->result = rmi_response.ErrorID;
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController), e.what());
    response->result = 1;
  }
}

void SetGroupIO(const std::shared_ptr<fanuc_msgs::srv::SetGroupIO::Request>& request,
                const std::shared_ptr<fanuc_msgs::srv::SetGroupIO::Response>& response)
{
  if (request->io_type.type != fanuc_msgs::msg::IOType::GO)
  {
    response->result = 1;
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController),
                 std::string("SetGroupIO does not support the type: ").append(request->io_type.type).c_str());
    return;
  }
  try
  {
    rmi::WriteIOPortPacket::Response rmi_response =
        getRMIInstance()->writeIOPort(request->index, request->io_type.type, request->value, 1.0);
    response->result = rmi_response.ErrorID;
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController), e.what());
    response->result = 1;
  }
}

void SetNumReg(const std::shared_ptr<fanuc_msgs::srv::SetNumReg::Request>& request,
               const std::shared_ptr<fanuc_msgs::srv::SetNumReg::Response>& response)
{
  try
  {
    const rmi::WriteNumericRegisterPacket::Response rmi_response =
        getRMIInstance()->writeNumericRegister(request->index, request->value, 1.0);
    response->result = rmi_response.ErrorID;
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController), e.what());
    response->result = 1;
  }
}

void SetPosReg(const std::shared_ptr<fanuc_msgs::srv::SetPosReg::Request>& request,
               const std::shared_ptr<fanuc_msgs::srv::SetPosReg::Response>& response)
{
  rmi::ConfigurationData configuration;
  rmi::PositionData position;
  rmi::JointAngleData joint_angle;
  configuration.UToolNumber = request->utool;
  configuration.UFrameNumber = request->uframe;
  configuration.Front = std::min(int(request->front), 1);
  configuration.Up = std::min(int(request->up), 1);
  configuration.Left = std::min(int(request->left), 1);
  configuration.Flip = std::min(int(request->flip), 1);
  configuration.Turn4 = request->turn4;
  configuration.Turn5 = request->turn5;
  configuration.Turn6 = request->turn6;
  position.X = request->x;
  position.Y = request->y;
  position.Z = request->z;
  position.W = request->w;
  position.P = request->p;
  position.R = request->r;
  position.Ext1 = request->ext1;
  position.Ext2 = request->ext2;
  position.Ext3 = request->ext3;
  joint_angle.J1 = request->j1;
  joint_angle.J2 = request->j2;
  joint_angle.J3 = request->j3;
  joint_angle.J4 = request->j4;
  joint_angle.J5 = request->j5;
  joint_angle.J6 = request->j6;
  joint_angle.J7 = request->j7;
  joint_angle.J8 = request->j8;
  joint_angle.J9 = request->j9;
  try
  {
    const rmi::WritePositionRegisterPacket::Response rmi_response = getRMIInstance()->writePositionRegister(
        request->index, request->representation, configuration, position, joint_angle, 1.0);
    response->result = rmi_response.ErrorID;
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController), e.what());
    response->result = 1;
  }
}

void SetPayloadID(const std::shared_ptr<fanuc_msgs::srv::SetPayloadID::Request>& request,
                  const std::shared_ptr<fanuc_msgs::srv::SetPayloadID::Response>& response)
{
  try
  {
    const rmi::SetPayloadPacket::Response rmi_response =
        getRMIInstance()->setPayloadSchedule(request->payload_schedule_id, 1.0);
    response->result = rmi_response.ErrorID;
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController), e.what());
    response->result = 1;
  }
}

void SetPayloadValue(const std::shared_ptr<fanuc_msgs::srv::SetPayloadValue::Request>& request,
                     const std::shared_ptr<fanuc_msgs::srv::SetPayloadValue::Response>& response)
{
  try
  {
    const rmi::SetPayloadValuePacket::Response rmi_response =
        getRMIInstance()->setPayloadValue(request->payload_schedule_id, request->mass, request->cg_x, request->cg_y,
                                          request->cg_z, request->use_in, request->in_x, request->in_y, request->in_z,
                                          1.0);
    response->result = rmi_response.ErrorID;
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController), e.what());
    response->result = 1;
  }
}

void SetPayloadComp(const std::shared_ptr<fanuc_msgs::srv::SetPayloadComp::Request>& request,
                    const std::shared_ptr<fanuc_msgs::srv::SetPayloadComp::Response>& response)
{
  try
  {
    const rmi::SetPayloadCompPacket::Response rmi_response =
        getRMIInstance()->setPayloadComp(request->payload_schedule_id, request->mass, request->cg_x, request->cg_y,
                                         request->cg_z, request->in_x, request->in_y, request->in_z, 1.0);
    response->result = rmi_response.ErrorID;
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRGPIOController), e.what());
    response->result = 1;
  }
}

fanuc_msgs::msg::IOType ToROSMsg(const gpio_config::BoolIOStateType& bool_io_state)
{
  fanuc_msgs::msg::IOType msg;
  switch (bool_io_state)
  {
    case gpio_config::BoolIOStateType::DI:
      msg.type = fanuc_msgs::msg::IOType::DI;
      break;
    case gpio_config::BoolIOStateType::RI:
      msg.type = fanuc_msgs::msg::IOType::RI;
      break;
    case gpio_config::BoolIOStateType::DO:
      msg.type = fanuc_msgs::msg::IOType::DO;
      break;
    case gpio_config::BoolIOStateType::RO:
      msg.type = fanuc_msgs::msg::IOType::RO;
      break;
    case gpio_config::BoolIOStateType::F:
      msg.type = fanuc_msgs::msg::IOType::F;
      break;
  }
  return msg;
}

fanuc_msgs::msg::IOType ToROSMsg(const gpio_config::BoolIOCmdType& bool_io_cmd)
{
  fanuc_msgs::msg::IOType msg;
  switch (bool_io_cmd)
  {
    case gpio_config::BoolIOCmdType::DO:
      msg.type = fanuc_msgs::msg::IOType::DO;
      break;
    case gpio_config::BoolIOCmdType::RO:
      msg.type = fanuc_msgs::msg::IOType::RO;
      break;
    case gpio_config::BoolIOCmdType::F:
      msg.type = fanuc_msgs::msg::IOType::F;
      break;
  }
  return msg;
}

std::string ToString(const gpio_config::BoolIOCmdType& bool_io_cmd)
{
  switch (bool_io_cmd)
  {
    case gpio_config::BoolIOCmdType::DO:
      return fanuc_msgs::msg::IOType::DO;
    case gpio_config::BoolIOCmdType::RO:
      return fanuc_msgs::msg::IOType::RO;
    case gpio_config::BoolIOCmdType::F:
      return fanuc_msgs::msg::IOType::F;
  }
  return "";
}

std::string ToString(const gpio_config::BoolIOStateType& bool_io_state)
{
  switch (bool_io_state)
  {
    case gpio_config::BoolIOStateType::DI:
      return fanuc_msgs::msg::IOType::DI;
    case gpio_config::BoolIOStateType::RI:
      return fanuc_msgs::msg::IOType::RI;
    case gpio_config::BoolIOStateType::DO:
      return fanuc_msgs::msg::IOType::DO;
    case gpio_config::BoolIOStateType::RO:
      return fanuc_msgs::msg::IOType::RO;
    case gpio_config::BoolIOStateType::F:
      return fanuc_msgs::msg::IOType::F;
  }
  return "";
}

std::string ToString(const gpio_config::AnalogIOStateType& bool_io_state)
{
  switch (bool_io_state)
  {
    case gpio_config::AnalogIOStateType::AO:
      return fanuc_msgs::msg::IOType::AO;
    case gpio_config::AnalogIOStateType::AI:
      return fanuc_msgs::msg::IOType::AI;
  }
  return "";
}

std::filesystem::path GetFilePath(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node)
{
  // Check for the ROS parameter "gpio_config_file"
  std::string gpio_config_file_package;
  node->get_parameter<std::string>("gpio_config_file_package", gpio_config_file_package);
  std::string gpio_config_file_path;
  node->get_parameter<std::string>("gpio_config_file_path", gpio_config_file_path);
  auto gpio_config_file =
      std::filesystem::path(ament_index_cpp::get_package_share_directory(gpio_config_file_package)) /
      gpio_config_file_path;
  return gpio_config_file;
}

std::string IOTypeToString(const fanuc_msgs::msg::IOType& type, const uint32_t index)
{
  return type.type + std::to_string(index);
}
}  // namespace

controller_interface::CallbackReturn FanucGPIOController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration FanucGPIOController::command_interface_configuration() const
{
  return command_interface_configuration_;
}

controller_interface::InterfaceConfiguration FanucGPIOController::state_interface_configuration() const
{
  return state_interface_configuration_;
}

void FanucGPIOController::publishRobotStatusExt()
{
  try
  {
    // Early return if shutting down
    if (shutting_down_.load())
    {
      return;
    }

    // Check if publisher is still valid (ROS2 context might be shutting down)
    if (!robot_status_ext_publisher_)
    {
      return;
    }

    // Check if node context is still valid
    auto node = get_node();
    if (!node)
    {
      return;
    }

    // Check context validity - this might throw during shutdown
    bool context_valid = false;
    try
    {
      context_valid = node->get_node_base_interface()->get_context()->is_valid();
    }
    catch (...)
    {
      // Context is being destroyed
      return;
    }

    if (!context_valid)
    {
      return;
    }

    // Try to get RMI instance and publish - all operations might fail during shutdown
    rmi::GetExtendedStatusPacket::Response robot_status_ext = getRMIInstance()->getExtendedStatus(1.0);
    robot_status_ext_msg_.drives_powered = robot_status_ext.DrivesPowered;
    robot_status_ext_msg_.error_code = robot_status_ext.ErrorCode.has_value() ? robot_status_ext.ErrorCode.value() : "";
    robot_status_ext_msg_.gen_override = robot_status_ext.GenOverride;
    robot_status_ext_msg_.in_motion = robot_status_ext.InMotion;
    robot_status_ext_msg_.speed_clamp_limit =
        robot_status_ext.SpeedClampLimit.has_value() ? robot_status_ext.SpeedClampLimit.value() : 0.0;
    robot_status_ext_msg_.control_mode =
        robot_status_ext.ControlMode.has_value() ? robot_status_ext.ControlMode.value() : "";

    // Final check before publishing - publisher might be invalid even if pointer is valid
    if (robot_status_ext_publisher_)
    {
      robot_status_ext_publisher_->publish(robot_status_ext_msg_);
    }
  }
  catch (...)
  {
    // Catch ALL exceptions including segfaults that manifest as exceptions
    // Silently ignore during shutdown
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FanucGPIOController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  // Reset shutdown flag when configuring
  shutting_down_.store(false);

  const gpio_config::GPIOTopicConfig gpio_topic_config =
      gpio_config::ParseGPIOConfig(GetFilePath(get_node())).gpio_topic_config;

  // Setup command interfaces
  command_interface_configuration_.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interface_configuration_.names.clear();

  size_t command_interface_index = 0;
  // Setup command interfaces using the parsed YAML
  if (gpio_topic_config.analog_io_cmd.has_value())
  {
    const auto& analog_io_cmd = gpio_topic_config.analog_io_cmd.value();
    const size_t total_size = std::accumulate(analog_io_cmd.begin(), analog_io_cmd.end(), 0,
                                              [](const size_t cum_sum, const gpio_config::AnalogIOCmdConfig& io_cmd_i) {
                                                return cum_sum + io_cmd_i.length;
                                              });
    analog_io_cmd_msg_.values.resize(total_size);
    index_analog_io_cmd_.resize(total_size);
    size_t offset = 0;
    for (const gpio_config::AnalogIOCmdConfig& analog_io_cmd_i : analog_io_cmd)
    {
      for (uint32_t i = 0; i < analog_io_cmd_i.length; ++i)
      {
        std::string name = std::string("AO/") + std::to_string(analog_io_cmd_i.start + i);
        command_interface_configuration_.names.push_back(name);
        index_analog_io_cmd_[i] = command_interface_index++;
        analog_io_cmd_msg_.values[i + offset].value = 0.0;
        analog_io_cmd_msg_.values[i + offset].index = analog_io_cmd_i.start + i;
        analog_io_cmd_msg_.values[i + offset].io_type.type = fanuc_msgs::msg::IOType::AO;
      }
      offset += analog_io_cmd_i.length;
    }
  }

  if (gpio_topic_config.io_cmd.has_value())
  {
    const auto& io_cmd = gpio_topic_config.io_cmd.value();
    const size_t total_size = std::accumulate(io_cmd.begin(), io_cmd.end(), size_t{},
                                              [](const size_t cum_sum, const gpio_config::BoolIOCmdConfig& io_cmd_i) {
                                                return cum_sum + io_cmd_i.length;
                                              });
    io_cmd_msg_.values.resize(total_size);
    index_io_cmd_.resize(total_size);
    size_t offset = 0;
    for (const gpio_config::BoolIOCmdConfig& io_cmd_i : io_cmd)
    {
      for (uint32_t i = 0; i < io_cmd_i.length; ++i)
      {
        std::stringstream name;
        name << ToString(io_cmd_i.type) << '/' << io_cmd_i.start + i;
        command_interface_configuration_.names.push_back(name.str());
        index_io_cmd_[i + offset] = command_interface_index++;
        io_cmd_msg_.values[i + offset].value = false;
        io_cmd_msg_.values[i + offset].index = io_cmd_i.start + i;
        io_cmd_msg_.values[i + offset].io_type = ToROSMsg(io_cmd_i.type);
      }
      offset += io_cmd_i.length;
    }
  }

  if (gpio_topic_config.num_reg_cmd.has_value())
  {
    const auto& num_reg_cmd = gpio_topic_config.num_reg_cmd.value();
    const size_t total_size = std::accumulate(num_reg_cmd.begin(), num_reg_cmd.end(), 0,
                                              [](const size_t cum_sum, const gpio_config::NumRegConfig& io_cmd_i) {
                                                return cum_sum + io_cmd_i.length;
                                              });
    num_reg_cmd_msg_.values.resize(total_size);
    index_num_reg_cmd_.resize(total_size);
    size_t offset = 0;
    for (const gpio_config::NumRegConfig& num_reg_cmd_i : num_reg_cmd)
    {
      for (uint32_t i = 0; i < num_reg_cmd_i.length; ++i)
      {
        std::string name = std::string("FloatReg/") + std::to_string(num_reg_cmd_i.start + i);
        command_interface_configuration_.names.push_back(name);
        index_num_reg_cmd_[i + offset] = command_interface_index++;
        num_reg_cmd_msg_.values[i + offset].value = 0.0;
        num_reg_cmd_msg_.values[i + offset].index = num_reg_cmd_i.start + i;
      }
      offset += num_reg_cmd_i.length;
    }
  }

  analog_io_cmd_msg_indexes_.clear();
  for (size_t i = 0; i < analog_io_cmd_msg_.values.size(); ++i)
  {
    analog_io_cmd_msg_indexes_.insert(
        { IOTypeToString(analog_io_cmd_msg_.values[i].io_type, analog_io_cmd_msg_.values[i].index), i });
  }
  io_cmd_msg_indexes_.clear();
  for (size_t i = 0; i < io_cmd_msg_.values.size(); ++i)
  {
    io_cmd_msg_indexes_.insert({ IOTypeToString(io_cmd_msg_.values[i].io_type, io_cmd_msg_.values[i].index), i });
  }
  num_reg_cmd_msg_indexes_.clear();
  for (size_t i = 0; i < num_reg_cmd_msg_.values.size(); ++i)
  {
    num_reg_cmd_msg_indexes_.insert({ num_reg_cmd_msg_.values[i].index, i });
  }

  // Setup state interfaces
  state_interface_configuration_.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interface_configuration_.names.clear();

  size_t state_interface_index = 0;
  // Setup state interfaces using the parsed YAML
  if (gpio_topic_config.analog_io_state)
  {
    const auto& analog_io_state = gpio_topic_config.analog_io_state.value();
    const size_t total_size =
        std::accumulate(analog_io_state.begin(), analog_io_state.end(), 0,
                        [](const size_t cum_sum, const gpio_config::AnalogIOStateConfig& io_cmd_i) {
                          return cum_sum + io_cmd_i.length;
                        });
    analog_io_state_msg_.values.resize(total_size);
    index_analog_io_state_.resize(total_size);
    size_t offset = 0;
    for (const gpio_config::AnalogIOStateConfig& analog_io_state_i : analog_io_state)
    {
      for (uint32_t i = 0; i < analog_io_state_i.length; ++i)
      {
        std::stringstream name;
        name << ToString(analog_io_state_i.type) << '/' << analog_io_state_i.start + i;
        state_interface_configuration_.names.push_back(name.str());
        index_analog_io_state_[i + offset] = state_interface_index++;
        analog_io_state_msg_.values[i + offset].io_type.type = ToString(analog_io_state_i.type);
        analog_io_state_msg_.values[i + offset].value = 0.0;
        analog_io_state_msg_.values[i + offset].index = analog_io_state_i.start + i;
      }
      offset += analog_io_state_i.length;
    }
  }

  if (gpio_topic_config.io_state)
  {
    const auto& io_state = gpio_topic_config.io_state.value();
    const size_t total_size = std::accumulate(io_state.begin(), io_state.end(), 0,
                                              [](const size_t cum_sum, const gpio_config::BoolIOStateConfig& io_cmd_i) {
                                                return cum_sum + io_cmd_i.length;
                                              });
    io_state_msg_.values.resize(total_size);
    index_io_state_.resize(total_size);
    size_t offset = 0;
    for (const gpio_config::BoolIOStateConfig& io_state_i : io_state)
    {
      for (uint32_t i = 0; i < io_state_i.length; ++i)
      {
        std::stringstream name;
        name << ToString(io_state_i.type) << '/' << io_state_i.start + i;
        state_interface_configuration_.names.push_back(name.str());
        index_io_state_[i + offset] = state_interface_index++;
        io_state_msg_.values[i + offset].io_type = ToROSMsg(io_state_i.type);
        io_state_msg_.values[i + offset].value = false;
        io_state_msg_.values[i + offset].index = io_state_i.start + i;
      }
      offset += io_state_i.length;
    }
  }

  if (gpio_topic_config.num_reg_state)
  {
    const auto& num_reg_state = gpio_topic_config.num_reg_state.value();
    const size_t total_size = std::accumulate(num_reg_state.begin(), num_reg_state.end(), 0,
                                              [](const size_t cum_sum, const gpio_config::NumRegConfig& io_cmd_i) {
                                                return cum_sum + io_cmd_i.length;
                                              });
    num_reg_state_msg_.values.resize(total_size);
    index_num_reg_state_.resize(total_size);
    size_t offset = 0;
    for (const gpio_config::NumRegConfig& num_reg_state_i : num_reg_state)
    {
      for (uint32_t i = 0; i < num_reg_state_i.length; ++i)
      {
        std::string name = std::string("FloatReg/") + std::to_string(num_reg_state_i.start + i);
        state_interface_configuration_.names.push_back(name);
        index_num_reg_state_[i + offset] = state_interface_index++;
        num_reg_state_msg_.values[i + offset].value = 0.0;
        num_reg_state_msg_.values[i + offset].index = num_reg_state_i.start + i;
      }
      offset += num_reg_state_i.length;
    }
  }

  using fanuc_robot_driver::kConnectionStatusName;
  using fanuc_robot_driver::kIsConnectedType;
  using fanuc_robot_driver::kRobotStatusInterfaceName;
  using fanuc_robot_driver::kStatusCollaborativeSpeedScalingType;
  using fanuc_robot_driver::kStatusContactStopModeType;
  using fanuc_robot_driver::kStatusEStoppedType;
  using fanuc_robot_driver::kStatusInErrorType;
  using fanuc_robot_driver::kStatusMotionPossibleType;
  using fanuc_robot_driver::kStatusTPEnabledType;

  index_connection_status_[0] = state_interface_index++;
  state_interface_configuration_.names.push_back(std::string(kConnectionStatusName) + "/" + kIsConnectedType);

  index_robot_status_[0] = state_interface_index++;
  state_interface_configuration_.names.push_back(std::string(kRobotStatusInterfaceName) + "/" +
                                                 kStatusContactStopModeType);
  index_robot_status_[1] = state_interface_index++;
  state_interface_configuration_.names.push_back(std::string(kRobotStatusInterfaceName) + "/" + kStatusEStoppedType);
  index_robot_status_[2] = state_interface_index++;
  state_interface_configuration_.names.push_back(std::string(kRobotStatusInterfaceName) + "/" + kStatusInErrorType);
  index_robot_status_[3] = state_interface_index++;
  state_interface_configuration_.names.push_back(std::string(kRobotStatusInterfaceName) + "/" +
                                                 kStatusMotionPossibleType);
  index_robot_status_[4] = state_interface_index++;
  state_interface_configuration_.names.push_back(std::string(kRobotStatusInterfaceName) + "/" + kStatusTPEnabledType);

  index_collaborative_speed_scaling_ = state_interface_index++;
  state_interface_configuration_.names.push_back(std::string(kRobotStatusInterfaceName) + "/" +
                                                 kStatusCollaborativeSpeedScalingType);

  rt_analog_io_cmd_buffer_.initRT(fanuc_msgs::msg::AnalogIOCmd());
  rt_io_cmd_buffer_.initRT(fanuc_msgs::msg::IOCmd());
  rt_num_reg_cmd_buffer_.initRT(fanuc_msgs::msg::NumRegCmd());

  analog_io_cmd_subscriber_ = get_node()->create_subscription<fanuc_msgs::msg::AnalogIOCmd>(
      "~/analog_io_cmd", rclcpp::QoS(1).reliable(), [this](const fanuc_msgs::msg::AnalogIOCmd::ConstSharedPtr& msg) {
        WriteMessageCallback(*msg, rt_analog_io_cmd_buffer_);
      });
  io_cmd_subscriber_ = get_node()->create_subscription<fanuc_msgs::msg::IOCmd>(
      "~/io_cmd", rclcpp::QoS(1).reliable(),
      [this](const fanuc_msgs::msg::IOCmd::ConstSharedPtr& msg) { WriteMessageCallback(*msg, rt_io_cmd_buffer_); });
  num_reg_cmd_subscriber_ = get_node()->create_subscription<fanuc_msgs::msg::NumRegCmd>(
      "~/num_reg_cmd", rclcpp::QoS(1).reliable(), [this](const fanuc_msgs::msg::NumRegCmd::ConstSharedPtr& msg) {
        WriteMessageCallback(*msg, rt_num_reg_cmd_buffer_);
      });

  analog_io_state_publisher_ =
      get_node()->create_publisher<fanuc_msgs::msg::AnalogIOState>("~/analog_io_state", rclcpp::QoS(1).reliable());
  connection_status_publisher_ =
      get_node()->create_publisher<fanuc_msgs::msg::ConnectionStatus>("~/connection_status", rclcpp::QoS(1).reliable());
  io_state_publisher_ = get_node()->create_publisher<fanuc_msgs::msg::IOState>("~/io_state", rclcpp::QoS(1).reliable());
  num_reg_state_publisher_ =
      get_node()->create_publisher<fanuc_msgs::msg::NumRegState>("~/num_reg_state", rclcpp::QoS(1).reliable());
  robot_status_publisher_ =
      get_node()->create_publisher<fanuc_msgs::msg::RobotStatus>("~/robot_status", rclcpp::QoS(1).reliable());
  robot_status_ext_publisher_ =
      get_node()->create_publisher<fanuc_msgs::msg::RobotStatusExt>("~/robot_status_ext", rclcpp::QoS(1).reliable());
  collaborative_speed_scaling_publisher_ = get_node()->create_publisher<fanuc_msgs::msg::CollaborativeSpeedScaling>(
      "~/collaborative_speed_scaling", rclcpp::QoS(1).reliable());

  rt_analog_io_state_publisher_ =
      std::make_unique<RealtimePublisher<fanuc_msgs::msg::AnalogIOState>>(analog_io_state_publisher_);
  rt_connection_status_publisher_ =
      std::make_unique<RealtimePublisher<fanuc_msgs::msg::ConnectionStatus>>(connection_status_publisher_);
  rt_io_state_publisher_ = std::make_unique<RealtimePublisher<fanuc_msgs::msg::IOState>>(io_state_publisher_);
  rt_num_reg_state_publisher_ =
      std::make_unique<RealtimePublisher<fanuc_msgs::msg::NumRegState>>(num_reg_state_publisher_);
  rt_robot_status_publisher_ =
      std::make_unique<RealtimePublisher<fanuc_msgs::msg::RobotStatus>>(robot_status_publisher_);
  rt_collaborative_speed_scaling_publisher_ =
      std::make_unique<RealtimePublisher<fanuc_msgs::msg::CollaborativeSpeedScaling>>(
          collaborative_speed_scaling_publisher_);

  get_analog_io_service_ = get_node()->create_service<fanuc_msgs::srv::GetAnalogIO>("~/get_analog_io", &GetAnalogIO);
  get_bool_io_service_ = get_node()->create_service<fanuc_msgs::srv::GetBoolIO>("~/get_bool_io", &GetBoolIO);
  get_num_reg_service_ = get_node()->create_service<fanuc_msgs::srv::GetNumReg>("~/get_num_reg", &GetNumReg);
  get_pos_reg_service_ = get_node()->create_service<fanuc_msgs::srv::GetPosReg>("~/get_pos_reg", &GetPosReg);
  set_analog_io_service_ = get_node()->create_service<fanuc_msgs::srv::SetAnalogIO>("~/set_analog_io", &SetAnalogIO);
  set_bool_io_service_ = get_node()->create_service<fanuc_msgs::srv::SetBoolIO>("~/set_bool_io", &SetBoolIO);
  set_gen_override_service_ =
      get_node()->create_service<fanuc_msgs::srv::SetGenOverride>("~/set_gen_override", &SetGenOverride);
  set_group_io_service_ = get_node()->create_service<fanuc_msgs::srv::SetGroupIO>("~/set_group_io", &SetGroupIO);
  get_group_io_service_ = get_node()->create_service<fanuc_msgs::srv::GetGroupIO>("~/get_group_io", &GetGroupIO);
  set_num_reg_service_ = get_node()->create_service<fanuc_msgs::srv::SetNumReg>("~/set_num_reg", &SetNumReg);
  set_pos_reg_service_ = get_node()->create_service<fanuc_msgs::srv::SetPosReg>("~/set_pos_reg", &SetPosReg);
  set_payload_id_service_ =
      get_node()->create_service<fanuc_msgs::srv::SetPayloadID>("~/set_payload_id", &SetPayloadID);
  set_payload_value_service_ =
      get_node()->create_service<fanuc_msgs::srv::SetPayloadValue>("~/set_payload_value", &SetPayloadValue);
  set_payload_comp_service_ =
      get_node()->create_service<fanuc_msgs::srv::SetPayloadComp>("~/set_payload_comp", &SetPayloadComp);

  reentrant_group_ = get_node()->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  robot_status_ext_timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds(33), [this]() { this->publishRobotStatusExt(); }, reentrant_group_);

  return ControllerInterface::on_configure(previous_state);
}

controller_interface::CallbackReturn FanucGPIOController::on_activate(const rclcpp_lifecycle::State& state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FanucGPIOController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  // Publish all state interface data
  if (rt_analog_io_state_publisher_->trylock())
  {
    for (size_t i = 0; i < index_analog_io_state_.size(); ++i)
    {
      analog_io_state_msg_.values[i].value = state_interfaces_[index_analog_io_state_[i]].get_value();
    }
    rt_analog_io_state_publisher_->msg_ = analog_io_state_msg_;
    rt_analog_io_state_publisher_->unlockAndPublish();
  }

  if (rt_connection_status_publisher_->trylock())
  {
    connection_status_msg_.is_connected = state_interfaces_[index_connection_status_[0]].get_value();
    rt_connection_status_publisher_->msg_ = connection_status_msg_;
    rt_connection_status_publisher_->unlockAndPublish();
  }

  if (rt_io_state_publisher_->trylock())
  {
    for (size_t i = 0; i < index_io_state_.size(); ++i)
    {
      io_state_msg_.values[i].value = state_interfaces_[index_io_state_[i]].get_value();
    }
    rt_io_state_publisher_->msg_ = io_state_msg_;
    rt_io_state_publisher_->unlockAndPublish();
  }

  if (rt_num_reg_state_publisher_->trylock())
  {
    for (size_t i = 0; i < index_num_reg_state_.size(); ++i)
    {
      num_reg_state_msg_.values[i].value = state_interfaces_[index_num_reg_state_[i]].get_value();
    }
    rt_num_reg_state_publisher_->msg_ = num_reg_state_msg_;
    rt_num_reg_state_publisher_->unlockAndPublish();
  }

  if (rt_robot_status_publisher_->trylock())
  {
    robot_status_msg_.contact_stop_mode = state_interfaces_[index_robot_status_[0]].get_value();
    robot_status_msg_.e_stopped = state_interfaces_[index_robot_status_[1]].get_value();
    robot_status_msg_.in_error = state_interfaces_[index_robot_status_[2]].get_value();
    robot_status_msg_.motion_possible = state_interfaces_[index_robot_status_[3]].get_value();
    robot_status_msg_.tp_enabled = state_interfaces_[index_robot_status_[4]].get_value();

    rt_robot_status_publisher_->msg_ = robot_status_msg_;
    rt_robot_status_publisher_->unlockAndPublish();
  }

  if (rt_collaborative_speed_scaling_publisher_->trylock())
  {
    collaborative_speed_scaling_msg_.collaborative_speed_scaling =
        state_interfaces_[index_collaborative_speed_scaling_].get_value();
    rt_collaborative_speed_scaling_publisher_->msg_ = collaborative_speed_scaling_msg_;
    rt_collaborative_speed_scaling_publisher_->unlockAndPublish();
  }

  // Set all command interfaces from the received messages
  const auto analog_io_cmd_msg_new = *rt_analog_io_cmd_buffer_.readFromRT();
  for (const auto& value : analog_io_cmd_msg_new.values)
  {
    if (auto it = analog_io_cmd_msg_indexes_.find(IOTypeToString(value.io_type, value.index));
        it != analog_io_cmd_msg_indexes_.end())
    {
      analog_io_cmd_msg_.values[it->second] = value;
    }
  }

  const auto num_reg_cmd_msg_new = *rt_num_reg_cmd_buffer_.readFromRT();
  for (const auto& value : num_reg_cmd_msg_new.values)
  {
    if (auto it = num_reg_cmd_msg_indexes_.find(value.index); it != num_reg_cmd_msg_indexes_.end())
    {
      num_reg_cmd_msg_.values[it->second] = value;
    }
  }

  const auto io_cmd_msg_new = *rt_io_cmd_buffer_.readFromRT();
  for (const auto& value : io_cmd_msg_new.values)
  {
    if (auto it = io_cmd_msg_indexes_.find(IOTypeToString(value.io_type, value.index)); it != io_cmd_msg_indexes_.end())
    {
      io_cmd_msg_.values[it->second] = value;
    }
  }

  for (size_t i = 0; i < index_analog_io_cmd_.size(); ++i)
  {
    command_interfaces_[index_analog_io_cmd_[i]].set_value(analog_io_cmd_msg_.values[i].value);
  }

  for (size_t i = 0; i < index_io_cmd_.size(); ++i)
  {
    command_interfaces_[index_io_cmd_[i]].set_value(io_cmd_msg_.values[i].value);
  }

  for (size_t i = 0; i < index_num_reg_cmd_.size(); ++i)
  {
    command_interfaces_[index_num_reg_cmd_[i]].set_value(num_reg_cmd_msg_.values[i].value);
  }

  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FanucGPIOController::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  // Set shutdown flag first to prevent timer callback from executing
  shutting_down_.store(true);

  // Reset publisher FIRST so any in-flight callbacks will see null pointer
  robot_status_ext_publisher_.reset();

  // Cancel and reset timer immediately
  if (robot_status_ext_timer_)
  {
    robot_status_ext_timer_->cancel();
  }
  robot_status_ext_timer_.reset();

  // Reset all publishers
  rt_analog_io_state_publisher_.reset();
  rt_connection_status_publisher_.reset();
  rt_io_state_publisher_.reset();
  rt_num_reg_state_publisher_.reset();
  rt_robot_status_publisher_.reset();
  rt_collaborative_speed_scaling_publisher_.reset();

  analog_io_state_publisher_.reset();
  connection_status_publisher_.reset();
  io_state_publisher_.reset();
  num_reg_state_publisher_.reset();
  robot_status_publisher_.reset();
  robot_status_ext_publisher_.reset();

  // Reset all subscribers
  analog_io_cmd_subscriber_.reset();
  io_cmd_subscriber_.reset();
  num_reg_cmd_subscriber_.reset();

  // Reset all services
  get_analog_io_service_.reset();
  get_bool_io_service_.reset();
  get_group_io_service_.reset();
  get_num_reg_service_.reset();
  get_pos_reg_service_.reset();
  set_analog_io_service_.reset();
  set_bool_io_service_.reset();
  set_gen_override_service_.reset();
  set_group_io_service_.reset();
  set_num_reg_service_.reset();
  set_pos_reg_service_.reset();
  set_payload_id_service_.reset();
  set_payload_value_service_.reset();
  set_payload_comp_service_.reset();

  return ControllerInterface::on_deactivate(previous_state);
}
}  // namespace fanuc_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fanuc_controllers::FanucGPIOController, controller_interface::ControllerInterface)
