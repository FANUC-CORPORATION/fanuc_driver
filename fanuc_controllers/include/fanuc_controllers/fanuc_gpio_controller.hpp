// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <filesystem>
#include <std_msgs/msg/detail/bool__builder.hpp>
#include <std_msgs/msg/detail/float64__struct.hpp>

#include "controller_interface/controller_interface.hpp"
#include "controller_manager_msgs/srv/list_hardware_interfaces.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "fanuc_msgs/srv/get_analog_io.hpp"
#include "fanuc_msgs/srv/get_bool_io.hpp"
#include "fanuc_msgs/srv/get_group_io.hpp"
#include "fanuc_msgs/srv/get_num_reg.hpp"
#include "fanuc_msgs/srv/get_pos_reg.hpp"
#include "fanuc_msgs/srv/set_analog_io.hpp"
#include "fanuc_msgs/srv/set_bool_io.hpp"
#include "fanuc_msgs/srv/set_gen_override.hpp"
#include "fanuc_msgs/srv/set_group_io.hpp"
#include "fanuc_msgs/srv/set_num_reg.hpp"
#include "fanuc_msgs/srv/set_payload_comp.hpp"
#include "fanuc_msgs/srv/set_payload_id.hpp"
#include "fanuc_msgs/srv/set_payload_value.hpp"
#include "fanuc_msgs/srv/set_pos_reg.hpp"

#include "fanuc_msgs/msg/analog_io_cmd.hpp"
#include "fanuc_msgs/msg/analog_io_state.hpp"
#include "fanuc_msgs/msg/collaborative_speed_scaling.hpp"
#include "fanuc_msgs/msg/connection_status.hpp"
#include "fanuc_msgs/msg/io_cmd.hpp"
#include "fanuc_msgs/msg/io_state.hpp"
#include "fanuc_msgs/msg/num_reg_cmd.hpp"
#include "fanuc_msgs/msg/num_reg_state.hpp"
#include "fanuc_msgs/msg/robot_status.hpp"
#include "fanuc_msgs/msg/robot_status_ext.hpp"

namespace fanuc_controllers
{
class FanucGPIOController : public controller_interface::ControllerInterface
{
public:
  FanucGPIOController() = default;

  ~FanucGPIOController() override = default;

  FanucGPIOController(const FanucGPIOController&) = delete;

  FanucGPIOController& operator=(const FanucGPIOController&) = delete;

  CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
  void publishRobotStatusExt();

  std::shared_ptr<rclcpp::TimerBase> robot_status_ext_timer_;
  rclcpp::CallbackGroup::SharedPtr reentrant_group_;

  template <typename T>
  using ServicePtr = std::shared_ptr<rclcpp::Service<T>>;
  ServicePtr<fanuc_msgs::srv::GetAnalogIO> get_analog_io_service_;
  ServicePtr<fanuc_msgs::srv::GetBoolIO> get_bool_io_service_;
  ServicePtr<fanuc_msgs::srv::GetGroupIO> get_group_io_service_;
  ServicePtr<fanuc_msgs::srv::GetNumReg> get_num_reg_service_;
  ServicePtr<fanuc_msgs::srv::GetPosReg> get_pos_reg_service_;
  ServicePtr<fanuc_msgs::srv::SetAnalogIO> set_analog_io_service_;
  ServicePtr<fanuc_msgs::srv::SetBoolIO> set_bool_io_service_;
  ServicePtr<fanuc_msgs::srv::SetGenOverride> set_gen_override_service_;
  ServicePtr<fanuc_msgs::srv::SetGroupIO> set_group_io_service_;
  ServicePtr<fanuc_msgs::srv::SetNumReg> set_num_reg_service_;
  ServicePtr<fanuc_msgs::srv::SetPosReg> set_pos_reg_service_;
  ServicePtr<fanuc_msgs::srv::SetPayloadID> set_payload_id_service_;
  ServicePtr<fanuc_msgs::srv::SetPayloadValue> set_payload_value_service_;
  ServicePtr<fanuc_msgs::srv::SetPayloadComp> set_payload_comp_service_;

  template <typename T>
  using SubscriberPtr = std::shared_ptr<rclcpp::Subscription<T>>;
  SubscriberPtr<fanuc_msgs::msg::AnalogIOCmd> analog_io_cmd_subscriber_;
  SubscriberPtr<fanuc_msgs::msg::IOCmd> io_cmd_subscriber_;
  SubscriberPtr<fanuc_msgs::msg::NumRegCmd> num_reg_cmd_subscriber_;

  template <typename T>
  using PublisherPtr = std::shared_ptr<rclcpp::Publisher<T>>;
  PublisherPtr<fanuc_msgs::msg::AnalogIOState> analog_io_state_publisher_;
  PublisherPtr<fanuc_msgs::msg::ConnectionStatus> connection_status_publisher_;
  PublisherPtr<fanuc_msgs::msg::IOState> io_state_publisher_;
  PublisherPtr<fanuc_msgs::msg::NumRegState> num_reg_state_publisher_;
  PublisherPtr<fanuc_msgs::msg::RobotStatus> robot_status_publisher_;
  PublisherPtr<fanuc_msgs::msg::RobotStatusExt> robot_status_ext_publisher_;
  PublisherPtr<fanuc_msgs::msg::CollaborativeSpeedScaling> collaborative_speed_scaling_publisher_;

  template <typename T>
  using RealtimePublisher = realtime_tools::RealtimePublisher<T>;
  std::unique_ptr<RealtimePublisher<fanuc_msgs::msg::AnalogIOState>> rt_analog_io_state_publisher_;
  std::unique_ptr<RealtimePublisher<fanuc_msgs::msg::ConnectionStatus>> rt_connection_status_publisher_;
  std::unique_ptr<RealtimePublisher<fanuc_msgs::msg::IOState>> rt_io_state_publisher_;
  std::unique_ptr<RealtimePublisher<fanuc_msgs::msg::NumRegState>> rt_num_reg_state_publisher_;
  std::unique_ptr<RealtimePublisher<fanuc_msgs::msg::RobotStatus>> rt_robot_status_publisher_;
  std::unique_ptr<RealtimePublisher<fanuc_msgs::msg::CollaborativeSpeedScaling>>
      rt_collaborative_speed_scaling_publisher_;

  fanuc_msgs::msg::AnalogIOState analog_io_state_msg_;
  fanuc_msgs::msg::ConnectionStatus connection_status_msg_;
  fanuc_msgs::msg::IOState io_state_msg_;
  fanuc_msgs::msg::NumRegState num_reg_state_msg_;
  fanuc_msgs::msg::RobotStatus robot_status_msg_;
  fanuc_msgs::msg::RobotStatusExt robot_status_ext_msg_;
  fanuc_msgs::msg::CollaborativeSpeedScaling collaborative_speed_scaling_msg_;

  fanuc_msgs::msg::AnalogIOCmd analog_io_cmd_msg_;
  fanuc_msgs::msg::IOCmd io_cmd_msg_;
  fanuc_msgs::msg::NumRegCmd num_reg_cmd_msg_;

  std::unordered_map<std::string, size_t> analog_io_cmd_msg_indexes_;
  std::unordered_map<std::string, size_t> io_cmd_msg_indexes_;
  std::unordered_map<size_t, size_t> num_reg_cmd_msg_indexes_;

  std::vector<size_t> index_analog_io_state_;
  std::array<size_t, 1> index_connection_status_{};
  std::vector<size_t> index_io_state_;
  std::vector<size_t> index_num_reg_state_;
  std::array<size_t, 6> index_robot_status_ext_{};
  std::array<size_t, 5> index_robot_status_{};
  std::vector<size_t> index_analog_io_cmd_;
  std::vector<size_t> index_io_cmd_;
  std::vector<size_t> index_num_reg_cmd_;
  size_t index_collaborative_speed_scaling_{};

  realtime_tools::RealtimeBuffer<fanuc_msgs::msg::AnalogIOCmd> rt_analog_io_cmd_buffer_;
  realtime_tools::RealtimeBuffer<fanuc_msgs::msg::IOCmd> rt_io_cmd_buffer_;
  realtime_tools::RealtimeBuffer<fanuc_msgs::msg::NumRegCmd> rt_num_reg_cmd_buffer_;

  controller_interface::InterfaceConfiguration state_interface_configuration_;
  controller_interface::InterfaceConfiguration command_interface_configuration_;
};
}  // namespace fanuc_controllers
