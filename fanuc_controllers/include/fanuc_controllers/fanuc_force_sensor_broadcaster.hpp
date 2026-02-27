// SPDX-FileCopyrightText: 2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "fanuc_msgs/msg/force_sensor.hpp"
#include "fanuc_msgs/srv/cfg_force_sensor.hpp"

namespace fanuc_controllers
{
class FanucForceSensorBroadcaster : public controller_interface::ControllerInterface
{
public:
  FanucForceSensorBroadcaster() = default;

  ~FanucForceSensorBroadcaster() override = default;

  FanucForceSensorBroadcaster(const FanucForceSensorBroadcaster&) = delete;

  FanucForceSensorBroadcaster& operator=(const FanucForceSensorBroadcaster&) = delete;

  CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
  template <typename T>
  using ServicePtr = std::shared_ptr<rclcpp::Service<T>>;
  ServicePtr<fanuc_msgs::srv::CfgForceSensor> cfg_force_sensor_service_;

  template <typename T>
  using PublisherPtr = std::shared_ptr<rclcpp::Publisher<T>>;
  PublisherPtr<fanuc_msgs::msg::ForceSensor> force_sensor_publisher_;

  template <typename T>
  using RealtimePublisher = realtime_tools::RealtimePublisher<T>;
  std::unique_ptr<RealtimePublisher<fanuc_msgs::msg::ForceSensor>> rt_force_sensor_publisher_;

  fanuc_msgs::msg::ForceSensor force_sensor_msg_;

  std::array<size_t, 7> index_force_sensor_{};

  controller_interface::InterfaceConfiguration state_interface_configuration_;
  controller_interface::InterfaceConfiguration command_interface_configuration_;
};
}  // namespace fanuc_controllers
