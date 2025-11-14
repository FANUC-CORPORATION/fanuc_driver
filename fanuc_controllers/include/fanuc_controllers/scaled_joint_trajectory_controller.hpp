// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#ifndef FANUC_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_
#define FANUC_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <string>

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/int32.hpp"

namespace fanuc_controllers
{

class ScaledJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController
{
public:
  ScaledJointTrajectoryController() = default;
  ~ScaledJointTrajectoryController() override = default;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;

  void time_scale_callback(const std::shared_ptr<std_msgs::msg::Int32> msg);
  double first_order_lag_filter(const double filter_input);

private:
  bool last_is_connected_ = false;
  std::atomic<int> time_scale_value_{ 100 };  // Default to 100% (no scaling)
  rclcpp::Time last_scaled_time_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr time_scale_subscriber_ = nullptr;

  std::string time_scale_topic_name_ = "speed_scaling_factor";
  double folag_state_ = 100.0;
  double folag_h_ = 1.0;
  double folag_tau_ = 0.2;  // time constant for first order lag in seconds

  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> scaling_state_interface_;
  std::atomic<double> scaling_factor_{ 1.0 };
};

}  // namespace fanuc_controllers

#endif  // FANUC_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_
