// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "fanuc_client/fanuc_client.hpp"
#include "fanuc_client/gpio_buffer.hpp"
#include "gpio_config/gpio_config.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace fanuc_robot_driver
{

struct IOCommandInterface;
struct IOStateInterface;

class FanucHardwareInterface : public hardware_interface::SystemInterface
{
public:
  FanucHardwareInterface();

  FanucHardwareInterface(const FanucHardwareInterface&) = delete;

  FanucHardwareInterface& operator=(const FanucHardwareInterface&) = delete;

  ~FanucHardwareInterface() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  struct RobotStatusValues
  {
    // Unless otherwise stated, all values are bools.
    double in_error{};
    double tp_enabled{};
    double e_stopped{};
    double motion_possible{};
    // Enum type:
    // None = 0
    // SAFE = 1
    // STOP = 2
    // DSBL = 3
    // ESCP = 4
    double contact_stop_mode{};
    // unsigned int
    double collaborative_speed_scaling{};

    double is_connected{};
  };

  std::unique_ptr<fanuc_client::FanucClient> fanuc_client_;
  Eigen::VectorXd fr_joint_pos_;
  Eigen::VectorXd fr_prev_joint_pos_;
  Eigen::VectorXd fr_joint_vel_;
  Eigen::VectorXd joint_targets_;
  Eigen::VectorXd joint_targets_degrees_;
  RobotStatusValues robot_status_;
  std::string ip_address_;
  int32_t payload_schedule_;
  uint16_t stream_motion_port_;
  uint16_t rmi_port_;
  uint32_t out_cmd_interp_buff_target_;

  std::shared_ptr<fanuc_client::GPIOBuffer> gpio_buffer_;

  std::vector<std::unique_ptr<IOCommandInterface>> io_commands_;
  std::vector<std::unique_ptr<IOStateInterface>> io_state_;
};

}  // namespace fanuc_robot_driver
