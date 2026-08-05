// SPDX-FileCopyrightText: 2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "rmi_msgs/srv/add_call.hpp"
#include "rmi_msgs/srv/add_motion_instruction.hpp"
#include "rmi_msgs/srv/add_set_payload.hpp"
#include "rmi_msgs/srv/add_set_u_frame.hpp"
#include "rmi_msgs/srv/add_set_u_tool.hpp"
#include "rmi_msgs/srv/add_wait_din.hpp"
#include "rmi_msgs/srv/add_wait_time.hpp"
#include "rmi_msgs/srv/call_command.hpp"

namespace fanuc_controllers
{
class FanucRMIController : public controller_interface::ControllerInterface
{
public:
  FanucRMIController() = default;

  ~FanucRMIController() override = default;

  FanucRMIController(const FanucRMIController&) = delete;

  FanucRMIController& operator=(const FanucRMIController&) = delete;

  CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
  void CallCommand(const std::shared_ptr<rmi_msgs::srv::CallCommand::Request>& request,
                   const std::shared_ptr<rmi_msgs::srv::CallCommand::Response>& response);
  void AddMotionInstruction(const std::shared_ptr<rmi_msgs::srv::AddMotionInstruction::Request>& request,
                            const std::shared_ptr<rmi_msgs::srv::AddMotionInstruction::Response>& response);
  void AddCall(const std::shared_ptr<rmi_msgs::srv::AddCall::Request>& request,
               const std::shared_ptr<rmi_msgs::srv::AddCall::Response>& response);
  template <typename T1, typename T2>
  void AddLogicInstruction(const std::shared_ptr<typename T1::Request>& request,
                           const std::shared_ptr<typename T1::Response>& response);

  bool ControllerIsAvailable();

  template <typename T>
  using ServicePtr = std::shared_ptr<rclcpp::Service<T>>;
  ServicePtr<rmi_msgs::srv::CallCommand> rmi_command_service_;
  ServicePtr<rmi_msgs::srv::AddMotionInstruction> rmi_motion_service_;
  ServicePtr<rmi_msgs::srv::AddCall> rmi_call_service_;
  ServicePtr<rmi_msgs::srv::AddWaitDIN> rmi_wait_din_service_;
  ServicePtr<rmi_msgs::srv::AddSetUFrame> rmi_set_uframe_service_;
  ServicePtr<rmi_msgs::srv::AddSetUTool> rmi_set_utool_service_;
  ServicePtr<rmi_msgs::srv::AddWaitTime> rmi_wait_time_service_;
  ServicePtr<rmi_msgs::srv::AddSetPayload> rmi_set_payload_service_;

  controller_interface::InterfaceConfiguration state_interface_configuration_;
  controller_interface::InterfaceConfiguration command_interface_configuration_;
};
}  // namespace fanuc_controllers
