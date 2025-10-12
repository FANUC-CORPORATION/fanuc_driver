// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "fanuc_client/fanuc_client.hpp"
#include "fanuc_controllers/fanuc_gpio_controller.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;

namespace moveit_pro_controllers
{
TEST(GPIOControllerTest, DISABLED_ROS2ControlLifecycle)
{
  // create command interfaces
  struct InterfaceConfig
  {
    std::string name;
    std::string type;
    double value;
  };
  std::vector<InterfaceConfig> command_interface_configs = { { { "Fanuc_IO", "1", 2 },
                                                               { "Fanuc_IO", "3", 2 },
                                                               { "Fanuc_IO", "5", 2 },
                                                               { "Fanuc_IO", "7", 2 },
                                                               { "Fanuc_IO", "9", 2 },
                                                               { "Fanuc_Connection_Status", "error", 2 },
                                                               { "Fanuc_Connection_Status", "is_connected", 2 },
                                                               { "Fanuc_Connection_Status", "is_real_robot_connected",
                                                                 2 },
                                                               { "Fanuc_Robot_Status_Ext", "control_mode", 2 },
                                                               { "Fanuc_Robot_Status_Ext", "drives_powered", 2 },
                                                               { "Fanuc_Robot_Status_Ext", "error_code", 2 },
                                                               { "Fanuc_Robot_Status_Ext", "gen_override", 2 },
                                                               { "Fanuc_Robot_Status_Ext", "in_motion", 2 },
                                                               { "Fanuc_Robot_Status_Ext", "prog_override", 2 },
                                                               { "Fanuc_Status", "e_stopped", 2 },
                                                               { "Fanuc_Status", "in_error", 2 },
                                                               { "Fanuc_Status", "motion_possible", 2 },
                                                               { "Fanuc_Status", "tp_enabled", 2 },
                                                               { "J1", "position", 2 },
                                                               { "J2", "position", 2 },
                                                               { "J3", "position", 2 } } };
  std::vector<InterfaceConfig> state_interface_configs = command_interface_configs;

  std::vector<hardware_interface::LoanedCommandInterface> loaned_command_interfaces;
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(command_interface_configs.size());
  for (auto& [name, type, value] : command_interface_configs)
  {
    command_interfaces.emplace_back(name, type, &value);
    loaned_command_interfaces.emplace_back(command_interfaces.back());
  }

  std::vector<hardware_interface::LoanedStateInterface> loaned_state_interfaces;
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(state_interface_configs.size());
  for (auto& [name, type, value] : state_interface_configs)
  {
    state_interfaces.emplace_back(name, type, &value);
    loaned_state_interfaces.emplace_back(state_interfaces.back());
  }

  const auto node = std::make_shared<rclcpp::Node>("controller_manager");
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node);
  std::thread ros_thread([&exec]() { exec->spin(); });
  // wait for node to start spinning
  while (!exec->is_spinning()) {}
  rclcpp::sleep_for(std::chrono::seconds(1));

  // set parameters for controller
  fanuc_controllers::FanucGPIOController gpio_controller;

  ASSERT_EQ(gpio_controller.init("gpio_controller", "", 100, "test_controller_manager", rclcpp::NodeOptions()), 
            controller_interface::return_type::OK)
      << "failed to initialize the controller";
  gpio_controller.get_node()->declare_parameter("gpio_config_file", std::string(GPIO_CONFIG_FILE));
  exec->add_node(gpio_controller.get_node()->get_node_base_interface());

  auto command_config = gpio_controller.command_interface_configuration();
  EXPECT_EQ(command_config.names.size(), 3);
  auto state_config = gpio_controller.state_interface_configuration();
  EXPECT_EQ(state_config.names.size(), 19);

  // assign interfaces
  gpio_controller.assign_interfaces(std::move(loaned_command_interfaces), std::move(loaned_state_interfaces));

  fanuc_client::RMISingleton::creatNewRMIInstance("192.168.1.100");
  fanuc_client::RMISingleton::getRMIInstance()->connect(std::nullopt);
  fanuc_client::RMISingleton::getRMIInstance()->reset(std::nullopt);
  fanuc_client::RMISingleton::getRMIInstance()->abort(std::nullopt);
  fanuc_client::RMISingleton::getRMIInstance()->initializeRemoteMotion(std::nullopt);

  // configure the controller
  EXPECT_EQ(gpio_controller.on_configure(gpio_controller.get_lifecycle_state()), controller_interface::CallbackReturn::SUCCESS)
      << "failed to configure controller";

  // activate the controller
  EXPECT_EQ(gpio_controller.on_activate(gpio_controller.get_lifecycle_state()), controller_interface::CallbackReturn::SUCCESS)
      << "failed to activate controller";

  // update the controller
  for (int i = 0; i < 1; ++i)
  {
    // update the controller: expect no memory allocation
    EXPECT_EQ(gpio_controller.update(gpio_controller.get_node()->now(), rclcpp::Duration::from_seconds(0.1)),
              controller_interface::return_type::OK)
        << "failed to update controller";
    rclcpp::sleep_for(std::chrono::milliseconds(8));
  }
  // call service
  const auto client_get_bool_io = node->create_client<fanuc_msgs::srv::GetBoolIO>("/gpio_controller/get_bool_io");
  ASSERT_TRUE(client_get_bool_io->wait_for_service(100s)) << "service missing";
  const auto get_bool_io_request = std::make_shared<fanuc_msgs::srv::GetBoolIO::Request>();
  get_bool_io_request->io_type.type = fanuc_msgs::msg::IOType::DI;
  get_bool_io_request->index = 81;
  auto get_bool_io_response = client_get_bool_io->async_send_request(get_bool_io_request).future.get();

  const auto client_set_bool_io = node->create_client<fanuc_msgs::srv::SetBoolIO>("/gpio_controller/set_bool_io");
  ASSERT_TRUE(client_set_bool_io->wait_for_service(100s)) << "service missing";
  const auto set_bool_io_request = std::make_shared<fanuc_msgs::srv::SetBoolIO::Request>();
  set_bool_io_request->io_type.type = fanuc_msgs::msg::IOType::F;
  set_bool_io_request->index = 81;
  set_bool_io_request->value = true;
  auto set_bool_io_response = client_set_bool_io->async_send_request(set_bool_io_request).future.get();

  const auto client_get_num_reg = node->create_client<fanuc_msgs::srv::GetNumReg>("/gpio_controller/get_num_reg");
  ASSERT_TRUE(client_get_num_reg->wait_for_service(100s)) << "service missing";
  const auto get_num_reg_request = std::make_shared<fanuc_msgs::srv::GetNumReg::Request>();
  get_num_reg_request->index = 5;
  auto client_get_num_reg_response = client_get_num_reg->async_send_request(get_num_reg_request).future.get();

  const auto client_set_num_reg = node->create_client<fanuc_msgs::srv::SetNumReg>("/gpio_controller/set_num_reg");
  ASSERT_TRUE(client_set_num_reg->wait_for_service(100s)) << "service missing";
  const auto set_num_reg_request = std::make_shared<fanuc_msgs::srv::SetNumReg::Request>();
  set_num_reg_request->index = 1;
  set_num_reg_request->value = 5.2;
  auto client_set_num_reg_response = client_set_num_reg->async_send_request(set_num_reg_request).future.get();

  const auto client_set_gen_override =
      node->create_client<fanuc_msgs::srv::SetGenOverride>("/gpio_controller/set_gen_override");
  ASSERT_TRUE(client_set_gen_override->wait_for_service(100s)) << "service missing";
  const auto set_gen_override_request = std::make_shared<fanuc_msgs::srv::SetGenOverride::Request>();
  set_gen_override_request->value = 100;
  auto client_set_gen_override_response =
      client_set_gen_override->async_send_request(set_gen_override_request).future.get();

  const auto client_set_payload_id =
      node->create_client<fanuc_msgs::srv::SetPayloadID>("/gpio_controller/set_payload_id");
  ASSERT_TRUE(client_set_payload_id->wait_for_service(100s)) << "service missing";
  const auto set_payload_id_request = std::make_shared<fanuc_msgs::srv::SetPayloadID::Request>();
  set_payload_id_request->payload_schedule_id = 1;
  auto client_set_payload_id_response = client_set_payload_id->async_send_request(set_payload_id_request).future.get();

  exec->cancel();
  ros_thread.join();

  fanuc_client::RMISingleton::getRMIInstance()->disconnect(std::nullopt);
}
}  // namespace moveit_pro_controllers

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
