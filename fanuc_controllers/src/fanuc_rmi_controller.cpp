// SPDX-FileCopyrightText: 2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "fanuc_controllers/fanuc_rmi_controller.hpp"
#include "fanuc_client/fanuc_client.hpp"
#include "fanuc_robot_driver/constants.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include <concepts>

namespace fanuc_controllers
{
constexpr auto kFRRMIController = "FR_RMI_Controller";

namespace
{
std::shared_ptr<rmi::RMIConnectionInterface> getRMIInstance()
{
  return fanuc_client::RMISingleton::getRMIInstance();
}

void CheckRMIBuffer()
{
  const auto remaining_size = getRMIInstance()->getRemainingBuffuerSize();
  if (remaining_size <= 0)
  {
    throw std::runtime_error("Remaining RMI buffer size is " + std::to_string(remaining_size));
  }
}

template <typename T>
void ApplyMotionInstructionOptions(const std::shared_ptr<rmi_msgs::srv::AddMotionInstruction::Request>& request,
                                   typename T::Request& packet)
{
  if (request->options.use_acc)
  {
    if constexpr (requires(typename T::Request t) { t.ACC; })
    {
      packet.ACC = request->options.acc;
    }
    else
    {
      throw std::runtime_error("The instruction does not have acc option");
    }
  }
  if (request->options.use_offset_pr_number)
  {
    if constexpr (requires(typename T::Request t) { t.OffsetPRNumber; })
    {
      packet.OffsetPRNumber = request->options.offset_pr_number;
    }
    else
    {
      throw std::runtime_error("The instruction does not have offset_pr_number option");
    }
  }

  if (request->options.use_vision_pr_number)
  {
    if constexpr (requires(typename T::Request t) { t.VisionPRNumber; })
    {
      packet.VisionPRNumber = request->options.vision_pr_number;
    }
    else
    {
      throw std::runtime_error("The instruction does not have vision_pr_number option");
    }
  }
  if (request->options.use_wrist_joint)
  {
    if constexpr (requires(typename T::Request t) { t.WristJoint; })
    {
      packet.WristJoint = request->options.wrist_joint;
    }
    else
    {
      throw std::runtime_error("The instruction does not have wrist_joint option");
    }
  }
  if (request->options.use_mrot)
  {
    if constexpr (requires(typename T::Request t) { t.MROT; })
    {
      packet.MROT = request->options.mrot;
    }
    else
    {
      throw std::runtime_error("The instruction does not have mrot option");
    }
  }
  if (request->options.use_lcb_type)
  {
    if constexpr (requires(typename T::Request t) { t.LCBType; })
    {
      packet.LCBType = request->options.lcb_type;
    }
    else
    {
      throw std::runtime_error("The instruction does not have lcb_type option");
    }
  }
  if (request->options.use_lcb_value)
  {
    if constexpr (requires(typename T::Request t) { t.LCBValue; })
    {
      packet.LCBValue = request->options.lcb_value;
    }
    else
    {
      throw std::runtime_error("The instruction does not have lcb_value option");
    }
  }
  if (request->options.use_port_type)
  {
    if constexpr (requires(typename T::Request t) { t.PortType; })
    {
      packet.PortType = request->options.port_type;
    }
    else
    {
      throw std::runtime_error("The instruction does not have port_type option");
    }
  }
  if (request->options.use_port_number)
  {
    if constexpr (requires(typename T::Request t) { t.portNumber; })
    {
      packet.portNumber = request->options.port_number;
    }
    else
    {
      throw std::runtime_error("The instruction does not have port_number option");
    }
  }
  if (request->options.use_port_value)
  {
    if constexpr (requires(typename T::Request t) { t.portValue; })
    {
      packet.portValue = request->options.port_value;
    }
    else
    {
      throw std::runtime_error("The instruction does not have port_value option");
    }
  }
  if (request->options.use_tool_offset_pr_number)
  {
    if constexpr (requires(typename T::Request t) { t.ToolOffsetPRNumber; })
    {
      packet.ToolOffsetPRNumber = request->options.tool_offset_pr_number;
    }
    else
    {
      throw std::runtime_error("The instruction does not have tool_offset_pr_number option");
    }
  }
  if (request->options.use_alim)
  {
    if constexpr (requires(typename T::Request t) { t.ALIM; })
    {
      packet.ALIM = request->options.alim;
    }
    else
    {
      throw std::runtime_error("The instruction does not have alim option");
    }
  }
  if (request->options.use_alim_reg)
  {
    if constexpr (requires(typename T::Request t) { t.ALIMREG; })
    {
      packet.ALIMREG = request->options.alim_reg;
    }
    else
    {
      throw std::runtime_error("The instruction does not have alim_reg option");
    }
  }
  if (request->options.use_no_blend)
  {
    if constexpr (requires(typename T::Request t) { t.NoBlend; })
    {
      packet.NoBlend = request->options.no_blend;
    }
    else
    {
      throw std::runtime_error("The instruction does not have no_blend option");
    }
  }
}

template <typename T>
typename T::Request
MotionInstructionRequestToPacket(const std::shared_ptr<rmi_msgs::srv::AddMotionInstruction::Request>& request)
{
  typename T::Request packet;

  if (request->representation.representation == rmi_msgs::msg::Representation::JOINT)
  {
    if constexpr (requires(typename T::Request t) { t.JointAngle; })
    {
      packet.JointAngle.J1 = request->joint.positions[0];
      packet.JointAngle.J2 = request->joint.positions[1];
      packet.JointAngle.J3 = request->joint.positions[2];
      packet.JointAngle.J4 = request->joint.positions[3];
      packet.JointAngle.J5 = request->joint.positions[4];
      packet.JointAngle.J6 = request->joint.positions[5];
      packet.JointAngle.J7 = request->joint.positions[6];
      packet.JointAngle.J8 = request->joint.positions[7];
      packet.JointAngle.J9 = request->joint.positions[8];
    }
    else
    {
      throw std::runtime_error("Representation is JOINT but the packet type does not have joint angles.");
    }
  }
  else if (request->representation.representation == rmi_msgs::msg::Representation::CART)
  {
    if constexpr (requires(typename T::Request t) { t.Configuration; })
    {
      packet.Configuration.UToolNumber = request->cartesian.utool;
      packet.Configuration.UFrameNumber = request->cartesian.uframe;
      packet.Configuration.Front = request->cartesian.front;
      packet.Configuration.Up = request->cartesian.up;
      packet.Configuration.Left = request->cartesian.left;
      packet.Configuration.Flip = request->cartesian.flip;
      packet.Configuration.Turn4 = request->cartesian.turn4;
      packet.Configuration.Turn5 = request->cartesian.turn5;
      packet.Configuration.Turn6 = request->cartesian.turn6;
    }
    else
    {
      throw std::runtime_error("Representation is CARTESIAN but the packet type does not have configuration.");
    }
    if constexpr (requires(typename T::Request t) { t.Position; })
    {
      packet.Position.X = request->cartesian.x;
      packet.Position.Y = request->cartesian.y;
      packet.Position.Z = request->cartesian.z;
      packet.Position.W = request->cartesian.w;
      packet.Position.P = request->cartesian.p;
      packet.Position.R = request->cartesian.r;
      packet.Position.Ext1 = request->cartesian.ext1;
      packet.Position.Ext2 = request->cartesian.ext2;
      packet.Position.Ext3 = request->cartesian.ext3;
    }
    else
    {
      throw std::runtime_error("Representation is CARTESIAN but the packet type does not have position.");
    }
  }
  else
  {
    throw std::runtime_error("The representation is not supported.: " + request->representation.representation);
  }

  if constexpr (requires(typename T::Request t) { t.ViaConfiguration; })
  {
    packet.ViaConfiguration.UToolNumber = request->via_cartesian.utool;
    packet.ViaConfiguration.UFrameNumber = request->via_cartesian.uframe;
    packet.ViaConfiguration.Front = request->via_cartesian.front;
    packet.ViaConfiguration.Up = request->via_cartesian.up;
    packet.ViaConfiguration.Left = request->via_cartesian.left;
    packet.ViaConfiguration.Flip = request->via_cartesian.flip;
    packet.ViaConfiguration.Turn4 = request->via_cartesian.turn4;
    packet.ViaConfiguration.Turn5 = request->via_cartesian.turn5;
    packet.ViaConfiguration.Turn6 = request->via_cartesian.turn6;
  }
  if constexpr (requires(typename T::Request t) { t.ViaPosition; })
  {
    packet.ViaPosition.X = request->via_cartesian.x;
    packet.ViaPosition.Y = request->via_cartesian.y;
    packet.ViaPosition.Z = request->via_cartesian.z;
    packet.ViaPosition.W = request->via_cartesian.w;
    packet.ViaPosition.P = request->via_cartesian.p;
    packet.ViaPosition.R = request->via_cartesian.r;
    packet.ViaPosition.Ext1 = request->via_cartesian.ext1;
    packet.ViaPosition.Ext2 = request->via_cartesian.ext2;
    packet.ViaPosition.Ext3 = request->via_cartesian.ext3;
  }

  packet.SpeedType = request->speed_type.type;
  packet.Speed = request->speed_value;
  packet.TermType = request->term_type.type;
  packet.TermValue = request->term_value;

  try
  {
    ApplyMotionInstructionOptions<T>(request, packet);
  }
  catch (std::runtime_error& e)
  {
    throw std::runtime_error(std::string("Motion option: ") + std::string(e.what()));
  }
  return packet;
}

template <typename T>
void SendMotionInstruction(const std::shared_ptr<rmi_msgs::srv::AddMotionInstruction::Request>& request,
                           const std::shared_ptr<rmi_msgs::srv::AddMotionInstruction::Response>& response)
{
  try
  {
    CheckRMIBuffer();
    typename T::Request packet = MotionInstructionRequestToPacket<T>(request);
    getRMIInstance()->sendRMIPacketNonBlocking(packet);
    response->sequence_id = packet.SequenceID;
  }
  catch (std::runtime_error& e)
  {
    throw std::runtime_error(std::string("SendMotionInstruction: ") + std::string(e.what()));
  }
}

void ProcessJointMotion(const std::shared_ptr<rmi_msgs::srv::AddMotionInstruction::Request>& request,
                        const std::shared_ptr<rmi_msgs::srv::AddMotionInstruction::Response>& response)
{
  try
  {
    if (request->incremental)
    {
      if (request->representation.representation == rmi_msgs::msg::Representation::CARTESIAN)
      {
        SendMotionInstruction<rmi::JointRelativePacket>(request, response);
      }
      else
      {
        SendMotionInstruction<rmi::JointRelativeJRepPacket>(request, response);
      }
    }
    else
    {
      if (request->representation.representation == rmi_msgs::msg::Representation::CARTESIAN)
      {
        SendMotionInstruction<rmi::JointMotionPacket>(request, response);
      }
      else
      {
        SendMotionInstruction<rmi::JointMotionJRepPacket>(request, response);
      }
    }
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRRMIController), "ProcessJointMotion Failed: %s", e.what());
    response->result = 1;
  }
}

void ProcessLinearMotion(const std::shared_ptr<rmi_msgs::srv::AddMotionInstruction::Request>& request,
                         const std::shared_ptr<rmi_msgs::srv::AddMotionInstruction::Response>& response)
{
  try
  {
    if (request->incremental)
    {
      if (request->representation.representation == rmi_msgs::msg::Representation::CARTESIAN)
      {
        SendMotionInstruction<rmi::LinearRelativePacket>(request, response);
      }
      else
      {
        SendMotionInstruction<rmi::LinearRelativeJRepPacket>(request, response);
      }
    }
    else
    {
      if (request->representation.representation == rmi_msgs::msg::Representation::CARTESIAN)
      {
        SendMotionInstruction<rmi::LinearMotionPacket>(request, response);
      }
      else
      {
        SendMotionInstruction<rmi::LinearMotionJRepPacket>(request, response);
      }
    }
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRRMIController), "ProcessLinearMotion Failed: %s", e.what());
    response->result = 1;
  }
}

void ProcessCircularMotion(const std::shared_ptr<rmi_msgs::srv::AddMotionInstruction::Request>& request,
                           const std::shared_ptr<rmi_msgs::srv::AddMotionInstruction::Response>& response)
{
  try
  {
    if (request->representation.representation == rmi_msgs::msg::Representation::J)
    {
      throw std::runtime_error(std::string("Circular motion does not support Joint representation."));
    }
    else
    {
      if (request->incremental)
      {
        SendMotionInstruction<rmi::CircularRelativePacket>(request, response);
      }
      else
      {
        SendMotionInstruction<rmi::CircularMotionPacket>(request, response);
      }
    }
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRRMIController), "ProcessCircularMotion Failed: %s", e.what());
    response->result = 1;
  }
}

void ProcessSplineMotion(const std::shared_ptr<rmi_msgs::srv::AddMotionInstruction::Request>& request,
                         const std::shared_ptr<rmi_msgs::srv::AddMotionInstruction::Response>& response)
{
  try
  {
    if (request->incremental)
    {
      throw std::runtime_error(std::string("Spline motion does not support Incremental motion."));
    }
    else
    {
      if (request->representation.representation == rmi_msgs::msg::Representation::CARTESIAN)
      {
        SendMotionInstruction<rmi::SplineMotionPacket>(request, response);
      }
      else
      {
        SendMotionInstruction<rmi::SplineMotionJRepPacket>(request, response);
      }
    }
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRRMIController), "ProcessSplineMotion Failed: %s", e.what());
    response->result = 1;
  }
}

std::vector<rmi::RMICallParam> getCallParamsVector(const std::shared_ptr<rmi_msgs::srv::AddCall::Request>& request)
{
  std::vector<rmi::RMICallParam> params = {};
  const auto params_size = request->params.size();

  if (params_size > 10)
  {
    throw std::runtime_error("Too many params for FRC_Call: " + std::to_string(params_size));
    return params;
  }

  if (params_size == 0)
  {
    return params;
  }

  for (const auto& req_param : request->params)
  {
    rmi::RMICallParam param;
    param.first = req_param.type;
    if (param.first == rmi_msgs::msg::CallParam::INT)
    {
      param.first = std::string("Constant");
      param.second = req_param.int_value;
    }
    else if (param.first == rmi_msgs::msg::CallParam::FLOAT)
    {
      param.first = std::string("Constant");
      param.second = req_param.float_value;
    }
    else if (param.first == rmi_msgs::msg::CallParam::STRING)
    {
      param.second = req_param.string_value;
    }
    else
    {
      param.second = req_param.int_value;
    }
    params.push_back(param);
  }

  return params;
}

}  // namespace

void FanucRMIController::CallCommand(const std::shared_ptr<rmi_msgs::srv::CallCommand::Request>& request,
                                     const std::shared_ptr<rmi_msgs::srv::CallCommand::Response>& response)
{
  response->result = 0;
  if (!ControllerIsAvailable())
  {
    response->result = 1;
    return;
  }

  if (request->command.type == rmi_msgs::msg::Command::INIT)
  {
    try
    {
      std::optional<uint8_t> groupmask = std::nullopt;
      if (request->options.use_group_mask)
      {
        groupmask = request->options.group_mask;
      }
      std::optional<std::string> rtsa = std::nullopt;
      if (request->options.use_rtsa)
      {
        rtsa = request->options.rtsa;
      }
      std::optional<std::string> pltzmode = std::nullopt;
      if (request->options.use_pltzmode)
      {
        pltzmode = request->options.pltzmode;
      }
      getRMIInstance()->initializeRemoteMotion(std::nullopt, groupmask, rtsa, pltzmode);
    }
    catch (std::runtime_error& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(kFRRMIController), "FRC_Initialize failed: %s", e.what());
      response->result = 1;
    }
  }
  else if (request->command.type == rmi_msgs::msg::Command::ABORT)
  {
    try
    {
      getRMIInstance()->abort(std::nullopt);
    }
    catch (std::runtime_error& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(kFRRMIController), "FRC_Abort failed: %s", e.what());
      response->result = 1;
    }
  }
  else if (request->command.type == rmi_msgs::msg::Command::PAUSE)
  {
    try
    {
      getRMIInstance()->pause(std::nullopt);
    }
    catch (std::runtime_error& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(kFRRMIController), "FRC_Pause failed: %s", e.what());
      response->result = 1;
    }
  }
  else if (request->command.type == rmi_msgs::msg::Command::RESUME)
  {
    try
    {
      getRMIInstance()->resume(std::nullopt);
    }
    catch (std::runtime_error& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(kFRRMIController), "FRC_Resume failed: %s", e.what());
      response->result = 1;
    }
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRRMIController), "Not supported command: %s", request->command.type.c_str());
    response->result = 1;
  }
}

void FanucRMIController::AddMotionInstruction(
    const std::shared_ptr<rmi_msgs::srv::AddMotionInstruction::Request>& request,
    const std::shared_ptr<rmi_msgs::srv::AddMotionInstruction::Response>& response)
{
  response->result = 0;
  if (!ControllerIsAvailable())
  {
    response->result = 1;
    return;
  }

  if (request->motion_type.type == rmi_msgs::msg::MotionType::J)
  {
    ProcessJointMotion(request, response);
  }
  else if (request->motion_type.type == rmi_msgs::msg::MotionType::L)
  {
    ProcessLinearMotion(request, response);
  }
  else if (request->motion_type.type == rmi_msgs::msg::MotionType::C)
  {
    ProcessCircularMotion(request, response);
  }
  else if (request->motion_type.type == rmi_msgs::msg::MotionType::S)
  {
    ProcessSplineMotion(request, response);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRRMIController), "Not supported motion instruction: %s",
                 request->motion_type.type.c_str());
    response->result = 1;
  }
}

void FanucRMIController::AddCall(const std::shared_ptr<rmi_msgs::srv::AddCall::Request>& request,
                                 const std::shared_ptr<rmi_msgs::srv::AddCall::Response>& response)
{
  response->result = 0;
  if (!ControllerIsAvailable())
  {
    response->result = 1;
    return;
  }

  try
  {
    const auto params = getCallParamsVector(request);
    const auto packet = getRMIInstance()->programCallNonBlocking(request->program_name, params);
    response->sequence_id = packet.SequenceID;
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRRMIController), "FRC_CALL failed: %s", e.what());
    response->result = 1;
  }
}

template <typename T1, typename T2>
void FanucRMIController::AddLogicInstruction(const std::shared_ptr<typename T1::Request>& request,
                                             const std::shared_ptr<typename T1::Response>& response)
{
  if (!ControllerIsAvailable())
  {
    response->result = 1;
    return;
  }
  typename T2::Request packet;
  response->result = 0;
  if constexpr (requires(typename T2::Request t) { t.PortNumber; })
  {
    packet.PortNumber = request->port_number;
  }
  if constexpr (requires(typename T2::Request t) { t.PortValue; })
  {
    packet.PortValue = request->port_value;
  }
  if constexpr (requires(typename T2::Request t) { t.FrameNumber; })
  {
    packet.FrameNumber = request->frame_number;
  }
  if constexpr (requires(typename T2::Request t) { t.ToolNumber; })
  {
    packet.ToolNumber = request->tool_number;
  }
  if constexpr (requires(typename T2::Request t) { t.Time; })
  {
    packet.Time = request->time;
  }
  if constexpr (requires(typename T2::Request t) { t.ScheduleNumber; })
  {
    packet.ScheduleNumber = request->schedule_number;
  }

  try
  {
    CheckRMIBuffer();
    getRMIInstance()->sendRMIPacketNonBlocking(packet);
    response->sequence_id = packet.SequenceID;
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRRMIController), "Sending Logic Instruction failed: %s", e.what());
    response->result = 1;
  }
}

bool FanucRMIController::ControllerIsAvailable()
{
  const bool active = (this->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  if (!active)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kFRRMIController), "Controller state is not active.");
  }
  return active;
}

controller_interface::CallbackReturn FanucRMIController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration FanucRMIController::command_interface_configuration() const
{
  return command_interface_configuration_;
}

controller_interface::InterfaceConfiguration FanucRMIController::state_interface_configuration() const
{
  return state_interface_configuration_;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FanucRMIController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  // Setup command interfaces
  command_interface_configuration_.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interface_configuration_.names.clear();

  using fanuc_robot_driver::kRMICommandName;
  using fanuc_robot_driver::kRMIInterfaceName;

  command_interface_configuration_.names.push_back(std::string(kRMIInterfaceName) + "/" + std::string(kRMICommandName));

  // Setup state interfaces
  state_interface_configuration_.type = controller_interface::interface_configuration_type::NONE;
  state_interface_configuration_.names.clear();

  using namespace std::placeholders;
  rmi_command_service_ = get_node()->create_service<rmi_msgs::srv::CallCommand>(
      "~/call_command", std::bind(&FanucRMIController::CallCommand, this, _1, _2));

  rmi_motion_service_ = get_node()->create_service<rmi_msgs::srv::AddMotionInstruction>(
      "~/add_motion_instruction", std::bind(&FanucRMIController::AddMotionInstruction, this, _1, _2));

  rmi_call_service_ = get_node()->create_service<rmi_msgs::srv::AddCall>(
      "~/add_call_instruction", std::bind(&FanucRMIController::AddCall, this, _1, _2));

  rmi_wait_din_service_ = get_node()->create_service<rmi_msgs::srv::AddWaitDIN>(
      "~/add_wait_din_instruction",
      std::bind(&FanucRMIController::AddLogicInstruction<rmi_msgs::srv::AddWaitDIN, rmi::WaitForDINPacket>, this, _1,
                _2));
  rmi_set_uframe_service_ = get_node()->create_service<rmi_msgs::srv::AddSetUFrame>(
      "~/add_set_uframe_instruction",
      std::bind(&FanucRMIController::AddLogicInstruction<rmi_msgs::srv::AddSetUFrame, rmi::SetUFramePacket>, this, _1,
                _2));
  rmi_set_utool_service_ = get_node()->create_service<rmi_msgs::srv::AddSetUTool>(
      "~/add_set_utool_instruction",
      std::bind(&FanucRMIController::AddLogicInstruction<rmi_msgs::srv::AddSetUTool, rmi::SetToolFramePacket>, this, _1,
                _2));
  rmi_wait_time_service_ = get_node()->create_service<rmi_msgs::srv::AddWaitTime>(
      "~/add_wait_time_instruction",
      std::bind(&FanucRMIController::AddLogicInstruction<rmi_msgs::srv::AddWaitTime, rmi::WaitForTimePacket>, this, _1,
                _2));
  rmi_set_payload_service_ = get_node()->create_service<rmi_msgs::srv::AddSetPayload>(
      "~/add_set_payload_instruction",
      std::bind(&FanucRMIController::AddLogicInstruction<rmi_msgs::srv::AddSetPayload, rmi::SetPayloadInstructionPacket>,
                this, _1, _2));

  return ControllerInterface::on_configure(previous_state);
}

controller_interface::CallbackReturn FanucRMIController::on_activate(const rclcpp_lifecycle::State& state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FanucRMIController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  // do nothing

  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FanucRMIController::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return ControllerInterface::on_deactivate(previous_state);
}
}  // namespace fanuc_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fanuc_controllers::FanucRMIController, controller_interface::ControllerInterface)
