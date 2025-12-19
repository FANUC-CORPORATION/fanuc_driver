// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "fanuc_client/fanuc_client.hpp"

#include <Eigen/Core>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <utility>

#include "fanuc_client/gpio_buffer.hpp"
#include "readerwriterqueue.h"
#include "stream_motion/packets.hpp"

namespace fanuc_client
{
// Static member initialization
FanucClient* FanucClient::instance_ = nullptr;
std::mutex FanucClient::instance_mutex_;
struct sigaction FanucClient::previous_sigaction_;

namespace
{
constexpr double kFullPayload = 7.0;
constexpr auto kStatusPacketFailureMessage = "Invalid robot status packet. Make sure the robot connected can be "
                                             "reached on the network and is in a running state.";
constexpr auto kStatusStatusNotReadyMessage = "Stream motion control is not ready. Check if the robot has alarms "
                                              "or something is disturbing Remote Motion's TP program execution.";

void AssertIsStreaming(const std::atomic<bool>& is_streaming)
{
  if (!is_streaming)
  {
    throw std::runtime_error(
        "Robot is not streaming. Please ensure the real-time stream is running before stream motion command.");
  }
}

void AssertNotStreaming(const std::atomic<bool>& is_streaming)
{
  if (is_streaming)
  {
    throw std::runtime_error(
        "Robot is currently streaming. RMI motion commands cannot be issued when stream motion is active.");
  }
}

constexpr ContactStopMode ToContactStopMode(stream_motion::ContactStopStatus status)
{
  using ::stream_motion::ContactStopStatus;
  switch (status)
  {
    case ContactStopStatus::SAFE:
      return ContactStopMode::SAFE;
    case ContactStopStatus::STOP:
      return ContactStopMode::STOP;
    case ContactStopStatus::DSBL:
      return ContactStopMode::DSBL;
    case ContactStopStatus::ESCP:
      return ContactStopMode::ESCP;
    case ContactStopStatus::None:
      return ContactStopMode::None;
  }
  return ContactStopMode::None;
}

}  // namespace

struct FanucClient::PQueueImpl
{
  using StampedEigen = std::tuple<std::chrono::duration<double>, Eigen::VectorXd>;
  // TODO: Consider merging the command and command_io queues.
  moodycamel::BlockingReaderWriterQueue<StampedEigen> command_queue_;
  moodycamel::BlockingReaderWriterQueue<std::array<uint8_t, 256>> command_io_queue_;
  moodycamel::BlockingReaderWriterQueue<stream_motion::RobotStatusPacket> robot_state_queue_;
};

FanucClient::FanucClient(std::string robot_ip, const uint16_t stream_motion_port, const uint16_t rmi_port,
                         std::unique_ptr<stream_motion::StreamMotionInterface> stream_motion_interface,
                         std::unique_ptr<rmi::RMIConnectionInterface> rmi_connection_interface)
  : robot_ip_{ std::move(robot_ip) }
  , stream_motion_port_{ stream_motion_port }
  , rmi_port_{ rmi_port }
  , stream_motion_{ stream_motion_interface == nullptr ?
                        std::make_unique<stream_motion::StreamMotionConnection>(robot_ip_, 1.0, stream_motion_port_) :
                        std::move(stream_motion_interface) }
  , command_pos{}
  , rmi_connection_{ rmi_connection_interface == nullptr ?
                         RMISingleton::creatNewRMIInstance(robot_ip_, rmi_port_) :
                         RMISingleton::setRMIInstance(std::move(rmi_connection_interface)) }
  , out_cmd_interp_buff_target_{ 8 }
  , p_queue_impl_(std::make_unique<PQueueImpl>())
{
  rmi_connection_->connect(5);
  stream_motion_->sendStopPacket();
  stream_motion::ControllerCapabilityResultPacket controller_capability;
  stream_motion_->getControllerCapability(controller_capability);
  control_period_ = controller_capability.sampling_rate;
  fetchRobotLimits();

  setupSignalHandler();
}

FanucClient::~FanucClient()
{
  if (is_streaming_)
  {
    try
    {
      std::cout << "Stopping realtime stream during destruction" << std::endl;
      stopRealtimeStream();
    }
    catch (const std::exception& e)
    {
      // During destruction, network might already be down or robot disconnected
      // Log the error but don't throw to avoid std::terminate
      std::cerr << "Warning: Failed to stop realtime stream during FanucClient destruction: " << e.what() << std::endl;
    }
  }
  else
  {
    try
    {
      std::cout << "Aborting RMI connection during destruction" << std::endl;
      rmi_connection_->abort(std::nullopt);
    }
    catch (const std::exception& e)
    {
      std::cerr << "Warning: Failed to abort RMI connection during destruction (connection may be lost): " << e.what()
                << std::endl;
    }
    catch (...)
    {
      std::cerr << "Warning: Unknown exception during RMI abort (connection may be lost)" << std::endl;
    }

    try
    {
      std::cout << "Sending stop packet during destruction" << std::endl;
      stream_motion_->sendStopPacket();
    }
    catch (const std::exception& e)
    {
      std::cerr << "Warning: Failed to send stop packet during destruction: " << e.what() << std::endl;
    }
    catch (...)
    {
      std::cerr << "Warning: Unknown exception during stop packet send" << std::endl;
    }
  }

  try
  {
    rmi_connection_->disconnect(std::nullopt);
  }
  catch (const std::exception& e)
  {
    std::cerr << "Warning: Failed to disconnect RMI during destruction (connection may be lost): " << e.what()
              << std::endl;
  }
  catch (...)
  {
    std::cerr << "Warning: Unknown exception during RMI disconnect" << std::endl;
  }

  restoreSignalHandler();
}

void FanucClient::readStateFromQueue()
{
  stream_motion::RobotStatusPacket robot_status;
  bool updated = false;
  while (p_queue_impl_->robot_state_queue_.try_dequeue(robot_status))
  {
    updated = true;
  }
  if (!updated)
  {
    return;
  }

  for (Eigen::Index i = 0; i < robot_status.joint_angle.size(); ++i)
  {
    last_joint_angles_[i] = static_cast<double>(robot_status.joint_angle[i]);
  }

  if (gpio_buffer_ != nullptr)
  {
    gpio_buffer_->status_buffer() = robot_status.io_status;
  }

  robot_status_.in_error = robot_status.robot_status & 0x1;
  robot_status_.tp_enabled = robot_status.robot_status & 0x2;
  robot_status_.e_stopped = robot_status.robot_status & 0x4;
  robot_status_.motion_possible = robot_status.status & 0x1;
  robot_status_.contact_stop_mode = ToContactStopMode(robot_status.contact_stop_status);
  robot_status_.safety_scale = robot_status.safety_scale;
}

void FanucClient::writeJointTarget(const Eigen::VectorXd& joint_targets)
{
  AssertIsStreaming(is_streaming_);
  readStateFromQueue();
  last_joint_angles_cmd_ = joint_targets;

  if (joint_targets.size() != last_joint_angles_.size())
  {
    throw std::invalid_argument("Joint targets size does not match the size of last joint angles.");
  }
  auto cur_time_from_start = std::chrono::high_resolution_clock::now() - start_time_;
  p_queue_impl_->command_queue_.enqueue({ cur_time_from_start, last_joint_angles_cmd_ });
}

void FanucClient::writeJointTargetRMI(const Eigen::VectorXd& joint_targets)
{
  AssertNotStreaming(is_streaming_);
  last_joint_angles_cmd_ = joint_targets;
  last_joint_angles_cmd_[2] = last_joint_angles_cmd_[2] - last_joint_angles_cmd_[1];

  rmi::JointMotionJRepPacket::Request request_joint_motion;
  request_joint_motion.JointAngle.J1 = static_cast<float>(last_joint_angles_cmd_[0]);
  request_joint_motion.JointAngle.J2 = static_cast<float>(last_joint_angles_cmd_[1]);
  request_joint_motion.JointAngle.J3 = static_cast<float>(last_joint_angles_cmd_[2]);
  request_joint_motion.JointAngle.J4 = static_cast<float>(last_joint_angles_cmd_[3]);
  request_joint_motion.JointAngle.J5 = static_cast<float>(last_joint_angles_cmd_[4]);
  request_joint_motion.JointAngle.J6 = static_cast<float>(last_joint_angles_cmd_[5]);
  request_joint_motion.JointAngle.J7 = static_cast<float>(last_joint_angles_cmd_[6]);
  request_joint_motion.JointAngle.J8 = static_cast<float>(last_joint_angles_cmd_[7]);
  request_joint_motion.JointAngle.J9 = static_cast<float>(last_joint_angles_cmd_[8]);
  request_joint_motion.SpeedType = "Percent";
  request_joint_motion.Speed = 100;
  request_joint_motion.TermType = "FINE";
  rmi_connection_->sendJointMotion(request_joint_motion, 5.0);
}

Eigen::Ref<const Eigen::VectorXd> FanucClient::readJointAngles()
{
  AssertIsStreaming(is_streaming_);
  readStateFromQueue();

  return last_joint_angles_;
}

Eigen::Ref<const Eigen::VectorXd> FanucClient::readJointAnglesRMI()
{
  AssertNotStreaming(is_streaming_);

  const auto response = rmi_connection_->readJointAngles(std::nullopt, std::nullopt);
  last_joint_angles_[0] = response.JointAngle.J1;
  last_joint_angles_[1] = response.JointAngle.J2;
  last_joint_angles_[2] = response.JointAngle.J3;
  last_joint_angles_[3] = response.JointAngle.J4;
  last_joint_angles_[4] = response.JointAngle.J5;
  last_joint_angles_[5] = response.JointAngle.J6;
  last_joint_angles_[6] = response.JointAngle.J7;
  last_joint_angles_[7] = response.JointAngle.J8;
  last_joint_angles_[8] = response.JointAngle.J9;

  last_joint_angles_[2] = last_joint_angles_[2] + last_joint_angles_[1];

  return last_joint_angles_;
}

bool FanucClient::sendIOCommand() const
{
  if (gpio_buffer_ == nullptr)
  {
    // Nothing to command.
    return true;
  }

  return p_queue_impl_->command_io_queue_.try_enqueue(gpio_buffer_->command_buffer());
}

void FanucClient::streamMotionThread(const Eigen::VectorXd& joint_angles)
{
  stream_motion::RobotStatusPacket status;
  double command_timestamp = 0.0;
  double last_command_timestamp = 0.0;
  Eigen::VectorXd command = joint_angles;
  Eigen::VectorXd last_command = joint_angles;
  std::array<uint8_t, 256> command_io{};
  double ts_drift = 0.0;
  double dev_time = 0.0;
  double dev_time_prev = 0.0;

  while (is_streaming_)
  {
    if (!stream_motion_->getStatusPacket(status))
    {
      // Abort stream if we cannot get the status packet
      is_streaming_ = false;
    }

    // set estimated time to first command's timestamp
    if ((dev_time == 0.0) && (p_queue_impl_->command_queue_.size_approx() != 0))
    {
      const PQueueImpl::StampedEigen* queue_entry = p_queue_impl_->command_queue_.peek();
      dev_time = std::get<0>(*queue_entry).count();
      dev_time_prev = dev_time;
    }
    else
    {
      // calculate drift
      double time_error = static_cast<double>(p_queue_impl_->command_queue_.size_approx()) -
                          static_cast<double>(out_cmd_interp_buff_target_);
      ts_drift = 0.99 * ts_drift + time_error * 0.000001;

      // push the time forward
      dev_time_prev = dev_time;
      dev_time += (getControlPeriod() / 1000.0) + ts_drift;
    }

    int size_before = p_queue_impl_->command_queue_.size_approx();

    // find the right interval to use.
    while (p_queue_impl_->command_queue_.size_approx() != 0)
    {
      const PQueueImpl::StampedEigen* queue_entry = p_queue_impl_->command_queue_.peek();
      last_command = command;
      last_command_timestamp = command_timestamp;
      command_timestamp = std::get<0>(*queue_entry).count();
      command = std::get<1>(*queue_entry);
      if (dev_time == 0.0)
      {
        dev_time = command_timestamp;
        dev_time_prev = command_timestamp;
      }
      if (command_timestamp >= dev_time_prev)
      {
        break;
      }
      p_queue_impl_->command_queue_.pop();
    }

    // Do interpolation.
    double alpha;
    if (command_timestamp - last_command_timestamp < 1e-6)
    {
      alpha = 0;
    }
    else
    {
      alpha = (dev_time_prev - last_command_timestamp) / (command_timestamp - last_command_timestamp);
    }
    alpha = std::min(alpha, 1.0);
    alpha = std::max(alpha, 0.0);
    for (Eigen::Index i = 0; i < status.joint_angle.size(); ++i)
    {
      command_pos[i] = alpha * command[i] + (1.0 - alpha) * last_command[i];
    }

    // Handle IO commands.
    while (p_queue_impl_->command_io_queue_.try_dequeue(command_io)) {}

    stream_motion_->sendCommand(command_pos, !is_streaming_, command_io);
    p_queue_impl_->robot_state_queue_.enqueue(status);
  }
}

void FanucClient::fetchRobotLimits()
{
  AssertNotStreaming(is_streaming_);

  for (int j = 0; j < stream_motion::kMaxAxisNumber; ++j)
  {
    stream_motion::RobotThresholdPacket robot_threshold_velocity;
    stream_motion::RobotThresholdPacket robot_threshold_acceleration;
    stream_motion::RobotThresholdPacket robot_threshold_jerk;
    if (!stream_motion_->getRobotLimits(j + 1, robot_threshold_velocity, robot_threshold_acceleration,
                                        robot_threshold_jerk))
    {
      throw std::runtime_error("Failed to get robot limits for axis " + std::to_string(j + 1) +
                               ". Ensure that the robot is reachable on the network by its IP.");
    }
    for (int index = 0; index < 20; ++index)
    {
      vel_limits_no_load_(j, index) = robot_threshold_velocity.no_payload[index];
      vel_limits_full_load_(j, index) = robot_threshold_velocity.full_payload[index];
      acc_limits_no_load_(j, index) = robot_threshold_acceleration.no_payload[index];
      acc_limits_full_load_(j, index) = robot_threshold_acceleration.full_payload[index];
      jerk_limits_no_load_(j, index) = robot_threshold_jerk.no_payload[index];
      jerk_limits_full_load_(j, index) = robot_threshold_jerk.full_payload[index];
    }
  }
}

bool FanucClient::getLimits(const double v_peak, const double payload, std::vector<double>& vel_limit,
                            std::vector<double>& acc_limit, std::vector<double>& jerk_limit) const
{
  const double v_max = 2000;
  const double v_min = v_max / 20;
  const double num = v_peak * 1.2 - v_min;
  const double denom = 1 / (v_max - v_min);
  const double pct = (std::min)((std::max)(num * denom, 0.0) * 19.0, 19.0);
  const int idx_l = (std::max)(static_cast<int>(pct), 0);
  const int idx_u = (std::min)(static_cast<int>(ceil(pct)), 19);
  const double idx_frac = pct - static_cast<double>(pct);
  Eigen::VectorXd vel_limit_no_load =
      idx_frac * (vel_limits_no_load_.col(idx_u) - vel_limits_no_load_.col(idx_l)) + vel_limits_no_load_.col(idx_l);
  Eigen::VectorXd acc_limit_no_load =
      idx_frac * (acc_limits_no_load_.col(idx_u) - acc_limits_no_load_.col(idx_l)) + acc_limits_no_load_.col(idx_l);
  Eigen::VectorXd jerk_limit_no_load =
      idx_frac * (jerk_limits_no_load_.col(idx_u) - jerk_limits_no_load_.col(idx_l)) + jerk_limits_no_load_.col(idx_l);
  Eigen::VectorXd vel_limit_full_load =
      idx_frac * (vel_limits_full_load_.col(idx_u) - vel_limits_full_load_.col(idx_l)) +
      vel_limits_full_load_.col(idx_l);
  Eigen::VectorXd acc_limit_full_load =
      idx_frac * (acc_limits_full_load_.col(idx_u) - acc_limits_full_load_.col(idx_l)) +
      acc_limits_full_load_.col(idx_l);
  Eigen::VectorXd jerk_limit_full_load =
      idx_frac * (jerk_limits_full_load_.col(idx_u) - jerk_limits_full_load_.col(idx_l)) +
      jerk_limits_full_load_.col(idx_l);
  const double payload_pct = payload / kFullPayload;
  vel_limit.resize(vel_limit_no_load.size(), 0.0);
  acc_limit.resize(acc_limit_full_load.size(), 0.0);
  jerk_limit.resize(jerk_limit_full_load.size(), 0.0);
  for (Eigen::Index i = 0; i < vel_limit_no_load.size(); ++i)
  {
    vel_limit[i] = payload_pct * (vel_limit_full_load[i] - vel_limit_no_load[i]) + vel_limit_no_load[i];
    acc_limit[i] = payload_pct * (acc_limit_full_load[i] - acc_limit_no_load[i]) + acc_limit_no_load[i];
    jerk_limit[i] = payload_pct * (jerk_limit_full_load[i] - jerk_limit_no_load[i]) + jerk_limit_no_load[i];
  }
  return true;
}

void FanucClient::startRMI()
{
  if (rmi_running_)
  {
    return;
  }
  try
  {
    rmi_connection_->getStatus(std::nullopt);
    rmi_connection_->reset(std::nullopt);
    rmi_connection_->initializeRemoteMotion(std::nullopt);
  }
  catch (const std::runtime_error&)
  {
    std::cout << "Need to reset and abort" << std::endl;
    rmi_connection_->abort(std::nullopt);
    rmi_connection_->reset(std::nullopt);
    rmi_connection_->getStatus(std::nullopt);
    rmi_connection_->initializeRemoteMotion(std::nullopt);
  }
  rmi_running_ = true;
}

// Throws if it fails to start real-time communication
void FanucClient::startRealtimeStream(std::shared_ptr<GPIOBuffer> gpio_buffer)
{
  AssertNotStreaming(is_streaming_);

  gpio_buffer_ = std::move(gpio_buffer);
  if (gpio_buffer_ != nullptr)
  {
    stream_motion_->configureGPIO(gpio_buffer_->toStreamMotionConfig());
  }

  startRMI();
  rmi_connection_->programCallNonBlocking("STREAM_MOTN");

  // Wait for the stream connection to be ready
  stream_motion::RobotStatusPacket status;
  stream_motion_->sendStartPacket();
  const auto pre_loop_time = std::chrono::steady_clock::now();
  bool got_status = false;
  while (true)
  {
    if (stream_motion_->getStatusPacket(status))
    {
      got_status = true;
      if (status.status & 0x1)
      {
        /* STREAM_MOTN.TP is ready */
        break;
      }
    }
    if (std::chrono::steady_clock::now() - pre_loop_time > std::chrono::seconds(2))
    {
      if (got_status)
      {
        throw std::runtime_error(kStatusStatusNotReadyMessage);
      }
      else
      {
        throw std::runtime_error(kStatusPacketFailureMessage);
      }
    }
  }
  start_time_ = std::chrono::high_resolution_clock::now();
  p_queue_impl_->robot_state_queue_.enqueue(status);
  is_streaming_ = true;
  last_joint_angles_ = Eigen::VectorXd::Zero(status.joint_angle.size());
  for (Eigen::Index i = 0; i < status.joint_angle.size(); ++i)
  {
    last_joint_angles_[i] = static_cast<double>(status.joint_angle[i]);
    command_pos[i] = static_cast<double>(status.joint_angle[i]);
  }
  stream_motion_->sendCommand(command_pos, false, {});

  if (rt_thread_.joinable())
  {
    rt_thread_.join();
  }
  rt_thread_ = std::thread([this] { streamMotionThread(last_joint_angles_); });
}

void FanucClient::stopRealtimeStream()
{
  is_streaming_ = false;
  if (rt_thread_.joinable())
  {
    rt_thread_.join();
  }

  // Wait for robot to stop motion
  stream_motion::RobotStatusPacket status;
  const auto motion_pre_loop_time = std::chrono::steady_clock::now();
  do
  {
    if (std::chrono::steady_clock::now() - motion_pre_loop_time > std::chrono::seconds(1))
    {
      break;
    }

    const auto status_pre_loop_time = std::chrono::steady_clock::now();
    while (!stream_motion_->getStatusPacket(status))
    {
      if (std::chrono::steady_clock::now() - status_pre_loop_time > std::chrono::seconds(1))
      {
        throw std::runtime_error(kStatusPacketFailureMessage);
      }
    }
  } while (status.status & 0x8);

  rmi_connection_->abort(std::nullopt);
  stream_motion_->sendStopPacket();
}

bool FanucClient::isStreaming()
{
  return is_streaming_;
}

uint32_t FanucClient::getControlPeriod() const
{
  return control_period_;
}

void FanucClient::setPayloadSchedule(const uint8_t payload_schedule) const
{
  rmi_connection_->setPayloadSchedule(payload_schedule, std::nullopt);
}

void FanucClient::validateGPIOBuffer(const std::shared_ptr<GPIOBuffer>& gpio_buffer) const
{
  if (gpio_buffer != nullptr)
  {
    if (!stream_motion_->configureGPIO(gpio_buffer->toStreamMotionConfig()))
    {
      throw std::runtime_error(
          "Failed to configure GPIO buffer. Ensure the GPIO buffer is correctly set up for the robot.");
    }
  }
}

void FanucClient::stopStreaming()
{
  is_streaming_ = false;
}

void FanucClient::signalHandler(int signal)
{
  if (signal == SIGINT)
  {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (instance_ != nullptr)
    {
      // Stop streaming to allow proper cleanup
      instance_->stopStreaming();
    }
  }
}

void FanucClient::setupSignalHandler()
{
  std::lock_guard<std::mutex> lock(instance_mutex_);
  instance_ = this;
  struct sigaction sa;
  sa.sa_handler = signalHandler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  // Save previous handler and install new one
  sigaction(SIGINT, &sa, &previous_sigaction_);
}

void FanucClient::restoreSignalHandler()
{
  std::lock_guard<std::mutex> lock(instance_mutex_);
  if (instance_ == this)
  {
    instance_ = nullptr;
    // Restore previous signal handler using sigaction (consistent with setup)
    sigaction(SIGINT, &previous_sigaction_, nullptr);
  }
}

}  // namespace fanuc_client
