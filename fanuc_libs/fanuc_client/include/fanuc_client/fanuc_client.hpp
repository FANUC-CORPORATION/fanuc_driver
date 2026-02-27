// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <csignal>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>

#include "fanuc_client/gpio_buffer.hpp"
#include "rmi/rmi.hpp"
#include "stream_motion/stream.hpp"

namespace fanuc_client
{

enum class ContactStopMode
{
  None = 0,
  SAFE = 1,
  STOP = 2,
  DSBL = 3,
  ESCP = 4,
};

struct RobotStatus
{
  bool in_error;
  bool tp_enabled;
  bool e_stopped;
  bool motion_possible;
  ContactStopMode contact_stop_mode;
  float safety_scale;
};

struct ForceSensor
{
  float force_x;
  float force_y;
  float force_z;
  float moment_x;
  float moment_y;
  float moment_z;
  uint32_t fs_type;
};

class FanucClient
{
public:
  FanucClient() = delete;
  explicit FanucClient(std::string robot_ip, uint16_t stream_motion_port = 60015, uint16_t rmi_port = 16001,
                       std::unique_ptr<stream_motion::StreamMotionInterface> stream_motion_interface = nullptr,
                       std::unique_ptr<rmi::RMIConnectionInterface> rmi_connection_interface = nullptr);

  FanucClient(const FanucClient&) = delete;
  FanucClient& operator=(const FanucClient&) = delete;

  ~FanucClient();

  void writeJointTarget(const Eigen::VectorXd& joint_targets);

  void writeJointTargetRMI(const Eigen::VectorXd& joint_targets);

  Eigen::Ref<const Eigen::VectorXd> readJointAngles();

  Eigen::Ref<const Eigen::VectorXd> readJointAnglesRMI();

  bool sendIOCommand() const;

  // Throws if it fails to start real-time communication
  void startRealtimeStream(std::shared_ptr<GPIOBuffer> gpio_buffer = nullptr);

  void stopRealtimeStream();

  void stopStreaming();

  bool isStreaming();

  void startRMI();

  bool getLimits(double v_peak, double payload, std::vector<double>& vel_limit, std::vector<double>& acc_limit,
                 std::vector<double>& jerk_limit) const;

  uint32_t getControlPeriod() const;

  void setPayloadSchedule(uint8_t payload_schedule) const;

  void validateGPIOBuffer(const std::shared_ptr<GPIOBuffer>& gpio_buffer) const;

  void setOutCmdInterpBuffTarget(uint32_t out_cmd_interp_buff_target)
  {
    out_cmd_interp_buff_target_ = out_cmd_interp_buff_target;
  }

  uint32_t getOutCmdInterpBuffTarget() const
  {
    return out_cmd_interp_buff_target_;
  }

  void setForceSensorType(uint32_t force_sensor_type)
  {
    force_sensor_type_ = force_sensor_type;
  }

  uint32_t getForceSensorType() const
  {
    return force_sensor_type_;
  }

  const RobotStatus& robot_status() const
  {
    return robot_status_;
  }

  const ForceSensor& force_sensor() const
  {
    return force_sensor_;
  }

  void configureForceSensor(uint32_t do_reset, uint32_t force_sensor_type) const;

  // Get static instance
  static FanucClient* get_instance()
  {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    return instance_;
  }

  // Get client (stream motion) version
  uint32_t getClientVersion() const
  {
    return client_version_;
  }

private:
  /** Setup signal handler for SIGINT */
  void setupSignalHandler();

  /** Restore previous signal handler */
  void restoreSignalHandler();

  /** Static signal handler function */
  static void signalHandler(int signal);

  /** Static pointer to current instance for signal handler */
  static FanucClient* instance_;

  /** Mutex to protect instance_ access from signal handler */
  static std::mutex instance_mutex_;

  /** Previous signal handler to restore */
  static struct sigaction previous_sigaction_;

private:
  void readStateFromQueue();

  void streamMotionThread(const Eigen::VectorXd& joint_angles);

  /** Grab the limits from the robot.*/
  void fetchRobotLimits();

  const std::string robot_ip_;
  const uint16_t stream_motion_port_;
  const uint16_t rmi_port_;

  // Limits
  Eigen::MatrixXd vel_limits_no_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd acc_limits_no_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd jerk_limits_no_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd vel_limits_full_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd acc_limits_full_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd jerk_limits_full_load_ = Eigen::MatrixXd::Zero(9, 20);

  // Manages stream motion connection
  std::atomic<bool> is_streaming_ = false;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
  std::unique_ptr<stream_motion::StreamMotionInterface> stream_motion_;

  std::array<double, stream_motion::kMaxAxisNumber> command_pos;
  Eigen::VectorXd last_joint_angles_ = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd last_joint_angles_cmd_ = Eigen::VectorXd::Zero(9);
  RobotStatus robot_status_;
  ForceSensor force_sensor_;
  uint32_t control_period_ = 0;
  uint32_t client_version_ = 0;  // stream motion client version

  // IO data only accessed from the non-realitime thread.
  std::shared_ptr<GPIOBuffer> gpio_buffer_;

  // Real time thread data
  std::thread rt_thread_;

  // Manages RMI connection
  std::shared_ptr<rmi::RMIConnectionInterface> rmi_connection_;
  std::atomic<bool> rmi_running_ = false;

  // Output command interpolation buffer target size for stream motion control
  uint32_t out_cmd_interp_buff_target_;

  // Force sensor default type
  uint32_t force_sensor_type_;

  struct PQueueImpl;
  std::unique_ptr<PQueueImpl> p_queue_impl_;
};

class RMISingleton
{
public:
  static std::shared_ptr<rmi::RMIConnectionInterface> creatNewRMIInstance(const std::string& robot_ip_address,
                                                                          uint16_t rmi_port = 16001);

  static std::shared_ptr<rmi::RMIConnectionInterface> getRMIInstance();

  static std::shared_ptr<rmi::RMIConnectionInterface>
  setRMIInstance(std::unique_ptr<rmi::RMIConnectionInterface> rmi_connection);

private:
  RMISingleton();

  ~RMISingleton();

  RMISingleton(const RMISingleton&) = delete;
  RMISingleton& operator=(const RMISingleton&) = delete;

  static RMISingleton& getInstance();
  std::mutex mtx_;

  std::shared_ptr<rmi::RMIConnectionInterface> rmi_connection_interface_;
};

}  // namespace fanuc_client
