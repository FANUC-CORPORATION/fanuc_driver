// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "fanuc_controllers/scaled_joint_trajectory_controller.hpp"

#include "angles/angles.h"
#include "fanuc_robot_driver/constants.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace fanuc_controllers
{

controller_interface::CallbackReturn ScaledJointTrajectoryController::on_init()
{
  folag_state_ = time_scale_value_.load();
  return JointTrajectoryController::on_init();
}

controller_interface::InterfaceConfiguration ScaledJointTrajectoryController::state_interface_configuration() const
{
  auto state_interface_config = JointTrajectoryController::state_interface_configuration();
  using fanuc_robot_driver::kConnectionStatusName;
  using fanuc_robot_driver::kIsConnectedType;
  state_interface_config.names.push_back(std::string(kConnectionStatusName) + "/" + kIsConnectedType);
  return state_interface_config;
}

controller_interface::CallbackReturn ScaledJointTrajectoryController::on_activate(const rclcpp_lifecycle::State& state)
{
  // Initialize slider subscriber for time scaling control
  time_scale_subscriber_ = get_node()->create_subscription<std_msgs::msg::Int32>(
      time_scale_topic_name_, 10,
      std::bind(&ScaledJointTrajectoryController::time_scale_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "Time scale subscriber created for topic: %s", time_scale_topic_name_.c_str());

  return JointTrajectoryController::on_activate(state);
}

controller_interface::return_type ScaledJointTrajectoryController::update(const rclcpp::Time& time,
                                                                          const rclcpp::Duration& period)
{
  if (!state_interfaces_.empty() && state_interfaces_.back().get_optional() == 0.0)
  {
    last_is_connected_ = false;
    // The robot state indicated that the robot is no longer connected.
    return controller_interface::return_type::OK;
  }
  if (!last_is_connected_)
  {
    // The controller should update its reference to use the current state
    read_state_from_state_interfaces(state_current_);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    rt_has_pending_goal_ = false;
    new_trajectory_msg_.reset();
    new_trajectory_msg_.initRT(set_hold_position());
    // command_joint_names_
  }
  last_is_connected_ = true;

  folag_h_ = period.seconds();
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    return controller_interface::return_type::OK;
  }

  auto logger = this->get_node()->get_logger();

  // update dynamic parameters
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    default_tolerances_ = get_segment_tolerances(logger, params_);
    // Note: update_pids() is private in parent class, so we can't call it here
    // The parent class will handle PID updates internally
  }

  auto compute_error_for_joint = [&](JointTrajectoryPoint& error, size_t index, const JointTrajectoryPoint& current,
                                     const JointTrajectoryPoint& desired) {
    // error defined as the difference between current and desired
    if (joints_angle_wraparound_[index])
    {
      // if desired, the shortest_angular_distance is calculated, i.e., the error is
      //  normalized between -pi<error<pi
      error.positions[index] = angles::shortest_angular_distance(current.positions[index], desired.positions[index]);
    }
    else
    {
      error.positions[index] = desired.positions[index] - current.positions[index];
    }
    if (has_velocity_state_interface_ && (has_velocity_command_interface_ || has_effort_command_interface_))
    {
      error.velocities[index] = desired.velocities[index] - current.velocities[index];
    }
    if (has_acceleration_state_interface_ && has_acceleration_command_interface_)
    {
      error.accelerations[index] = desired.accelerations[index] - current.accelerations[index];
    }
  };

  // don't update goal after we sampled the trajectory to avoid any racecondition
  const auto active_goal = *rt_active_goal_.readFromRT();

  // Check if a new external message has been received from nonRT threads
  auto current_external_msg = current_trajectory_->get_trajectory_msg();
  auto new_external_msg = new_trajectory_msg_.readFromRT();
  // Discard, if a goal is pending but still not active (somewhere stuck in goal_handle_timer_)
  if (current_external_msg != *new_external_msg && (rt_has_pending_goal_ && !active_goal) == false)
  {
    fill_partial_goal(*new_external_msg);
    sort_to_local_joint_order(*new_external_msg);
    // TODO(denis): Add here integration of position and velocity
    current_trajectory_->update(*new_external_msg);
  }

  // TODO(anyone): can I here also use const on joint_interface since the reference_wrapper is not
  // changed, but its value only?
  auto assign_interface_from_point = [&](auto& joint_interface, const std::vector<double>& trajectory_point_interface) {
    for (size_t index = 0; index < dof_; ++index)
    {
      joint_interface[index].get().set_value(trajectory_point_interface[index]);
    }
  };

  // current state update
  state_current_.time_from_start.sec = 0;
  state_current_.time_from_start.nanosec = 0;
  read_state_from_state_interfaces(state_current_);

  // currently carrying out a trajectory
  if (has_active_trajectory())
  {
    bool first_sample = false;
    // if sampling the first time, set the point before you sample
    if (!current_trajectory_->is_sampled_already())
    {
      first_sample = true;
      last_scaled_time_ = time;
      if (params_.open_loop_control)
      {
        current_trajectory_->set_point_before_trajectory_msg(time, last_commanded_state_, joints_angle_wraparound_);
      }
      else
      {
        current_trajectory_->set_point_before_trajectory_msg(time, state_current_, joints_angle_wraparound_);
      }
    }
    double scale_percentage = 0.01 * first_order_lag_filter(static_cast<double>(time_scale_value_.load()));
    rclcpp::Duration deltatime_scaled = period * scale_percentage;
    rclcpp::Time sample_time = last_scaled_time_ + deltatime_scaled;

    // find segment for current timestamp
    joint_trajectory_controller::TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    const bool valid_point = current_trajectory_->sample(sample_time, interpolation_method_, state_desired_,
                                                              start_segment_itr, end_segment_itr);
    last_scaled_time_ = sample_time;

    if (valid_point)
    {
      const rclcpp::Time traj_start = current_trajectory_->time_from_start();
      // this is the time instance
      // - started with the first segment: when the first point will be reached (in the future)
      // - later: when the point of the current segment was reached
      const rclcpp::Time segment_time_from_start = traj_start + start_segment_itr->time_from_start;
      // time_difference is
      // - negative until first point is reached
      // - counting from zero to time_from_start of next point
      double time_difference = time.seconds() - segment_time_from_start.seconds();
      bool tolerance_violated_while_moving = false;
      bool outside_goal_tolerance = false;
      bool within_goal_time = true;
      const bool before_last_point = end_segment_itr != current_trajectory_->end();
      auto active_tol = active_tolerances_.readFromRT();

      // have we reached the end, are not holding position, and is a timeout configured?
      // Check independently of other tolerances
      if (!before_last_point && !rt_is_holding_ && cmd_timeout_ > 0.0 && time_difference > cmd_timeout_)
      {
        RCLCPP_WARN(logger, "Aborted due to command timeout");

        new_trajectory_msg_.reset();
        new_trajectory_msg_.initRT(set_hold_position());
      }

      // Check state/goal tolerance
      for (size_t index = 0; index < dof_; ++index)
      {
        compute_error_for_joint(state_error_, index, state_current_, state_desired_);

        // Always check the state tolerance on the first sample in case the first sample
        // is the last point
        // print output per default, goal will be aborted afterwards
        if ((before_last_point || first_sample) && !rt_is_holding_ &&
            !check_state_tolerance_per_joint(state_error_, index, active_tol->state_tolerance[index],
                                             true /* show_errors */))
        {
          tolerance_violated_while_moving = true;
        }
        // past the final point, check that we end up inside goal tolerance
        if (!before_last_point && !rt_is_holding_ &&
            !check_state_tolerance_per_joint(state_error_, index, active_tol->goal_state_tolerance[index],
                                             false /* show_errors */))
        {
          outside_goal_tolerance = true;

          if (active_tol->goal_time_tolerance != 0.0)
          {
            // if we exceed goal_time_tolerance set it to aborted
            if (time_difference > active_tol->goal_time_tolerance)
            {
              within_goal_time = false;
              // print once, goal will be aborted afterwards
              check_state_tolerance_per_joint(state_error_, index, default_tolerances_.goal_state_tolerance[index],
                                              true /* show_errors */);
            }
          }
        }
      }

      // set values for next hardware write() if tolerance is met
      if (!tolerance_violated_while_moving && within_goal_time)
      {
        if (use_closed_loop_pid_adapter_)
        {
          // Update PIDs
          for (auto i = 0ul; i < dof_; ++i)
          {
            tmp_command_[i] = (state_desired_.velocities[i] * ff_velocity_scale_[i]) +
                              pids_[i]->compute_command(state_error_.positions[i], state_error_.velocities[i],
                                                       static_cast<rcl_duration_value_t>(period.nanoseconds()));
          }
        }

        // set values for next hardware write()
        if (has_position_command_interface_)
        {
          assign_interface_from_point(joint_command_interface_[0], state_desired_.positions);
        }
        if (has_velocity_command_interface_)
        {
          if (use_closed_loop_pid_adapter_)
          {
            assign_interface_from_point(joint_command_interface_[1], tmp_command_);
          }
          else
          {
            assign_interface_from_point(joint_command_interface_[1], state_desired_.velocities);
          }
        }
        if (has_acceleration_command_interface_)
        {
          assign_interface_from_point(joint_command_interface_[2], state_desired_.accelerations);
        }
        if (has_effort_command_interface_)
        {
          assign_interface_from_point(joint_command_interface_[3], tmp_command_);
        }

        // store the previous command. Used in open-loop control mode
        last_commanded_state_ = state_desired_;
      }

      if (active_goal)
      {
        // send feedback
        auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
        feedback->header.stamp = last_scaled_time_;
        feedback->joint_names = params_.joints;

        feedback->actual = state_current_;
        feedback->desired = state_desired_;
        feedback->error = state_error_;
        active_goal->setFeedback(feedback);

        // check abort
        if (tolerance_violated_while_moving)
        {
          auto result = std::make_shared<FollowJTrajAction::Result>();
          result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
          result->set__error_string("Aborted due to path tolerance violation");
          active_goal->setAborted(result);
          // TODO(matthew-reynolds): Need a lock-free write here
          // See https://github.com/ros-controls/ros2_controllers/issues/168
          rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
          rt_has_pending_goal_ = false;
          RCLCPP_WARN(logger, "Aborted due to state tolerance violation");
          new_trajectory_msg_.reset();
          new_trajectory_msg_.initRT(set_hold_position());
        }
        // check goal tolerance
        else if (!before_last_point)
        {
          if (!outside_goal_tolerance)
          {
            auto result = std::make_shared<FollowJTrajAction::Result>();
            result->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
            result->set__error_string("Goal successfully reached!");
            active_goal->setSucceeded(result);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            rt_has_pending_goal_ = false;
            RCLCPP_INFO(logger, "Goal reached, success!");
            new_trajectory_msg_.reset();
            new_trajectory_msg_.initRT(set_success_trajectory_point());
          }
          else if (!within_goal_time)
          {
            const std::string error_string =
                "Aborted due to goal_time_tolerance exceeding by " + std::to_string(time_difference) + " seconds";
            auto result = std::make_shared<FollowJTrajAction::Result>();
            result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
            result->set__error_string(error_string);
            active_goal->setAborted(result);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            rt_has_pending_goal_ = false;
            RCLCPP_WARN(logger, "%s", error_string.c_str());
            new_trajectory_msg_.reset();
            new_trajectory_msg_.initRT(set_hold_position());
          }
        }
      }
      else if (tolerance_violated_while_moving && !rt_has_pending_goal_)
      {
        // we need to ensure that there is no pending goal -> we get a race condition otherwise
        RCLCPP_ERROR(logger, "Holding position due to state tolerance violation");
        new_trajectory_msg_.reset();
        new_trajectory_msg_.initRT(set_hold_position());
      }
      else if (!before_last_point && !within_goal_time && !rt_has_pending_goal_)
      {
        RCLCPP_ERROR(logger, "Exceeded goal_time_tolerance: holding position...");
        new_trajectory_msg_.reset();
        new_trajectory_msg_.initRT(set_hold_position());
      }
      // else, run another cycle while waiting for outside_goal_tolerance
      // to be satisfied (will stay in this state until new message arrives)
      // or outside_goal_tolerance violated within the goal_time_tolerance
    }
  }

  publish_state(time, state_desired_, state_current_, state_error_);
  return controller_interface::return_type::OK;
}

double ScaledJointTrajectoryController::first_order_lag_filter(const double filter_input)
{
  // discrete time first order lag filter
  folag_state_ = (1 - folag_h_ / folag_tau_) * folag_state_ + (folag_h_ / folag_tau_) * filter_input;
  return folag_state_;
}

void ScaledJointTrajectoryController::time_scale_callback(const std::shared_ptr<std_msgs::msg::Int32> msg)
{
  // Clamp the slider value between 1 and 100 (1% to 100%)
  int clamped_value = std::clamp(msg->data, 0, 100);
  time_scale_value_.store(clamped_value);
}

}  // namespace fanuc_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fanuc_controllers::ScaledJointTrajectoryController, controller_interface::ControllerInterface)
