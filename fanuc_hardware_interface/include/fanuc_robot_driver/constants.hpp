// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

namespace fanuc_robot_driver
{

constexpr auto kRobotStatusInterfaceName = "Status";
constexpr auto kStatusInErrorType = "in_error";
constexpr auto kStatusTPEnabledType = "tp_enabled";
constexpr auto kStatusEStoppedType = "e_stopped";
constexpr auto kStatusMotionPossibleType = "motion_possible";
constexpr auto kStatusContactStopModeType = "contact_stop_mode";
constexpr auto kStatusCollaborativeSpeedScalingType = "collaborative_speed_scaling";

constexpr auto kConnectionStatusName = "ConnectionStatus";
constexpr auto kIsConnectedType = "is_connected";
constexpr auto kMotionCommandType = "motion_command_type";
constexpr int MotionCommandTypeNone = 0;
constexpr int MotionCommandTypeInitialState = 1;
constexpr int MotionCommandTypePosition = 2;
constexpr int MotionCommandTypeRMI = 3;

constexpr auto kForceInterfaceName = "Force";
constexpr auto kForceXType = "force_x";
constexpr auto kForceYType = "force_y";
constexpr auto kForceZType = "force_z";
constexpr auto kMomentXType = "moment_x";
constexpr auto kMomentYType = "moment_y";
constexpr auto kMomentZType = "moment_z";
constexpr auto kForceSensorType = "fs_type";

constexpr auto kRMIInterfaceName = "RMI";
constexpr auto kRMICommandName = "control";

}  // namespace fanuc_robot_driver
