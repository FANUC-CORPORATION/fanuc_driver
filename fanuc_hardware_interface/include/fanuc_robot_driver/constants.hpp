// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
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

constexpr auto kForceInterfaceName = "Force";
constexpr auto kForceXType = "force_x";
constexpr auto kForceYType = "force_y";
constexpr auto kForceZType = "force_z";
constexpr auto kMomentXType = "moment_x";
constexpr auto kMomentYType = "moment_y";
constexpr auto kMomentZType = "moment_z";
constexpr auto kForceSensorType = "fs_type";

}  // namespace fanuc_robot_driver
