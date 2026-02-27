# SPDX-FileCopyrightText: 2025, FANUC America Corporation
# SPDX-FileCopyrightText: 2025, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(fanuc_controllers)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
get_property(importTargets DIRECTORY "${CMAKE_SOURCE_DIR}" PROPERTY IMPORTED_TARGETS)
message(STATUS "Imported targets: ${importTargets}")



if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(fanuc_hardware_interface REQUIRED)
find_package(fanuc_libs REQUIRED)
find_package(fanuc_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Include directories
include_directories(include)

# Build the scaled joint trajectory controller library
add_library(scaled_joint_trajectory_controller SHARED
    src/scaled_joint_trajectory_controller.cpp
)

# Link dependencies
ament_target_dependencies(scaled_joint_trajectory_controller
    controller_interface
    controller_manager_msgs
    geometry_msgs
    hardware_interface
    joint_trajectory_controller
    lifecycle_msgs
    pluginlib
    rclcpp
    realtime_tools
    sensor_msgs
    std_msgs
    trajectory_msgs
)
target_link_libraries(scaled_joint_trajectory_controller fanuc_hardware_interface::fanuc_hardware_interface_constants)

# Build the FANUC GPIO controller library
add_library(fanuc_gpio_controller SHARED
    src/fanuc_gpio_controller.cpp
)

# Link dependencies
ament_target_dependencies(fanuc_gpio_controller
    ament_index_cpp
    controller_interface
    controller_manager_msgs
    fanuc_msgs
    hardware_interface
    joint_trajectory_controller
    lifecycle_msgs
    pluginlib
    rclcpp
    realtime_tools
    std_msgs
    trajectory_msgs
)
target_link_libraries(fanuc_gpio_controller fanuc_libs::fanuc_client fanuc_libs::gpio_config
    fanuc_hardware_interface::fanuc_hardware_interface_constants)

# Build the FANUC force sensor broadcaster library
add_library(fanuc_force_sensor_broadcaster SHARED
    src/fanuc_force_sensor_broadcaster.cpp
)

# Link dependencies
ament_target_dependencies(fanuc_force_sensor_broadcaster
    ament_index_cpp
    controller_interface
    controller_manager_msgs
    fanuc_msgs
    hardware_interface
    joint_trajectory_controller
    lifecycle_msgs
    pluginlib
    rclcpp
    realtime_tools
    std_msgs
    trajectory_msgs
)
target_link_libraries(fanuc_force_sensor_broadcaster fanuc_libs::fanuc_client
    fanuc_hardware_interface::fanuc_hardware_interface_constants)

if (BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  enable_testing()
  add_subdirectory(test)
endif ()

# Install the library
install(TARGETS
    scaled_joint_trajectory_controller fanuc_gpio_controller fanuc_force_sensor_broadcaster
    DESTINATION lib/${PROJECT_NAME}
)

# Install plugin description
install(FILES
    scaled_joint_trajectory_controller_plugins.xml
    fanuc_gpio_controller_plugins.xml
    fanuc_force_sensor_broadcaster_plugins.xml
    DESTINATION share/${PROJECT_NAME}
)

# Export pluginlib plugins
pluginlib_export_plugin_description_file(controller_interface scaled_joint_trajectory_controller_plugins.xml)
pluginlib_export_plugin_description_file(controller_interface fanuc_gpio_controller_plugins.xml)
pluginlib_export_plugin_description_file(controller_interface fanuc_force_sensor_broadcaster_plugins.xml)

ament_package()
