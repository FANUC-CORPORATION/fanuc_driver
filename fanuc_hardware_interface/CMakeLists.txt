# SPDX-FileCopyrightText: 2025, FANUC America Corporation
# SPDX-FileCopyrightText: 2025, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(fanuc_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall)
  add_compile_options(-Wextra)
  add_compile_options(-Wno-unused-parameter)
endif()
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(fanuc_libs REQUIRED)

add_library(fanuc_robot_driver SHARED src/hardware_interface.cpp)

target_link_libraries(fanuc_robot_driver PUBLIC
  pluginlib::pluginlib
  fanuc_libs::rmi
  fanuc_libs::stream_motion
  fanuc_libs::fanuc_client
  fanuc_libs::gpio_config
)

if(${hardware_interface_VERSION} VERSION_LESS 2.52.0)
  target_link_libraries(fanuc_robot_driver PUBLIC ${hardware_interface_LIBRARIES})
  target_include_directories(fanuc_robot_driver PRIVATE ${hardware_interface_INCLUDE_DIRS})
else()
  target_link_libraries(fanuc_robot_driver PUBLIC hardware_interface::hardware_interface)
endif()

target_include_directories(fanuc_robot_driver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

add_library(fanuc_hardware_interface_constants INTERFACE)
target_include_directories(fanuc_hardware_interface_constants INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)



if(BUILD_TESTING)
  #  ...
endif()

install(
  TARGETS fanuc_robot_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include/
)

install(
    TARGETS fanuc_hardware_interface_constants
    EXPORT export_fanuc_hardware_interface_constants
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

ament_export_targets(export_fanuc_hardware_interface_constants)

install(
    DIRECTORY
    config
    launch
    robot
    DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
