cmake_minimum_required(VERSION 3.8)
project(fanuc_forward_command)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(forward_command_controller REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(fanuc_hardware_interface REQUIRED)

# Install launch files
install(FILES
  launch/fanuc_forward_command.launch.py
  launch/fanuc_sine_wave_demo.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# No config files to install - using standard ros2_controllers.yaml

# Install scripts
install(FILES
  scripts/sine_wave_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
