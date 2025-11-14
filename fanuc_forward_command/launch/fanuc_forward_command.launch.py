#!/usr/bin/env python3

# Copyright 2025 FANUC CORPORATION
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import OpaqueFunction, ExecuteProcess
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_ip = LaunchConfiguration("robot_ip")
    ros2_control_config = LaunchConfiguration("ros2_control_config")
    gpio_configuration = LaunchConfiguration("gpio_configuration")
    use_mock = LaunchConfiguration("use_mock")
    launch_rviz = LaunchConfiguration("launch_rviz")

    nodes_to_launch = []

    # Generate robot description (same as working fanuc_moveit.launch.py)
    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("fanuc_hardware_interface"), "robot", ""]
            ),
            "6dof_robot.urdf.xacro",
            " ",
            "robot_series:=crx",
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "robot_model:=",
            robot_model,
            " ",
            "gpio_configuration:=",
            gpio_configuration,
            " ",
            "use_mock:=",
            use_mock,
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description, value_type=str)
    }

    # ROS2 Control Node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_control_config],
        output="both",
    )
    nodes_to_launch.append(control_node)

    # Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    nodes_to_launch.append(robot_state_pub_node)

    # RViz
    rviz_file = PathJoinSubstitution(
        [FindPackageShare("fanuc_crx_description"), "rviz", "view_crx.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        arguments=["--display-config", rviz_file],
        condition=IfCondition(launch_rviz),
    )
    nodes_to_launch.append(rviz_node)

    # Controller spawners - only the ones we want
    controller_spawner_processes = [
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner "
                "--controller-manager-timeout 180 joint_state_broadcaster"
            ],
            shell=True,
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner "
                "--controller-manager-timeout 180 fanuc_gpio_controller"
            ],
            shell=True,
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner "
                "--controller-manager-timeout 180 forward_position_controller"
            ],
            shell=True,
            output="screen",
        ),
    ]

    return nodes_to_launch + controller_spawner_processes


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_model",
            description="The robot model (required).",
            choices=["crx5ia", "crx10ia", "crx10ia_l", "crx20ia_l", "crx30ia"],
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.100",
            description="The robot's IP address.",
        ),
        DeclareLaunchArgument(
            "ros2_control_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "config",
                    "ros2_controllers.yaml",
                ]
            ),
            description="ROS 2 control configuration file the controllers.",
        ),
        DeclareLaunchArgument(
            "gpio_configuration",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "config",
                    "example_gpio_config.yaml",
                ]
            ),
            description="YAML file configuration to specify the GPIO configuration..",
        ),
        DeclareLaunchArgument(
            "use_mock",
            default_value="false",
            description="Whether to use a mock hardware interface.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Whether to launch RViz for visualization.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
