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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.1.100",
        description="IP address of the FANUC robot",
    )

    use_mock_arg = DeclareLaunchArgument(
        "use_mock",
        default_value="true",
        description="Whether to use a mock hardware interface",
    )

    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Whether to launch RViz for visualization",
    )

    robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value="crx10ia",
        description="The robot model",
        choices=["crx5ia", "crx10ia", "crx10ia_l", "crx20ia_l", "crx30ia"],
    )

    # Include the forward command launch file
    forward_command_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("fanuc_forward_command"),
                        "launch",
                        "fanuc_forward_command.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_mock": LaunchConfiguration("use_mock"),
            "robot_model": LaunchConfiguration("robot_model"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
        }.items(),
    )

    # Sine wave publisher node
    sine_wave_publisher = Node(
        package="fanuc_forward_command",
        executable="sine_wave_publisher.py",
        name="sine_wave_publisher",
        output="screen",
    )

    return LaunchDescription(
        [
            robot_ip_arg,
            use_mock_arg,
            launch_rviz_arg,
            robot_model_arg,
            forward_command_launch,
            sine_wave_publisher,
        ]
    )
