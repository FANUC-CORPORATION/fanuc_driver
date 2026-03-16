# SPDX-FileCopyrightText: 2025, FANUC America Corporation
# SPDX-FileCopyrightText: 2025, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_series = LaunchConfiguration("robot_series")
    robot_ip = LaunchConfiguration("robot_ip")
    ros2_control_config = LaunchConfiguration("ros2_control_config")
    gpio_configuration = LaunchConfiguration("gpio_configuration")
    launch_rviz = LaunchConfiguration("launch_rviz")

    robot_model_str = robot_model.perform(context)
    robot_series_str = robot_series.perform(context)

    if robot_series_str == "crx":
        urdf_xacro_file = robot_model_str + ".urdf.xacro"
    else:
        urdf_xacro_file = "6dof_robot.urdf.xacro"

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("fanuc_hardware_interface"), "robot", urdf_xacro_file]
            ),
            " ",
            "robot_series:=",
            robot_series,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "gpio_configuration:=",
            gpio_configuration,
            " ",
            "robot_model:=",
            robot_model,
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description, value_type=str)
    }

    ros_parameters = [robot_description, ros2_control_config]
    nodes_to_launch = []
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=ros_parameters,
        output="both",
    )
    nodes_to_launch.append(control_node)

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    nodes_to_launch.append(robot_state_pub_node)

    rviz_file = PathJoinSubstitution(
        [
            FindPackageShare(
                PythonExpression(['"fanuc_" + "', robot_series, '" + "_description"'])
            ),
            "rviz",
            PythonExpression(['"view_" + "', robot_series, '" + ".rviz"']),
        ]
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

    controller_spawner_processes = [
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 joint_state_broadcaster"
            ],
            shell=True,
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 joint_trajectory_controller"
            ],
            shell=True,
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 fanuc_gpio_controller"
            ],
            shell=True,
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 fanuc_force_sensor_broadcaster"
            ],
            shell=True,
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 force_torque_sensor_broadcaster"
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
        ),
        DeclareLaunchArgument(
            "robot_series",
            default_value="crx",
            description='The robot series such as "crx" (required).',
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
            description="YAML file configuration to specify the IO streaming data.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Specify whether or not to open RVIZ.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
