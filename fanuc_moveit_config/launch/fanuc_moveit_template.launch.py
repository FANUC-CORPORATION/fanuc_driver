# SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_series = LaunchConfiguration("robot_series")
    moveit_config_name = LaunchConfiguration("moveit_config")
    robot_ip = LaunchConfiguration("robot_ip")
    ros2_control_config = LaunchConfiguration("ros2_control_config")
    gpio_config_package = LaunchConfiguration("gpio_config_package")
    gpio_config_path = LaunchConfiguration("gpio_config_path")
    use_mock = LaunchConfiguration("use_mock")
    motion_control = LaunchConfiguration("motion_control")

    nodes_to_launch = []

    # Conditionally include the appropriate control launch file
    include_fanuc_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "launch",
                    "fanuc_physical_control.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "robot_model": robot_model,
            "robot_series": robot_series,
            "robot_ip": robot_ip,
            "gpio_config_package": gpio_config_package,
            "gpio_config_path": gpio_config_path,
            "ros2_control_config": ros2_control_config,
            "launch_rviz": "false",
            "use_mock": use_mock,
            "motion_control": motion_control,
        }.items(),
        condition=UnlessCondition(use_mock),
    )
    nodes_to_launch.append(include_fanuc_control)

    include_fanuc_mock_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "launch",
                    "fanuc_mock_control.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "robot_model": robot_model,
            "robot_series": robot_series,
            "gpio_config_package": gpio_config_package,
            "gpio_config_path": gpio_config_path,
            "ros2_control_config": ros2_control_config,
            "launch_rviz": "false",
        }.items(),
        condition=IfCondition(use_mock),
    )
    nodes_to_launch.append(include_fanuc_mock_control)

    description_arguments = {
        "robot_ip": robot_ip.perform(context),
        "robot_model": robot_model.perform(context),
        "robot_series": robot_series.perform(context),
        "use_mock": use_mock.perform(context),
        "gpio_configuration": PathJoinSubstitution(
            [FindPackageShare(gpio_config_package), gpio_config_path]
        ),
    }

    urdf_full_path = os.path.join(
        get_package_share_directory("fanuc_hardware_interface"),
        "robot",
        "6dof_robot.urdf.xacro",
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_model.perform(context),
            package_name=moveit_config_name.perform(context),
        )
        .robot_description(file_path=urdf_full_path, mappings=description_arguments)
        .robot_description_semantic(
            file_path=f"config/{robot_model.perform(context)}.srdf"
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="log",
        parameters=[moveit_config.to_dict()],
    )
    nodes_to_launch.append(move_group_node)

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("fanuc_moveit_config"), "rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        arguments=["--display-config", rviz_file],
    )
    nodes_to_launch.append(rviz_node)

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_model",
            description="The robot model (required).",
        ),
        DeclareLaunchArgument(
            "robot_series",
            description="The robot model (required).",
        ),
        DeclareLaunchArgument(
            "moveit_config",
            description="The package name for moveit config (required).",
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
                    "example_ros2_controllers.yaml",
                ]
            ),
            description="ROS 2 control configuration file the controllers.",
        ),
        DeclareLaunchArgument(
            "gpio_config_package",
            default_value="fanuc_hardware_interface",
            description="The package name where gpio_configuration file exists",
        ),
        DeclareLaunchArgument(
            "gpio_config_path",
            default_value="config/example_gpio_config_small.yaml",
            description="The gpio_configuration file path in gpio_config_package",
        ),
        DeclareLaunchArgument(
            "use_mock",
            default_value="false",
            description="Whether to use a mock hardware interface.",
        ),
        DeclareLaunchArgument(
            "motion_control",
            default_value="1",
            description="Initial motion control state.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
