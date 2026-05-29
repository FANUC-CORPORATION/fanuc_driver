# SPDX-FileCopyrightText: 2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2026, FANUC CORPORATION
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
    rarm_model = LaunchConfiguration("rarm_model")
    larm_model = LaunchConfiguration("larm_model")
    rarm_ip = LaunchConfiguration("rarm_ip")
    larm_ip = LaunchConfiguration("larm_ip")
    rarm_robot_series = LaunchConfiguration("rarm_robot_series")
    larm_robot_series = LaunchConfiguration("larm_robot_series")
    rarm_prefix = LaunchConfiguration("rarm_prefix")
    larm_prefix = LaunchConfiguration("larm_prefix")
    rarm_namespace = LaunchConfiguration("rarm_namespace")
    larm_namespace = LaunchConfiguration("larm_namespace")
    rarm_child_link = LaunchConfiguration("rarm_child_link")
    larm_child_link = LaunchConfiguration("larm_child_link")
    rarm_ros2_control_config = LaunchConfiguration("rarm_ros2_control_config")
    larm_ros2_control_config = LaunchConfiguration("larm_ros2_control_config")
    rarm_gpio_config_package = LaunchConfiguration("rarm_gpio_config_package")
    rarm_gpio_config_path = LaunchConfiguration("rarm_gpio_config_path")
    larm_gpio_config_package = LaunchConfiguration("larm_gpio_config_package")
    larm_gpio_config_path = LaunchConfiguration("larm_gpio_config_path")
    rarm_use_mock = LaunchConfiguration("rarm_use_mock")
    larm_use_mock = LaunchConfiguration("larm_use_mock")
    rarm_origin_x = LaunchConfiguration("rarm_origin_x")
    rarm_origin_y = LaunchConfiguration("rarm_origin_y")
    rarm_origin_z = LaunchConfiguration("rarm_origin_z")
    rarm_origin_rr = LaunchConfiguration("rarm_origin_rr")
    rarm_origin_rp = LaunchConfiguration("rarm_origin_rp")
    rarm_origin_ry = LaunchConfiguration("rarm_origin_ry")
    larm_origin_x = LaunchConfiguration("larm_origin_x")
    larm_origin_y = LaunchConfiguration("larm_origin_y")
    larm_origin_z = LaunchConfiguration("larm_origin_z")
    larm_origin_rr = LaunchConfiguration("larm_origin_rr")
    larm_origin_rp = LaunchConfiguration("larm_origin_rp")
    larm_origin_ry = LaunchConfiguration("larm_origin_ry")

    nodes_to_launch = []

    # Conditionally include the appropriate control launch file
    joint_state_merger_node = Node(
        package="fanuc_dual_arm_example",
        executable="joint_state_merger.py",
        output="log",
    )
    nodes_to_launch.append(joint_state_merger_node)

    include_fanuc_control1 = IncludeLaunchDescription(
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
            "robot_ip": rarm_ip,
            "robot_model": rarm_model,
            "robot_series": rarm_robot_series,
            "ros2_control_config": rarm_ros2_control_config,
            "gpio_config_package": rarm_gpio_config_package,
            "gpio_config_path": rarm_gpio_config_path,
            "use_mock": rarm_use_mock,
            "prefix": rarm_prefix,
            "namespace": rarm_namespace,
            "child_link": rarm_child_link,
            "launch_rviz": "false",
            "origin_x": rarm_origin_x,
            "origin_y": rarm_origin_y,
            "origin_z": rarm_origin_z,
            "origin_rr": rarm_origin_rr,
            "origin_rp": rarm_origin_rp,
            "origin_ry": rarm_origin_ry,
        }.items(),
        condition=UnlessCondition(rarm_use_mock),
    )
    nodes_to_launch.append(include_fanuc_control1)

    include_fanuc_control2 = IncludeLaunchDescription(
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
            "robot_ip": larm_ip,
            "robot_model": larm_model,
            "robot_series": larm_robot_series,
            "ros2_control_config": larm_ros2_control_config,
            "gpio_config_package": larm_gpio_config_package,
            "gpio_config_path": larm_gpio_config_path,
            "use_mock": larm_use_mock,
            "prefix": larm_prefix,
            "namespace": larm_namespace,
            "child_link": larm_child_link,
            "launch_rviz": "false",
            "origin_x": larm_origin_x,
            "origin_y": larm_origin_y,
            "origin_z": larm_origin_z,
            "origin_rr": larm_origin_rr,
            "origin_rp": larm_origin_rp,
            "origin_ry": larm_origin_ry,
        }.items(),
        condition=UnlessCondition(larm_use_mock),
    )
    nodes_to_launch.append(include_fanuc_control2)

    include_fanuc_mock_control1 = IncludeLaunchDescription(
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
            "robot_model": rarm_model,
            "robot_series": rarm_robot_series,
            "ros2_control_config": rarm_ros2_control_config,
            "gpio_config_package": rarm_gpio_config_package,
            "gpio_config_path": rarm_gpio_config_path,
            "prefix": rarm_prefix,
            "namespace": rarm_namespace,
            "child_link": rarm_child_link,
            "launch_rviz": "false",
            "origin_x": rarm_origin_x,
            "origin_y": rarm_origin_y,
            "origin_z": rarm_origin_z,
            "origin_rr": rarm_origin_rr,
            "origin_rp": rarm_origin_rp,
            "origin_ry": rarm_origin_ry,
        }.items(),
        condition=IfCondition(rarm_use_mock),
    )
    nodes_to_launch.append(include_fanuc_mock_control1)

    include_fanuc_mock_control2 = IncludeLaunchDescription(
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
            "robot_model": larm_model,
            "robot_series": larm_robot_series,
            "ros2_control_config": larm_ros2_control_config,
            "gpio_config_package": larm_gpio_config_package,
            "gpio_config_path": larm_gpio_config_path,
            "prefix": larm_prefix,
            "namespace": larm_namespace,
            "child_link": larm_child_link,
            "launch_rviz": "false",
            "origin_x": larm_origin_x,
            "origin_y": larm_origin_y,
            "origin_z": larm_origin_z,
            "origin_rr": larm_origin_rr,
            "origin_rp": larm_origin_rp,
            "origin_ry": larm_origin_ry,
        }.items(),
        condition=IfCondition(larm_use_mock),
    )
    nodes_to_launch.append(include_fanuc_mock_control2)

    description_arguments = {
        "robot_ip1": rarm_ip.perform(context),
        "robot_ip2": larm_ip.perform(context),
        # "use_mock": use_mock.perform(context),
    }

    urdf_full_path = os.path.join(
        get_package_share_directory("fanuc_dual_arm_example"),
        "urdf",
        f"{robot_model.perform(context)}.urdf.xacro",
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_model.perform(context), package_name="fanuc_dual_arm_example"
        )
        .robot_description(file_path=urdf_full_path, mappings=description_arguments)
        .robot_description_semantic(
            file_path=f"srdf/{robot_model.perform(context)}.srdf"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
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
        name="rviz2_multi",
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
            "rarm_model",
            default_value="crx10ia_l",
            description="The robot model (required).",
        ),
        DeclareLaunchArgument(
            "larm_model",
            default_value="lrmate200id7l",
            description="The robot model (required).",
        ),
        DeclareLaunchArgument(
            "rarm_ip",
            default_value="192.168.1.100",
            description="The robot's IP address.",
        ),
        DeclareLaunchArgument(
            "larm_ip",
            default_value="192.168.1.101",
            description="The robot's IP address.",
        ),
        DeclareLaunchArgument(
            "rarm_robot_series",
            default_value="crx",
            description="The robot series name (required).",
        ),
        DeclareLaunchArgument(
            "larm_robot_series",
            default_value="lrmate",
            description="The robot series name (required).",
        ),
        DeclareLaunchArgument(
            "rarm_ros2_control_config",
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
            "larm_ros2_control_config",
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
            "rarm_gpio_config_package",
            default_value="fanuc_hardware_interface",
            description="The package name where gpio_configuration file exists",
        ),
        DeclareLaunchArgument(
            "rarm_gpio_config_path",
            default_value="config/example_gpio_config_small.yaml",
            description="The gpio_configuration file path in gpio_config_package",
        ),
        DeclareLaunchArgument(
            "larm_gpio_config_package",
            default_value="fanuc_hardware_interface",
            description="The package name where gpio_configuration file exists",
        ),
        DeclareLaunchArgument(
            "larm_gpio_config_path",
            default_value="config/example_gpio_config_small.yaml",
            description="The gpio_configuration file path in gpio_config_package",
        ),
        DeclareLaunchArgument(
            "rarm_use_mock",
            default_value="true",
            description="Whether to use a mock hardware interface.",
        ),
        DeclareLaunchArgument(
            "larm_use_mock",
            default_value="true",
            description="Whether to use a mock hardware interface.",
        ),
        DeclareLaunchArgument(
            "robot_model",
            default_value="dual_arm",
            description="The two robot model.",
        ),
        DeclareLaunchArgument(
            "rarm_prefix",
            default_value="rarm_",
            description="The robot link name prefix.",
        ),
        DeclareLaunchArgument(
            "larm_prefix",
            default_value="larm_",
            description="The robot link name prefix.",
        ),
        DeclareLaunchArgument(
            "rarm_namespace",
            default_value="rarm",
            description="The robot controller prefix.",
        ),
        DeclareLaunchArgument(
            "larm_namespace",
            default_value="larm",
            description="The robot controller prefix",
        ),
        DeclareLaunchArgument(
            "rarm_child_link",
            default_value="rarm_end_effector",
            description="The robot child link name",
        ),
        DeclareLaunchArgument(
            "larm_child_link",
            default_value="larm_end_effector",
            description="The robot child link name",
        ),
        DeclareLaunchArgument(
            "rarm_origin_x",
            default_value="0",
            description="X transition from parent_link to base_link",
        ),
        DeclareLaunchArgument(
            "rarm_origin_y",
            default_value="-1.0",
            description="Y transition from parent_link to base_link",
        ),
        DeclareLaunchArgument(
            "rarm_origin_z",
            default_value="0",
            description="Z transition from parent_link to base_link",
        ),
        DeclareLaunchArgument(
            "rarm_origin_rr",
            default_value="0",
            description="Roll rotation from parent_link to base_link",
        ),
        DeclareLaunchArgument(
            "rarm_origin_rp",
            default_value="0",
            description="Pitch rotation from parent_link to base_link",
        ),
        DeclareLaunchArgument(
            "rarm_origin_ry",
            default_value="0",
            description="Yaw rotation from parent_link to base_link",
        ),
        DeclareLaunchArgument(
            "larm_origin_x",
            default_value="0",
            description="X transition from parent_link to base_link",
        ),
        DeclareLaunchArgument(
            "larm_origin_y",
            default_value="1.0",
            description="Y transition from parent_link to base_link",
        ),
        DeclareLaunchArgument(
            "larm_origin_z",
            default_value="0",
            description="Z transition from parent_link to base_link",
        ),
        DeclareLaunchArgument(
            "larm_origin_rr",
            default_value="0",
            description="Roll rotation from parent_link to base_link",
        ),
        DeclareLaunchArgument(
            "larm_origin_rp",
            default_value="0",
            description="Pitch rotation from parent_link to base_link",
        ),
        DeclareLaunchArgument(
            "larm_origin_ry",
            default_value="0",
            description="Yaw rotation from parent_link to base_link",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
