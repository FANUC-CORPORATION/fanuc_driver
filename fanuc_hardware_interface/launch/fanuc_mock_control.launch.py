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
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_series = LaunchConfiguration("robot_series")
    ros2_control_config = LaunchConfiguration("ros2_control_config")
    launch_rviz = LaunchConfiguration("launch_rviz")
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")
    child_link = LaunchConfiguration("child_link")
    origin_x = LaunchConfiguration("origin_x")
    origin_y = LaunchConfiguration("origin_y")
    origin_z = LaunchConfiguration("origin_z")
    origin_rr = LaunchConfiguration("origin_rr")
    origin_rp = LaunchConfiguration("origin_rp")
    origin_ry = LaunchConfiguration("origin_ry")

    robot_model_str = robot_model.perform(context)
    robot_series_str = robot_series.perform(context)
    namespace_str = namespace.perform(context)

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
            "robot_ip:=1.1.1.1",
            " ",
            "use_mock:=true",
            " ",
            "robot_series:=",
            robot_series,
            " ",
            "robot_model:=",
            robot_model,
            " ",
            "prefix:=",
            prefix,
            " ",
            "child_link:=",
            child_link,
            " ",
            "origin_xyz:='",
            origin_x,
            " ",
            origin_y,
            " ",
            origin_z,
            "' ",
            "origin_rpy:='",
            origin_rr,
            " ",
            origin_rp,
            " ",
            origin_ry,
            "' ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description, value_type=str)
    }

    ros_parameters = [
        robot_description,
        ParameterFile(ros2_control_config, allow_substs=True),
    ]
    nodes_to_launch = []
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=ros_parameters,
        namespace=namespace,
        output="both",
    )
    nodes_to_launch.append(control_node)

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
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
        namespace=namespace,
        name="rviz2",
        output="both",
        arguments=["--display-config", rviz_file],
        condition=IfCondition(launch_rviz),
    )
    nodes_to_launch.append(rviz_node)

    slider_test_node = Node(
        package="slider_publisher",
        executable="slider_gui_node",
        name="slider_gui_node",
        namespace=namespace,
        output="both",
    )
    nodes_to_launch.append(slider_test_node)

    if namespace_str == "":
        controller_manager_name_argument = ""
    else:
        controller_manager_name_argument = (
            " -c /" + namespace_str + "/controller_manager"
        )

    controller_spawner_processes = [
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 joint_state_broadcaster",
                controller_manager_name_argument,
            ],
            shell=True,
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 joint_trajectory_controller",
                controller_manager_name_argument,
            ],
            shell=True,
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 fanuc_gpio_controller",
                controller_manager_name_argument,
            ],
            shell=True,
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 fanuc_force_sensor_broadcaster",
                controller_manager_name_argument,
            ],
            shell=True,
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 force_torque_sensor_broadcaster",
                controller_manager_name_argument,
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
            "launch_rviz",
            default_value="true",
            description="Specify whether or not to open RVIZ.",
        ),
        DeclareLaunchArgument(
            "parent_link",
            default_value="world",
            description="Parent link name of the urdf",
        ),
        DeclareLaunchArgument(
            "child_link",
            default_value="end_effector",
            description="Child link name of the urdf",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace of the ROS 2 nodes",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Name prefix of the robot",
        ),
        DeclareLaunchArgument(
            "origin_x",
            default_value="0",
            description="X transition from parent_link to base_link",
        ),
        DeclareLaunchArgument(
            "origin_y",
            default_value="0",
            description="Y transition from parent_link to base_link",
        ),
        DeclareLaunchArgument(
            "origin_z",
            default_value="0",
            description="Z transition from parent_link to base_link",
        ),
        DeclareLaunchArgument(
            "origin_rr",
            default_value="0",
            description="Roll rotation from parent_link to base_link",
        ),
        DeclareLaunchArgument(
            "origin_rp",
            default_value="0",
            description="Pitch rotation from parent_link to base_link",
        ),
        DeclareLaunchArgument(
            "origin_ry",
            default_value="0",
            description="Yaw rotation from parent_link to base_link",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
