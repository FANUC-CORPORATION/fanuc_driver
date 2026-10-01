# SPDX-FileCopyrightText: 2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

"""Launch a standalone MoveItPy/Pilz Pick & Place application."""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
import xacro
import yaml


PACKAGE_NAME = "fanuc_pick_and_place_example"


def _set_initial_value(interface, value):
    """Set a mock state-interface initial value in the generated URDF."""
    # Reuse an existing parameter when the source xacro already defines it.
    for parameter in interface.findall("param"):
        if parameter.get("name") == "initial_value":
            parameter.text = str(value)
            return
    parameter = ET.SubElement(interface, "param", {"name": "initial_value"})
    parameter.text = str(value)


def _prepare_mock_status_interfaces(robot_description):
    """Set status interfaces required by the mock controllers."""
    # Edit only the generated description passed to this launch invocation.
    root = ET.fromstring(robot_description)
    for gpio in root.findall(".//gpio"):
        if gpio.get("name") == "ConnectionStatus":
            interfaces = {
                item.get("name"): item for item in gpio.findall("state_interface")
            }
            if "motion_command_type" not in interfaces:
                # GenericSystem xacros omit this interface used by the controller.
                interfaces["motion_command_type"] = ET.SubElement(
                    gpio, "state_interface", {"name": "motion_command_type"}
                )
            _set_initial_value(interfaces["is_connected"], 1.0)
            _set_initial_value(interfaces["motion_command_type"], 1.0)
        elif gpio.get("name") == "Status":
            interfaces = {
                item.get("name"): item for item in gpio.findall("state_interface")
            }
            # A scale of one lets mock trajectories advance at their planned rate.
            _set_initial_value(interfaces["collaborative_speed_scaling"], 1.0)
            _set_initial_value(interfaces["motion_possible"], 1.0)
            _set_initial_value(interfaces["contact_stop_mode"], 0.0)
            _set_initial_value(interfaces["e_stopped"], 0.0)
            _set_initial_value(interfaces["in_error"], 0.0)
            _set_initial_value(interfaces["tp_enabled"], 0.0)
    return ET.tostring(root, encoding="unicode")


def _load_yaml(path):
    """Load and validate a YAML mapping."""
    with open(path, "r", encoding="utf-8") as stream:
        values = yaml.safe_load(stream)
    if not isinstance(values, dict):
        raise RuntimeError(f"YAML root must be a mapping: {path}")
    return values


def _load_ros_parameters(path):
    """Load a wildcard ROS parameter mapping."""
    values = _load_yaml(path)
    parameters = values.get("/**", {}).get("ros__parameters")
    if not isinstance(parameters, dict):
        raise RuntimeError(f"YAML must define /**/ros__parameters: {path}")
    return parameters


def launch_setup(context, *args, **kwargs):
    """Resolve paths/arguments and create the control, MoveIt, RViz, and app nodes."""
    # Resolve substitutions inside OpaqueFunction so paths can use launch args.
    package_share = get_package_share_directory(PACKAGE_NAME)
    hardware_share = get_package_share_directory("fanuc_hardware_interface")
    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    use_mock_text = LaunchConfiguration("use_mock").perform(context).lower()
    if use_mock_text not in ("true", "1", "yes", "false", "0", "no"):
        raise ValueError("use_mock must be true/false, 1/0, or yes/no")
    use_mock = use_mock_text in ("true", "1", "yes")
    gpio_package = LaunchConfiguration("gpio_config_package").perform(context)
    gpio_path = LaunchConfiguration("gpio_config_path").perform(context)
    gpio_file = os.path.join(get_package_share_directory(gpio_package), gpio_path)
    motion_control = LaunchConfiguration("motion_control").perform(context)
    encoding = LaunchConfiguration("encoding").perform(context)
    example_config = LaunchConfiguration("example_config").perform(context)
    rviz_start_delay = float(
        LaunchConfiguration("rviz_start_delay_sec").perform(context)
    )
    if rviz_start_delay < 0.0:
        raise ValueError("rviz_start_delay_sec must be zero or positive")
    mock_controller_update_rate = int(
        LaunchConfiguration("mock_controller_update_rate_hz").perform(context)
    )
    if mock_controller_update_rate <= 0:
        raise ValueError("mock_controller_update_rate_hz must be positive")

    # Use identical xacro mappings for ros2_control and the MoveIt model.
    robot_xacro = os.path.join(hardware_share, "robot", f"{robot_model}.urdf.xacro")
    description_mappings = {
        "robot_ip": robot_ip,
        "gpio_configuration": gpio_file,
        "use_mock": str(use_mock).lower(),
        "motion_control": motion_control,
        "encoding": encoding,
    }
    robot_description = xacro.process_file(
        robot_xacro, mappings=description_mappings
    ).toxml()
    if use_mock:
        # Seed only the mock-only status interfaces needed for motion execution.
        robot_description = _prepare_mock_status_interfaces(robot_description)

    robot_description_parameter = {
        "robot_description": ParameterValue(robot_description, value_type=str)
    }
    controller_parameters = ParameterFile(
        os.path.join(package_share, "config", "ros2_controllers.yaml"),
        allow_substs=True,
    )

    # Build the robot model plus OMPL and Pilz planning pipelines for MoveItPy.
    moveit_config = (
        MoveItConfigsBuilder(robot_model, package_name="fanuc_moveit_config")
        .robot_description(file_path=robot_xacro, mappings=description_mappings)
        .robot_description_semantic(file_path=f"srdf/{robot_model}.srdf")
        .trajectory_execution(
            file_path=os.path.join(package_share, "config", "moveit_controllers.yaml")
        )
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "pilz_industrial_motion_planner"],
        )
        .pilz_cartesian_limits(
            file_path=os.path.join(
                package_share, "config", "pilz_cartesian_limits.yaml"
            )
        )
        .to_moveit_configs()
    )
    moveit_config.planning_pipelines["ompl"] = _load_yaml(
        os.path.join(package_share, "config", "ompl_planning.yaml")
    )
    display_adapter = "default_planning_response_adapters/DisplayMotionPath"
    # Candidate paths stay hidden; the application publishes only its selection.
    for pipeline_name in ("ompl", "pilz_industrial_motion_planner"):
        pipeline = moveit_config.planning_pipelines[pipeline_name]
        pipeline["response_adapters"] = [
            adapter
            for adapter in pipeline.get("response_adapters", [])
            if adapter != display_adapter
        ]
    moveit_config.robot_description = {"robot_description": robot_description}

    control_parameters = [
        robot_description_parameter,
        controller_parameters,
    ]
    if use_mock:
        # Physical robots retain the driver controller-manager update rate.
        control_parameters.append({"update_rate": mock_controller_update_rate})
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="log",
        parameters=control_parameters,
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[robot_description_parameter],
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "30"],
        output="log",
    )
    gpio_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fanuc_gpio_controller", "--controller-manager-timeout", "30"],
        output="log",
    )
    trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager-timeout", "30"],
        output="screen",
    )

    # RViz starts only after the application exposes a ready Planning Scene.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=[
            "--display-config",
            os.path.join(package_share, "rviz", "pick_and_place.rviz"),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
        ],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    # A small service probe prevents RViz from racing MoveItPy initialization.
    scene_ready = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/get_planning_scene",
            "moveit_msgs/srv/GetPlanningScene",
            "{components: {components: 1}}",
        ],
        output="log",
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    # Validate every named profile before launching the application process.
    moveit_py_parameters = _load_yaml(
        os.path.join(package_share, "config", "moveit_py.yaml")
    )
    example_values = _load_ros_parameters(example_config)
    unconstrained_profiles = example_values.get(
        "motion.ompl.unconstrained.parallel_profiles",
        ["ompl_bkpiece", "ompl_bkpiece_coarse", "ompl_bkpiece_fine"],
    )
    single_ompl_profile = example_values.get("ompl_profile", "ompl_rrtc")
    constrained_profiles = example_values.get(
        "motion.ompl.constrained.parallel_profiles",
        [
            "ompl_bkpiece",
            "ompl_bkpiece_coarse",
            "ompl_bkpiece_fine",
        ],
    )
    linear_profile = example_values.get("linear_profile", "pilz_lin")
    profile_groups = {
        "single OMPL": [single_ompl_profile],
        "unconstrained OMPL": unconstrained_profiles,
        "constrained OMPL": constrained_profiles,
        "Pilz LIN": [linear_profile],
    }
    for group_name, profile_names in profile_groups.items():
        if not isinstance(profile_names, list) or not profile_names:
            raise ValueError(f"{group_name} profiles must be a non-empty list")
        if len(set(profile_names)) != len(profile_names):
            raise ValueError(f"{group_name} profiles must be unique")
        for profile_name in profile_names:
            if not isinstance(profile_name, str):
                raise ValueError(f"{group_name} profile names must be strings")
            if profile_name not in moveit_py_parameters:
                raise RuntimeError(f"Unknown {group_name} profile: {profile_name}")
    # Merge MoveIt, named profiles, task YAML, and launch-only overrides.
    application = Node(
        package=PACKAGE_NAME,
        executable="fanuc_pick_and_place_example.py",
        output="screen",
        additional_env={"PYTHONDONTWRITEBYTECODE": "1"},
        parameters=[
            moveit_config.to_dict(),
            moveit_py_parameters,
            example_config,
            {
                "use_mock": use_mock,
                "max_cycles": ParameterValue(
                    LaunchConfiguration("max_cycles"), value_type=int
                ),
            },
        ],
    )
    # Spawn controllers serially to reduce startup CPU and discovery contention.
    start_gpio_after_joint_state = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_spawner, on_exit=[gpio_spawner])
    )
    start_trajectory_after_gpio = RegisterEventHandler(
        OnProcessExit(target_action=gpio_spawner, on_exit=[trajectory_spawner])
    )
    start_application_after_controller = RegisterEventHandler(
        OnProcessExit(target_action=trajectory_spawner, on_exit=[application])
    )
    # Probe the scene after application start, then wait once more before RViz.
    start_scene_probe_after_application = RegisterEventHandler(
        OnProcessStart(target_action=application, on_start=[scene_ready])
    )
    start_rviz_after_scene = RegisterEventHandler(
        OnProcessExit(
            target_action=scene_ready,
            on_exit=[TimerAction(period=rviz_start_delay, actions=[rviz])],
        )
    )
    # Finite runs shut down every launch process when the script exits.
    shutdown_after_application = RegisterEventHandler(
        OnProcessExit(
            target_action=application,
            on_exit=[
                EmitEvent(event=Shutdown(reason="Pick & Place example completed"))
            ],
        )
    )

    return [
        control_node,
        state_publisher,
        start_gpio_after_joint_state,
        start_trajectory_after_gpio,
        start_application_after_controller,
        start_scene_probe_after_application,
        start_rviz_after_scene,
        shutdown_after_application,
        joint_state_spawner,
    ]


def generate_launch_description():
    """Declare user-facing launch arguments."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_model",
                default_value="crx10ia_l",
                choices=[
                    "crx3ia",
                    "crx5ia",
                    "crx10ia",
                    "crx10ia_l",
                    "crx20ia_l",
                    "crx30ia",
                ],
                description="CRX model; default task coordinates are tuned for crx10ia_l.",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="192.168.1.100",
                description="Robot controller IP address (ignored by mock).",
            ),
            DeclareLaunchArgument(
                "use_mock",
                default_value="true",
                description="Use GenericSystem instead of a physical robot.",
            ),
            DeclareLaunchArgument(
                "mock_controller_update_rate_hz",
                default_value="100",
                description="ros2_control update rate used only in mock mode.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Open the tutorial RViz configuration.",
            ),
            DeclareLaunchArgument(
                "rviz_start_delay_sec",
                default_value="1.0",
                description="Additional delay after the Planning Scene service responds.",
            ),
            DeclareLaunchArgument(
                "max_cycles",
                default_value="0",
                description=(
                    "Number of A-to-B-to-A round trips; 0 loops until Ctrl+C."
                ),
            ),
            DeclareLaunchArgument(
                "example_config",
                default_value=os.path.join(
                    get_package_share_directory(PACKAGE_NAME),
                    "config",
                    "pick_and_place.yaml",
                ),
                description="Application YAML for task poses, geometry, I/O, and safeguards.",
            ),
            DeclareLaunchArgument(
                "gpio_config_package",
                default_value=PACKAGE_NAME,
                description="Package containing the cyclic hand-I/O configuration.",
            ),
            DeclareLaunchArgument(
                "gpio_config_path",
                default_value="config/gpio_topic_config.yaml",
                description="Cyclic GPIO configuration used by the hand.",
            ),
            DeclareLaunchArgument(
                "motion_control",
                default_value="1",
                description="Initial physical robot motion-control state.",
            ),
            DeclareLaunchArgument(
                "encoding",
                default_value="UTF-8",
                description="Physical robot controller string encoding.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
