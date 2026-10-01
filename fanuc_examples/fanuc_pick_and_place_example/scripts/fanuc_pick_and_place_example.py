#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

"""MoveItPy Pick & Place using OMPL, Pilz LIN, Planning Scene, and FANUC GPIO.

``PickPlaceNode`` provides ROS I/O and visualization utilities.
``PickPlaceApplication`` provides planning utilities and the task sequence.
"""

from __future__ import annotations

from bisect import bisect_right
import copy
import math
import os
import sys
import threading
import time
from typing import Any, Optional

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from fanuc_msgs.msg import BoolIO, IOCmd, IOState, IOType
from geometry_msgs.msg import Point, Pose, PoseStamped, TransformStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.core.robot_state import RobotState
from moveit.core.robot_trajectory import RobotTrajectory
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
    PlanRequestParameters,
)
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    DisplayTrajectory,
    ObjectColor,
    OrientationConstraint,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)


class ToggleableCallbackGroup(CallbackGroup):
    """Expose callbacks to an executor only while explicitly enabled."""

    def __init__(self) -> None:
        super().__init__()
        # The lock protects both the enabled flag and mutually-exclusive slot.
        self._enabled = False
        self._active_entity = None
        self._lock = threading.Lock()

    def set_enabled(self, enabled: bool) -> None:
        """Enable or disable this group's entities in the executor wait set."""
        with self._lock:
            # The owning node calls executor.wake() after changing this flag.
            self._enabled = enabled

    def can_execute(self, entity) -> bool:
        """Return whether the entity may enter the executor wait set."""
        with self._lock:
            return self._enabled and self._active_entity is None

    def beginning_execution(self, entity) -> bool:
        """Reserve this group for one callback when it is enabled."""
        with self._lock:
            if self._enabled and self._active_entity is None:
                self._active_entity = entity
                return True
        return False

    def ending_execution(self, entity) -> None:
        """Release the entity after its callback finishes."""
        with self._lock:
            if self._active_entity is entity:
                self._active_entity = None


class PickPlaceNode(Node):
    """Provide robot I/O, Planning Scene, and RViz visualization utilities."""

    def __init__(self) -> None:
        super().__init__("fanuc_pick_and_place_example")

        # Include parameter and ROS-entity setup in startup CPU accounting.
        self._status_text = "STARTING"
        self._cpu_total_wall_start = time.perf_counter()
        self._cpu_total_process_start = time.process_time()
        self._cpu_state_wall_start = self._cpu_total_wall_start
        self._cpu_state_process_start = self._cpu_total_process_start

        # Lifecycle durations are seconds.
        # max_cycles=0 means repeat until shutdown; each positive count means that many A -> B -> A cycles.
        self.use_mock = self._parameter("use_mock", True)
        self.run_demo = self._parameter("run_demo", True)
        self.keep_alive = self._parameter("keep_alive_after_demo", False)
        self.max_cycles = int(self._parameter("max_cycles", 0))
        self.startup_delay = float(self._parameter("startup_delay_sec", 3.0))
        self.startup_timeout = float(self._parameter("startup_timeout_sec", 30.0))
        self.scene_update_delay = float(self._parameter("scene_update_delay_sec", 0.10))

        # These strings identify the MoveIt group, reference frame, TCP parent link, trajectory controller, and its ordered revolute joints.
        self.group_name = self._parameter("group_name", "manipulator")
        self.world_frame = self._parameter("world_frame", "world")
        self.tip_link = self._parameter("tip_link", "flange")
        self.trajectory_controller_name = self._parameter(
            "trajectory_controller_name", "joint_trajectory_controller"
        )
        self.joint_names = list(
            self._parameter("joint_names", ["J1", "J2", "J3", "J4", "J5", "J6"])
        )
        self.home_joint_positions = list(
            self._parameter(
                "home_joint_positions", [0.0, 0.0, 0.0, 0.0, -math.pi / 2.0, 0.0]
            )
        )
        # MoveIt RobotState joint positions use radians for these revolute joints.

        # HOME/approach motions use one OMPL profile; transfers submit all configured profiles through one multi-pipeline planning request.
        self.ompl_profile = self._parameter("ompl_profile", "ompl_rrtc")
        self.ompl_unconstrained_parallel_profiles = list(
            self._parameter(
                "motion.ompl.unconstrained.parallel_profiles",
                ["ompl_bkpiece", "ompl_bkpiece_coarse", "ompl_bkpiece_fine"],
            )
        )
        self.ompl_constrained_parallel_profiles = list(
            self._parameter(
                "motion.ompl.constrained.parallel_profiles",
                [
                    "ompl_bkpiece",
                    "ompl_bkpiece_coarse",
                    "ompl_bkpiece_fine",
                ],
            )
        )
        self.linear_profile = self._parameter("linear_profile", "pilz_lin")

        # IK timeout is seconds per seed.
        # Candidate separation and orientation tolerances are radians in joint/configuration space.
        self.ik_timeout = float(self._parameter("ik_timeout_sec", 0.10))
        self.ik_collision_free_attempts = int(
            self._parameter("ik.collision_free_attempts", 24)
        )
        self.ik_transfer_goal_candidates = int(
            self._parameter("ik.transfer_goal_candidates", 4)
        )
        self.ik_constrained_collision_free_attempts = int(
            self._parameter("ik.constrained.collision_free_attempts", 12)
        )
        self.ik_constrained_transfer_goal_candidates = int(
            self._parameter("ik.constrained.transfer_goal_candidates", 1)
        )
        self.ik_candidate_separation = float(
            self._parameter("ik.candidate_separation_rad", 0.05)
        )
        self.orientation_tolerance = float(
            self._parameter("orientation_tolerance_rad", 0.15)
        )
        self.ompl_goal_joint_tolerance = float(
            self._parameter(
                "motion.ompl.goal_joint_tolerance_rad",
                0.001,
            )
        )

        # This flag selects a complete-transfer OrientationConstraint or an unconstrained path with only the goal pose orientation.
        self.ompl_preserve_transfer_orientation = bool(
            self._parameter("motion.ompl.preserve_orientation_during_transfer", True)
        )
        self.ompl_joint_travel_weights = list(
            self._parameter(
                "motion.ompl.joint_travel_weights",
                [1.0] * len(self.joint_names),
            )
        )

        # Priority-joint acceptance, hard limits, step limits, and cumulative travel limits are radians.
        # Travel weights are positive unitless factors.
        self.ompl_priority_joint_names = list(
            self._parameter("motion.ompl.priority_joint_names", ["J4"])
        )
        self.ompl_priority_joint_early_acceptance = float(
            self._parameter(
                "motion.ompl.priority_joint_early_acceptance_rad",
                math.pi / 2.0,
            )
        )
        self.ompl_priority_joint_max_travel = float(
            self._parameter(
                "motion.ompl.priority_joint_max_travel_rad",
                2.0 * math.pi / 3.0,
            )
        )
        self.ompl_priority_joint_additional_retries = int(
            self._parameter("motion.ompl.priority_joint_additional_retries", 0)
        )
        self.max_joint_travel = float(self._parameter("max_joint_travel_rad", math.pi))
        self.max_trajectory_joint_step = float(
            self._parameter("max_trajectory_joint_step_rad", 1.0)
        )
        self.max_trajectory_joint_travel = float(
            self._parameter("max_trajectory_joint_travel_rad", math.tau)
        )

        # 0 retries OMPL indefinitely; positive values limit retry requests.
        # Retry delays and planning budgets are seconds.
        self.ompl_max_retries = int(self._parameter("ompl.max_retries", 0))
        self.ompl_retry_delay = float(self._parameter("ompl.retry_delay_sec", 0.50))
        # Velocity and acceleration scaling factors are unitless values.
        self.ompl_unconstrained_planning_time = float(
            self._parameter(
                "motion.ompl.unconstrained.planning_time_sec",
                2.0,
            )
        )
        self.ompl_constrained_planning_time = float(
            self._parameter(
                "motion.ompl.constrained.planning_time_sec",
                2.0,
            )
        )
        self.ompl_planning_attempts = int(
            self._parameter("motion.ompl.planning_attempts", 1)
        )
        self.ompl_velocity_scaling = float(
            self._parameter("motion.ompl.max_velocity_scaling_factor", 0.50)
        )
        self.ompl_acceleration_scaling = float(
            self._parameter("motion.ompl.max_acceleration_scaling_factor", 0.50)
        )

        # Pilz LIN uses its own seconds budget and unitless motion scaling.
        self.linear_planning_time = float(
            self._parameter("motion.linear.planning_time_sec", 5.0)
        )
        self.linear_planning_attempts = int(
            self._parameter("motion.linear.planning_attempts", 1)
        )
        self.linear_velocity_scaling = float(
            self._parameter("motion.linear.max_velocity_scaling_factor", 0.20)
        )
        self.linear_acceleration_scaling = float(
            self._parameter("motion.linear.max_acceleration_scaling_factor", 0.20)
        )

        # Workspace min/max are [x, y, z] in metres in world_frame.
        self.workspace_min = list(
            self._parameter("planning.workspace_min", [-2.0, -2.0, -1.0])
        )
        self.workspace_max = list(
            self._parameter("planning.workspace_max", [2.0, 2.0, 2.0])
        )

        # A and B positions are [x, y, z] metres in world_frame.
        # make_pose() combines each position with the orientation captured at HOME.
        self.pick_approach = list(
            self._parameter("pick.approach_position", [0.65, -0.40, 0.30])
        )
        self.pick_position = list(self._parameter("pick.position", [0.65, -0.40, 0.15]))
        self.place_approach = list(
            self._parameter("place.approach_position", [0.65, 0.40, 0.30])
        )
        self.place_position = list(
            self._parameter("place.position", [0.65, 0.40, 0.15])
        )

        # Work size, world positions, and grasp offset are metres.
        # Size order is [x, y, z]; the grasp offset is expressed in tip_link coordinates.
        self.work_id = self._parameter("work.id", "virtual_work")
        self.work_size = list(self._parameter("work.size", [0.05, 0.05, 0.05]))
        self.work_pick_position = list(
            self._parameter("work.pick_position", [0.65, -0.40, 0.025])
        )
        self.work_place_position = list(
            self._parameter("work.place_position", [0.65, 0.40, 0.025])
        )
        self.work_grasp_offset = list(
            self._parameter("work.grasp_offset_in_tip", [0.125, 0.0, 0.0])
        )

        # Obstacle size and position are [x, y, z] metres in world_frame.
        self.obstacle_id = self._parameter("obstacle.id", "movable_obstacle")
        self.obstacle_size = list(self._parameter("obstacle.size", [0.70, 0.10, 0.30]))
        self.obstacle_pose = self._pose_from_position(
            list(self._parameter("obstacle.position", [0.65, 0.0, 0.15]))
        )
        # Planning Scene RGBA values are unitless and each channel is 0..1.
        self._collision_object_rgba = {
            self.work_id: (1.0, 0.65, 0.05, 1.0),
            self.obstacle_id: (0.15, 0.45, 1.0, 1.0),
        }
        # Floor and ceiling boxes use [x, y, z] sizes/positions in metres.
        self.floor_enabled = bool(self._parameter("floor.enabled", True))
        self.floor_id = self._parameter("floor.id", "floor")
        self.floor_size = list(self._parameter("floor.size", [1.4, 1.5, 0.1]))
        self.floor_pose = self._pose_from_position(
            list(self._parameter("floor.position", [0.4, 0.0, -0.051]))
        )
        self.ceiling_enabled = bool(self._parameter("ceiling.enabled", True))
        self.ceiling_id = self._parameter("ceiling.id", "ceiling")
        self.ceiling_size = list(self._parameter("ceiling.size", [1.4, 1.5, 0.1]))
        self.ceiling_pose = self._pose_from_position(
            list(self._parameter("ceiling.position", [0.4, 0.0, 1.15]))
        )

        # Rear/front guard boxes use [x, y, z] sizes/positions in metres.
        self.rear_guard_enabled = bool(self._parameter("rear_guard.enabled", True))
        self.rear_guard_id = self._parameter("rear_guard.id", "rear_guard")
        self.rear_guard_size = list(
            self._parameter("rear_guard.size", [0.10, 1.5, 1.1])
        )
        self.rear_guard_pose = self._pose_from_position(
            list(self._parameter("rear_guard.position", [-0.25, 0.0, 0.55]))
        )

        self.front_guard_enabled = bool(self._parameter("front_guard.enabled", True))
        self.front_guard_id = self._parameter("front_guard.id", "front_guard")
        self.front_guard_size = list(
            self._parameter("front_guard.size", [0.10, 1.5, 1.1])
        )
        self.front_guard_pose = self._pose_from_position(
            list(self._parameter("front_guard.position", [1.05, 0.0, 0.55]))
        )

        # Left/right guard boxes use [x, y, z] sizes/positions in metres.
        self.left_guard_enabled = bool(self._parameter("left_guard.enabled", True))
        self.left_guard_id = self._parameter("left_guard.id", "left_guard")
        self.left_guard_size = list(
            self._parameter("left_guard.size", [1.2, 0.10, 1.1])
        )
        self.left_guard_pose = self._pose_from_position(
            list(self._parameter("left_guard.position", [0.4, 0.7, 0.55]))
        )
        self.right_guard_enabled = bool(self._parameter("right_guard.enabled", True))
        self.right_guard_id = self._parameter("right_guard.id", "right_guard")
        self.right_guard_size = list(
            self._parameter("right_guard.size", [1.2, 0.10, 1.1])
        )
        self.right_guard_pose = self._pose_from_position(
            list(self._parameter("right_guard.position", [0.4, -0.7, 0.55]))
        )

        # Hand command/status identify FANUC BoolIO ports as uppercase
        # (type_name, uint16 index) pairs. Feedback delays/timeouts are seconds.
        self.hand_command_type = str(self._parameter("hand.command_type", "RO")).upper()
        self.hand_command_index = self._parameter("hand.command_index", 1)
        self.hand_status_type = str(self._parameter("hand.status_type", "RO")).upper()
        self.hand_status_index = self._parameter("hand.status_index", 1)
        self.hand_feedback_timeout = float(
            self._parameter("hand.feedback_timeout_sec", 5.0)
        )
        self.mock_feedback_delay = float(
            self._parameter("hand.mock_feedback_delay_sec", 0.20)
        )

        # Virtual-hand dimensions, clear gaps, and status-text xyz/scale are
        # metres in their marker frame.
        self.finger_length = float(self._parameter("hand.finger_length", 0.14))
        self.finger_width = float(self._parameter("hand.finger_width", 0.025))
        self.finger_height = float(self._parameter("hand.finger_height", 0.04))
        self.open_gap = float(self._parameter("hand.open_gap", 0.12))
        self.closed_gap = float(self._parameter("hand.closed_gap", 0.05))
        self.status_text_position = list(
            self._parameter("status_text.position", [0.10, 0.0, 1.20])
        )
        self.status_text_scale = float(self._parameter("status_text.scale", 0.035))

        # Marker line widths and point scale are metres; RGBA is unitless 0..1.
        # Publish period is seconds and joint matching tolerance is radians.
        self.tcp_path_line_width = float(self._parameter("tcp_path.line_width", 0.008))
        self.tcp_path_color = list(
            self._parameter("tcp_path.color_rgba", [1.0, 0.35, 0.05, 1.0])
        )
        self.tcp_progress_line_width = float(
            self._parameter("tcp_path.progress_line_width", 0.014)
        )
        self.tcp_progress_color = list(
            self._parameter("tcp_path.progress_color_rgba", [0.1, 1.0, 0.2, 1.0])
        )
        self.tcp_progress_point_scale = float(
            self._parameter("tcp_path.progress_point_scale", 0.035)
        )
        self.tcp_progress_publish_period = float(
            self._parameter("tcp_path.progress_publish_period_sec", 0.10)
        )
        self.tcp_progress_joint_tolerance = float(
            self._parameter("tcp_path.progress_joint_tolerance_rad", 0.05)
        )

        # Completion checks use revolute-joint position radians, velocity rad/s,
        # stable duration seconds, and a maximum wait in seconds.
        self.execution_goal_tolerance = float(
            self._parameter("execution.goal_tolerance_rad", 0.005)
        )
        self.execution_stopped_velocity_tolerance = float(
            self._parameter(
                "execution.stopped_velocity_tolerance_rad_per_sec",
                0.02,
            )
        )
        self.execution_goal_stable_duration = float(
            self._parameter("execution.goal_stable_duration_sec", 0.20)
        )
        self.execution_goal_wait_timeout = float(
            self._parameter("execution.goal_wait_timeout_sec", 30.0)
        )
        self.tcp_path_tip_offset = list(
            self._parameter(
                "tcp_path.tip_offset_in_flange",
                [self.finger_length, 0.0, 0.0],
            )
        )
        # TCP offset is [x, y, z] metres in the flange coordinate frame.

        # Conditions let the application thread wait without polling callbacks.
        self._state_condition = threading.Condition()
        self._execution_goal_condition = threading.Condition()

        # Hand feedback state is updated only by the matching configured port.
        self._hand_closed = False
        self._hand_state_received = False

        # Goal-monitor state maps controller feedback joints to the final point.
        self._execution_goal_active = False
        self._execution_goal_reached = False
        self._execution_goal_joint_names: tuple[str, ...] = ()
        self._execution_goal_positions: list[float] = []
        self._execution_goal_controller_joint_names: tuple[str, ...] = ()
        self._execution_goal_joint_indices: list[int] = []
        self._execution_goal_previous_positions: Optional[list[float]] = None
        self._execution_goal_previous_sample_time: Optional[float] = None
        self._execution_goal_stable_since: Optional[float] = None
        self._execution_goal_position_error = math.inf
        self._execution_goal_velocity = math.inf

        # Visualization state is copied under one lock before publishing.
        self._visualization_lock = threading.Lock()
        self._tcp_path_points: list[Point] = []
        self._tcp_progress_points: list[Point] = []
        self._progress_joint_names: list[str] = []
        self._progress_controller_joint_names: tuple[str, ...] = ()
        self._progress_joint_indices: list[int] = []
        self._progress_joint_samples: list[list[float]] = []
        self._progress_times: list[float] = []
        self._progress_tcp_points: list[Point] = []
        self._progress_active = False
        self._last_progress_publish_time = 0.0

        # Obstacle edits and Planning Scene access have separate ownership.
        self._obstacle_lock = threading.Lock()
        self._planning_scene_monitor = None

        # IOCmd = {BoolIO[] values};
        # Each BoolIO contains {IOType io_type, uint16 index, bool value}.
        self.hand_command_publisher = self.create_publisher(
            IOCmd, "/fanuc_gpio_controller/io_cmd", 1
        )
        # IOState has the same BoolIO[] layout and provides hand feedback.
        self.hand_state_callback_group = ToggleableCallbackGroup()
        self.hand_state_subscription = self.create_subscription(
            IOState,
            "/fanuc_gpio_controller/io_state",
            self._hand_state_callback,
            1,
            callback_group=self.hand_state_callback_group,
        )
        # JointTrajectoryControllerState supplies ordered joint_names plus reference/feedback JointTrajectoryPoint fields in rad, rad/s, and time.
        self.trajectory_state_callback_group = ToggleableCallbackGroup()
        self.trajectory_state_subscription = self.create_subscription(
            JointTrajectoryControllerState,
            f"/{self.trajectory_controller_name}/controller_state",
            self._trajectory_state_callback,
            1,
            callback_group=self.trajectory_state_callback_group,
        )
        self.trajectory_progress_timer = self.create_timer(
            self.tcp_progress_publish_period,
            self._trajectory_progress_timer_callback,
        )
        # Start the display-rate timer disabled; execution enables samples.
        self.trajectory_progress_timer.cancel()

        # MarkerArray and DisplayTrajectory samples use transient-local QoS.
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Separate MarkerArray writers retain hand, path, progress, and status.
        self.marker_publishers = {
            component: self.create_publisher(
                MarkerArray, "/fanuc_pick_and_place/markers", marker_qos
            )
            for component in ("hand", "path", "progress", "status")
        }
        self.trajectory_publisher = self.create_publisher(
            DisplayTrajectory, "/display_planned_path", marker_qos
        )
        self.scene_publisher = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )
        self.planning_scene_service = None

        # RViz users can move the obstacle before a subsequent planning request.
        self.interactive_server = InteractiveMarkerServer(
            self, "fanuc_pick_and_place_obstacle"
        )
        self._create_obstacle_interactive_marker()
        self.publish_visual_markers()

    def _parameter(self, name: str, default: Any) -> Any:
        """Return a parameter, declaring it with the supplied value if needed."""
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    @staticmethod
    def _pose_from_position(position: list[float]) -> Pose:
        """Return an identity-orientation pose for a three-value position."""
        # geometry_msgs/Pose.position uses metres;
        # orientation is a unit quaternion and identity is (x=0, y=0, z=0, w=1).
        if len(position) != 3:
            raise ValueError("position must contain [x, y, z]")
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.w = 1.0
        return pose

    @staticmethod
    def _io_type(type_name: str) -> IOType:
        """Return an uppercase FANUC I/O type message."""
        # IOType contains one string field named type.
        io_type = IOType()
        io_type.type = type_name.upper()
        return io_type

    def set_status(self, text: str, *, publish: bool = True) -> None:
        """Publish one normalized task state to the terminal and RViz."""
        # Normalize state labels so terminal and RViz show the same identifier.
        next_status = "_".join(text.strip().upper().split())

        # Record wall/process seconds for the state interval that is ending.
        wall_now = time.perf_counter()
        process_now = time.process_time()
        self._log_cpu_metric(
            scope="state",
            state=self._status_text,
            next_state=next_status,
            wall_sec=wall_now - self._cpu_state_wall_start,
            cpu_sec=process_now - self._cpu_state_process_start,
        )
        # Start the next interval only after recording the previous one.
        self._cpu_state_wall_start = wall_now
        self._cpu_state_process_start = process_now
        self._status_text = next_status
        self.get_logger().info(f"[STATE] {self._status_text}")
        if publish:
            self._publish_visual_marker_components("status")

    def _log_cpu_metric(
        self,
        scope: str,
        state: str,
        wall_sec: float,
        cpu_sec: float,
        next_state: Optional[str] = None,
    ) -> None:
        """Log process CPU consumption without adding a sampling thread."""
        avg_cores = cpu_sec / wall_sec if wall_sec > 0.0 else 0.0
        transition = f" next={next_state}" if next_state is not None else ""
        self.get_logger().info(
            f"[CPU_METRIC] scope={scope} state={state}{transition} "
            f"wall_sec={wall_sec:.6f} cpu_sec={cpu_sec:.6f} "
            f"avg_cores={avg_cores:.3f} cpu_percent={avg_cores * 100.0:.1f}"
        )

    def log_total_cpu_metric(self) -> None:
        """Log whole-process CPU consumption accumulated since node startup."""
        self._log_cpu_metric(
            scope="total",
            state=self._status_text,
            wall_sec=time.perf_counter() - self._cpu_total_wall_start,
            cpu_sec=time.process_time() - self._cpu_total_process_start,
        )

    def configure_planning_scene_service(self, planning_scene_monitor: Any) -> None:
        """Make the current MoveIt Planning Scene available through the service."""
        # GetPlanningScene.Request contains a PlanningSceneComponents bit mask;
        # Response contains one PlanningScene message.
        self._planning_scene_monitor = planning_scene_monitor
        if self.planning_scene_service is None:
            self.planning_scene_service = self.create_service(
                GetPlanningScene,
                "/get_planning_scene",
                self._get_planning_scene_callback,
            )
        self.get_logger().info("Planning Scene service ready: /get_planning_scene")

    def wait_for_trajectory_controller(self) -> None:
        """Wait until the execution action server and DDS links are ready."""
        # FollowJointTrajectory.Goal carries a JointTrajectory;
        # this temporary ActionClient checks server discovery before HOME execution.
        action_name = f"/{self.trajectory_controller_name}/follow_joint_trajectory"
        action_client = ActionClient(self, FollowJointTrajectory, action_name)
        try:
            if not action_client.wait_for_server(timeout_sec=self.startup_timeout):
                raise TimeoutError(
                    f"trajectory action server was not ready: {action_name}"
                )
            self.get_logger().info(f"Trajectory action server ready: {action_name}")
            # MoveIt owns a separate rclcpp action client.
            # Server discovery by this probe can precede readiness of that client on a fast startup.
            time.sleep(2.0)
        finally:
            action_client.destroy()

    def _get_planning_scene_callback(
        self,
        request: GetPlanningScene.Request,
        response: GetPlanningScene.Response,
    ) -> GetPlanningScene.Response:
        """Return requested Planning Scene data from the current scene."""
        if self._planning_scene_monitor is None:
            self.get_logger().error("MoveItPy Planning Scene was not ready in time")
            return response
        with self._planning_scene_monitor.read_only() as scene:
            # Return only name/model/fixed-frame fields for SCENE_SETTINGS.
            if request.components.components == PlanningSceneComponents.SCENE_SETTINGS:
                response.scene.name = str(scene.name)
                response.scene.robot_model_name = str(scene.robot_model.name)
                root_transform = TransformStamped()
                root_transform.header.frame_id = str(scene.planning_frame)
                root_transform.child_frame_id = str(scene.planning_frame)
                root_transform.transform.rotation.w = 1.0
                response.scene.fixed_frame_transforms = [root_transform]
            else:
                # Return a detached complete PlanningScene for other masks.
                response.scene = copy.deepcopy(scene.planning_scene_message)
        return response

    def set_tcp_path(self, points: list[Point], *, publish: bool = True) -> None:
        """Replace the planned TCP path displayed in RViz."""
        # Each geometry_msgs/Point is an [x, y, z] position in world-frame metres.
        with self._visualization_lock:
            self._tcp_path_points = list(points)
        if publish:
            self._publish_visual_marker_components("path")

    @staticmethod
    def _duration_seconds(duration: Any) -> float:
        """Convert a ROS duration message to seconds."""
        return float(duration.sec) + float(duration.nanosec) * 1e-9

    @staticmethod
    def _interpolate_samples(
        times: list[float], samples: list[list[float]], elapsed: float
    ) -> tuple[int, float, list[float]]:
        """Return the segment, ratio, and value at a requested sample time."""
        if not times or len(times) != len(samples):
            raise ValueError("times and samples must have the same non-zero length")
        if elapsed <= times[0] or len(times) == 1:
            return 0, 0.0, list(samples[0])
        if elapsed >= times[-1]:
            return len(times) - 1, 0.0, list(samples[-1])

        # Locate the surrounding seconds samples, then linearly interpolate all joint values in radians with ratio in [0, 1].
        right = bisect_right(times, elapsed)
        left = right - 1
        duration = times[right] - times[left]
        ratio = 0.0 if duration <= 0.0 else (elapsed - times[left]) / duration
        sample = [
            start + ratio * (end - start)
            for start, end in zip(samples[left], samples[right])
        ]
        return left, ratio, sample

    @staticmethod
    def _interpolate_point(start: Point, end: Point, ratio: float) -> Point:
        """Interpolate between two world-coordinate points."""
        # Point coordinates remain in metres in the common world frame.
        point = Point()
        point.x = start.x + ratio * (end.x - start.x)
        point.y = start.y + ratio * (end.y - start.y)
        point.z = start.z + ratio * (end.z - start.z)
        return point

    def start_trajectory_progress(
        self,
        trajectory_message: Any,
        tcp_points: list[Point],
        *,
        publish: bool = True,
    ) -> None:
        """Start matching controller references to a planned TCP path."""
        # trajectory_message.joint_trajectory contains joint_names and points;
        # each point supplies positions [rad] and time_from_start.
        trajectory = trajectory_message.joint_trajectory
        # Display progress requires one TCP point for every joint waypoint.
        if not trajectory.points or len(trajectory.points) != len(tcp_points):
            raise ValueError("trajectory points and TCP points must have equal length")
        joint_names = list(trajectory.joint_names)
        joint_samples = [list(point.positions) for point in trajectory.points]
        if not joint_names or any(
            len(sample) != len(joint_names) for sample in joint_samples
        ):
            raise ValueError(
                "trajectory joint names and positions must have equal length"
            )
        # Convert ROS durations once before high-rate controller callbacks begin.
        times = [
            self._duration_seconds(point.time_from_start) for point in trajectory.points
        ]
        if any(end < start for start, end in zip(times, times[1:])):
            raise ValueError("trajectory point times must be nondecreasing")
        # Store copies used to map controller reference time to one interpolated TCP point in the planned path.
        with self._visualization_lock:
            if self._progress_active:
                raise RuntimeError("trajectory progress is already active")
            self._progress_joint_names = joint_names
            self._progress_controller_joint_names = ()
            self._progress_joint_indices = []
            self._progress_joint_samples = joint_samples
            self._progress_times = times
            self._progress_tcp_points = list(tcp_points)
            self._tcp_progress_points = [tcp_points[0]]
            self._progress_active = True
            self._last_progress_publish_time = 0.0
        # Use the final planned joint positions [rad] as the measured goal.
        with self._execution_goal_condition:
            self._execution_goal_active = True
            self._execution_goal_reached = False
            self._execution_goal_joint_names = tuple(joint_names)
            self._execution_goal_positions = list(joint_samples[-1])
            self._execution_goal_controller_joint_names = ()
            self._execution_goal_joint_indices = []
            self._execution_goal_previous_positions = None
            self._execution_goal_previous_sample_time = None
            self._execution_goal_stable_since = None
            self._execution_goal_position_error = math.inf
            self._execution_goal_velocity = math.inf
        # Request the first state sample immediately and later at marker rate.
        self.trajectory_progress_timer.reset()
        self._set_trajectory_state_callbacks_enabled(True)
        if publish:
            self._publish_visual_marker_components("progress")

    def _set_trajectory_state_callbacks_enabled(self, enabled: bool) -> None:
        """Toggle one controller-state sample and rebuild the executor wait set."""
        # Only enabled groups are admitted by ToggleableCallbackGroup.
        self.trajectory_state_callback_group.set_enabled(enabled)
        executor = self.executor
        if executor is not None:
            executor.wake()

    def _trajectory_progress_timer_callback(self) -> None:
        """Request one latest controller-state sample at the display rate."""
        with self._visualization_lock:
            if self._progress_active:
                self._set_trajectory_state_callbacks_enabled(True)

    def _set_hand_state_callbacks_enabled(self, enabled: bool) -> None:
        """Toggle hand feedback and rebuild the executor wait set."""
        self.hand_state_callback_group.set_enabled(enabled)
        executor = self.executor
        if executor is not None:
            executor.wake()

    def _update_execution_goal(
        self,
        message: JointTrajectoryControllerState,
        sample_time: float,
    ) -> None:
        """Update final-position and stopped-motion confirmation."""
        # message.feedback is the measured JointTrajectoryPoint.
        # positions are radians and velocities are rad/s for the configured revolute joints.
        with self._execution_goal_condition:
            if not self._execution_goal_active:
                return
            if len(message.feedback.positions) != len(message.joint_names):
                return
            controller_joint_names = tuple(message.joint_names)
            if controller_joint_names != self._execution_goal_controller_joint_names:
                # Resolve indices again only when the controller joint order changes.
                if len(set(controller_joint_names)) != len(controller_joint_names):
                    return
                try:
                    self._execution_goal_joint_indices = [
                        message.joint_names.index(name)
                        for name in self._execution_goal_joint_names
                    ]
                except ValueError:
                    return
                self._execution_goal_controller_joint_names = controller_joint_names
            feedback = [
                float(message.feedback.positions[index])
                for index in self._execution_goal_joint_indices
            ]
            if not all(math.isfinite(value) for value in feedback):
                return

            # Compute the largest absolute measured position error in radians.
            self._execution_goal_position_error = max(
                abs(value - goal)
                for value, goal in zip(
                    feedback,
                    self._execution_goal_positions,
                )
            )
            velocity = math.inf
            if len(message.feedback.velocities) == len(message.joint_names):
                # Prefer controller-reported velocities when all values are finite.
                velocities = [
                    abs(float(message.feedback.velocities[index]))
                    for index in self._execution_goal_joint_indices
                ]
                if all(math.isfinite(value) for value in velocities):
                    velocity = max(velocities)
            elif (
                self._execution_goal_previous_positions is not None
                and self._execution_goal_previous_sample_time is not None
                and sample_time > self._execution_goal_previous_sample_time
            ):
                # Estimate rad/s from consecutive positions when velocities are absent.
                elapsed = sample_time - self._execution_goal_previous_sample_time
                velocity = max(
                    abs(value - previous) / elapsed
                    for value, previous in zip(
                        feedback,
                        self._execution_goal_previous_positions,
                    )
                )
            self._execution_goal_previous_positions = feedback
            self._execution_goal_previous_sample_time = sample_time
            self._execution_goal_velocity = velocity

            # Both position and stopped-motion limits must remain true long enough.
            within_goal = (
                self._execution_goal_position_error <= self.execution_goal_tolerance
                and velocity <= self.execution_stopped_velocity_tolerance
            )
            if within_goal:
                if self._execution_goal_stable_since is None:
                    self._execution_goal_stable_since = sample_time
                self._execution_goal_reached = (
                    sample_time - self._execution_goal_stable_since
                    >= self.execution_goal_stable_duration
                )
            else:
                self._execution_goal_stable_since = None
                self._execution_goal_reached = False
            self._execution_goal_condition.notify_all()

    def _trajectory_state_callback(
        self, message: JointTrajectoryControllerState
    ) -> None:
        """Update execution confirmation and displayed TCP progress."""
        # Consume one requested controller sample, then disable this callback group until the timer or completion wait asks for another sample.
        self._set_trajectory_state_callbacks_enabled(False)
        publish = False
        now = time.monotonic()

        # Goal confirmation consumes feedback positions on every sampled callback.
        self._update_execution_goal(message, now)
        with self._visualization_lock:
            if (
                not self._progress_active
                or now - self._last_progress_publish_time
                < self.tcp_progress_publish_period
            ):
                return
            if len(message.reference.positions) != len(message.joint_names):
                return
            controller_joint_names = tuple(message.joint_names)
            if controller_joint_names != self._progress_controller_joint_names:
                # Map the configured plan order to the controller message order.
                try:
                    self._progress_joint_indices = [
                        message.joint_names.index(name)
                        for name in self._progress_joint_names
                    ]
                except ValueError:
                    return
                self._progress_controller_joint_names = controller_joint_names
            reference = [
                message.reference.positions[index]
                for index in self._progress_joint_indices
            ]
            # reference.time_from_start is converted to seconds before matching.
            elapsed = self._duration_seconds(message.reference.time_from_start)
            segment, ratio, expected = self._interpolate_samples(
                self._progress_times,
                self._progress_joint_samples,
                elapsed,
            )
            if (
                max(abs(actual - target) for actual, target in zip(reference, expected))
                > self.tcp_progress_joint_tolerance
            ):
                # Ignore a sample that does not correspond to the planned path.
                return
            if segment >= len(self._progress_tcp_points) - 1:
                progress = list(self._progress_tcp_points)
            else:
                progress = list(self._progress_tcp_points[: segment + 1])
                if ratio > 0.0:
                    progress.append(
                        self._interpolate_point(
                            self._progress_tcp_points[segment],
                            self._progress_tcp_points[segment + 1],
                            ratio,
                        )
                    )
            self._tcp_progress_points = progress
            self._last_progress_publish_time = now
            publish = True
        if publish:
            self._publish_visual_marker_components("progress")

    def wait_for_execution_goal(self, description: str) -> None:
        """Wait for measured joints to remain stopped at the final point."""
        # A successful action result enters this wait.
        # Completion requires both configured position [rad] and velocity [rad/s] limits for stable seconds.
        wait_start = time.monotonic()
        deadline = wait_start + self.execution_goal_wait_timeout
        with self._execution_goal_condition:
            if not self._execution_goal_active:
                raise RuntimeError("execution goal monitoring is not active")
            # Confirm stability using samples received after action success.
            self._execution_goal_reached = False
            self._execution_goal_previous_positions = None
            self._execution_goal_previous_sample_time = None
            self._execution_goal_stable_since = None
            self._execution_goal_position_error = math.inf
            self._execution_goal_velocity = math.inf
        self._set_trajectory_state_callbacks_enabled(True)

        # Sleep on callback notifications until stable completion or timeout.
        with self._execution_goal_condition:
            while not self._execution_goal_reached and rclpy.ok():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(
                        f"execution goal was not reached: {description}; "
                        f"max_position_error_rad="
                        f"{self._execution_goal_position_error:.6f}; "
                        f"max_velocity_rad_per_sec="
                        f"{self._execution_goal_velocity:.6f}"
                    )
                self._execution_goal_condition.wait(timeout=remaining)
            if not rclpy.ok():
                raise KeyboardInterrupt
            position_error = self._execution_goal_position_error
            velocity = self._execution_goal_velocity
        self.get_logger().info(
            f"[EXEC_GOAL] description={description} reached=true "
            f"wait_sec={time.monotonic() - wait_start:.6f} "
            f"max_position_error_rad={position_error:.6f} "
            f"max_velocity_rad_per_sec={velocity:.6f}"
        )

    def finish_trajectory_progress(self, completed: bool) -> None:
        """Stop progress tracking and show the full path when execution succeeded."""
        self.trajectory_progress_timer.cancel()
        # Preserve the full completed path only after successful execution.
        with self._visualization_lock:
            if completed:
                self._tcp_progress_points = list(self._progress_tcp_points)
            self._progress_active = False
        with self._execution_goal_condition:
            self._execution_goal_active = False
            self._execution_goal_reached = False
            self._execution_goal_condition.notify_all()
        self._set_trajectory_state_callbacks_enabled(False)
        self._publish_visual_marker_components("progress")

    def _hand_state_callback(self, message: IOState) -> None:
        """Update the hand state from physical controller I/O feedback."""
        # IOState.values[] contains BoolIO{io_type, index, value}.
        # Use only the configured status port and leave unrelated cyclic I/O entries untouched.
        for item in message.values:
            if (
                item.io_type.type == self.hand_status_type
                and item.index == self.hand_status_index
            ):
                with self._state_condition:
                    changed = self._hand_closed != item.value
                    self._hand_closed = item.value
                    self._hand_state_received = True
                    self._state_condition.notify_all()
                if changed:
                    self._publish_visual_marker_components("hand")
                return

    def command_hand(self, close: bool) -> None:
        """Command the hand and wait for the configured controller I/O state."""
        # close becomes BoolIO.value.
        # True/False meaning follows the configured cell wiring; this example maps True to the closed virtual-hand state.
        # Do not lose the one-shot command before the GPIO controller subscribes.
        deadline = time.monotonic() + self.startup_timeout
        while rclpy.ok() and self.hand_command_publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    "fanuc_gpio_controller did not subscribe to hand command"
                )
            time.sleep(0.05)

        with self._state_condition:
            self._hand_state_received = False
        if not self.use_mock:
            # Physical feedback callbacks are enabled only for this command wait.
            self._set_hand_state_callbacks_enabled(True)
        try:
            # Publish IOCmd{values=[BoolIO{io_type, index, bool value}]} once.
            value = BoolIO()
            value.io_type = self._io_type(self.hand_command_type)
            value.index = self.hand_command_index
            value.value = close
            message = IOCmd()
            message.values = [value]
            self.hand_command_publisher.publish(message)
            self.get_logger().info(
                "GPIO command: "
                f"{self.hand_command_type}[{self.hand_command_index}]={close}"
            )

            if self.use_mock:
                # Set the same local hand state after mock_feedback_delay seconds.
                time.sleep(self.mock_feedback_delay)
                with self._state_condition:
                    self._hand_closed = close
                    self._hand_state_received = True
                    self._state_condition.notify_all()

            # Wait for the matching state value before advancing the task sequence.
            feedback_deadline = time.monotonic() + self.hand_feedback_timeout
            with self._state_condition:
                while (
                    not self._hand_state_received or self._hand_closed != close
                ) and rclpy.ok():
                    remaining = feedback_deadline - time.monotonic()
                    if remaining <= 0.0:
                        raise TimeoutError(
                            "hand I/O feedback "
                            f"{self.hand_status_type}[{self.hand_status_index}] "
                            f"did not become {close}"
                        )
                    self._state_condition.wait(timeout=remaining)
            self.get_logger().info(
                "GPIO feedback: "
                f"{self.hand_status_type}[{self.hand_status_index}]={close}"
            )
            self._publish_visual_marker_components("hand")
        finally:
            self._set_hand_state_callbacks_enabled(False)

    def publish_planned_trajectory(
        self, start_positions: list[float], trajectory_message: Any
    ) -> None:
        """Publish the selected trajectory and its start state for RViz."""
        display = DisplayTrajectory()
        # DisplayTrajectory contains trajectory_start plus a trajectory[] array.
        # JointState.position and JointTrajectoryPoint.positions are radians.
        display.trajectory_start.joint_state.header.stamp = (
            self.get_clock().now().to_msg()
        )
        display.trajectory_start.joint_state.name = list(self.joint_names)
        display.trajectory_start.joint_state.position = [
            float(value) for value in start_positions
        ]
        display.trajectory_start.is_diff = True
        display.trajectory = [trajectory_message]
        self.trajectory_publisher.publish(display)

    @staticmethod
    def _make_box(
        object_id: str,
        frame_id: str,
        size: list[float],
        pose: Pose,
        operation: int,
    ) -> CollisionObject:
        """Return a validated box-shaped Planning Scene object."""
        # size is [x, y, z] metres and pose is expressed in frame_id.
        if len(size) != 3 or any(float(value) <= 0.0 for value in size):
            raise ValueError("box size must contain three positive dimensions")
        collision_object = CollisionObject()
        # CollisionObject pairs SolidPrimitive.BOX dimensions with one Pose.
        collision_object.header.frame_id = frame_id
        collision_object.id = object_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in size]
        collision_object.primitives = [primitive]
        collision_object.primitive_poses = [copy.deepcopy(pose)]
        collision_object.operation = operation
        return collision_object

    @staticmethod
    def _make_object_color(
        object_id: str,
        rgba: tuple[float, float, float, float],
    ) -> ObjectColor:
        """Return a Planning Scene color for one object ID."""
        # rgba channels are unitless values in the inclusive range 0..1.
        object_color = ObjectColor()
        # MoveIt associates colors with collision objects by matching IDs.
        object_color.id = object_id
        object_color.color.r = rgba[0]
        object_color.color.g = rgba[1]
        object_color.color.b = rgba[2]
        object_color.color.a = rgba[3]
        return object_color

    def apply_planning_scene(
        self,
        world_objects: Optional[list[CollisionObject]] = None,
        attached_objects: Optional[list[AttachedCollisionObject]] = None,
        wait: bool = True,
    ) -> None:
        """Publish world and attached-object changes to the Planning Scene."""
        scene = PlanningScene()
        # PlanningScene.is_diff updates only the supplied world objects, attached objects, and color records.
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = world_objects or []
        scene.robot_state.attached_collision_objects = attached_objects or []
        # Reapply fixed colors only for colored objects in this diff.
        changed_ids = {item.id for item in scene.world.collision_objects}
        changed_ids.update(
            item.object.id for item in scene.robot_state.attached_collision_objects
        )
        scene.object_colors = [
            self._make_object_color(
                object_id,
                self._collision_object_rgba[object_id],
            )
            for object_id in sorted(changed_ids)
            if object_id in self._collision_object_rgba
        ]
        self.scene_publisher.publish(scene)
        if wait:
            # Give the local Planning Scene monitor time to consume the diff.
            time.sleep(self.scene_update_delay)

    def add_initial_scene(self) -> None:
        """Add the work, obstacle, and enabled boundary objects."""
        with self._obstacle_lock:
            # Copy the pose so later RViz feedback cannot mutate this message.
            obstacle_pose = copy.deepcopy(self.obstacle_pose)
        obstacle = self._make_box(
            self.obstacle_id,
            self.world_frame,
            self.obstacle_size,
            obstacle_pose,
            CollisionObject.ADD,
        )
        work = self._make_box(
            self.work_id,
            self.world_frame,
            self.work_size,
            self._pose_from_position(self.work_pick_position),
            CollisionObject.ADD,
        )
        # Add work and obstacle, then append every enabled boundary box.
        world_objects = [obstacle, work]
        if self.floor_enabled:
            world_objects.append(
                self._make_box(
                    self.floor_id,
                    self.world_frame,
                    self.floor_size,
                    copy.deepcopy(self.floor_pose),
                    CollisionObject.ADD,
                )
            )
        if self.ceiling_enabled:
            world_objects.append(
                self._make_box(
                    self.ceiling_id,
                    self.world_frame,
                    self.ceiling_size,
                    copy.deepcopy(self.ceiling_pose),
                    CollisionObject.ADD,
                )
            )
        if self.rear_guard_enabled:
            world_objects.append(
                self._make_box(
                    self.rear_guard_id,
                    self.world_frame,
                    self.rear_guard_size,
                    copy.deepcopy(self.rear_guard_pose),
                    CollisionObject.ADD,
                )
            )
        if self.front_guard_enabled:
            world_objects.append(
                self._make_box(
                    self.front_guard_id,
                    self.world_frame,
                    self.front_guard_size,
                    copy.deepcopy(self.front_guard_pose),
                    CollisionObject.ADD,
                )
            )
        if self.left_guard_enabled:
            world_objects.append(
                self._make_box(
                    self.left_guard_id,
                    self.world_frame,
                    self.left_guard_size,
                    copy.deepcopy(self.left_guard_pose),
                    CollisionObject.ADD,
                )
            )
        if self.right_guard_enabled:
            world_objects.append(
                self._make_box(
                    self.right_guard_id,
                    self.world_frame,
                    self.right_guard_size,
                    copy.deepcopy(self.right_guard_pose),
                    CollisionObject.ADD,
                )
            )
        self.apply_planning_scene(world_objects)

    def attach_work(self) -> None:
        """Attach the work object to the configured robot link."""
        # AttachedCollisionObject uses tip_link and a local pose in metres.
        local_pose = self._pose_from_position(self.work_grasp_offset)
        attached = AttachedCollisionObject()
        attached.link_name = self.tip_link
        attached.touch_links = [self.tip_link]
        attached.object = self._make_box(
            self.work_id,
            self.tip_link,
            self.work_size,
            local_pose,
            CollisionObject.ADD,
        )
        self.apply_planning_scene(attached_objects=[attached])

    def detach_work_at(self, world_position: list[float]) -> None:
        """Detach the work and return it to a world-frame position."""
        # REMOVE clears the attached object with the same ID.
        detach = AttachedCollisionObject()
        detach.link_name = self.tip_link
        detach.object.id = self.work_id
        detach.object.operation = CollisionObject.REMOVE
        placed_work = self._make_box(
            self.work_id,
            self.world_frame,
            self.work_size,
            self._pose_from_position(world_position),
            CollisionObject.ADD,
        )
        # Publish attachment removal and the new world pose in one scene diff.
        self.apply_planning_scene(
            world_objects=[placed_work],
            attached_objects=[detach],
        )

    def _create_obstacle_interactive_marker(self) -> None:
        """Create six-axis RViz controls for moving the obstacle."""
        # InteractiveMarker pose and scale use world-frame metres.
        interactive = InteractiveMarker()
        interactive.header.frame_id = self.world_frame
        interactive.name = self.obstacle_id
        interactive.description = "Drag to update Planning Scene obstacle"
        interactive.pose = copy.deepcopy(self.obstacle_pose)
        interactive.scale = max(self.obstacle_size) * 1.4

        visible = InteractiveMarkerControl()
        visible.always_visible = True
        box = Marker()
        box.type = Marker.CUBE
        box.scale.x, box.scale.y, box.scale.z = self.obstacle_size
        obstacle_rgba = self._collision_object_rgba[self.obstacle_id]
        box.color.r = obstacle_rgba[0]
        box.color.g = obstacle_rgba[1]
        box.color.b = obstacle_rgba[2]
        box.color.a = obstacle_rgba[3]
        box.pose.orientation.w = 1.0
        visible.markers.append(box)
        interactive.controls.append(visible)

        # Add one ROTATE_AXIS and one MOVE_AXIS control for each local axis.
        axes = [("x", 1.0, 0.0, 0.0), ("y", 0.0, 0.0, 1.0), ("z", 0.0, 1.0, 0.0)]
        for name, x, y, z in axes:
            for mode, prefix in (
                (InteractiveMarkerControl.ROTATE_AXIS, "rotate"),
                (InteractiveMarkerControl.MOVE_AXIS, "move"),
            ):
                control = InteractiveMarkerControl()
                control.name = f"{prefix}_{name}"
                control.orientation.w = 1.0
                control.orientation.x = x
                control.orientation.y = y
                control.orientation.z = z
                control.interaction_mode = mode
                interactive.controls.append(control)

        self.interactive_server.insert(
            interactive, feedback_callback=self._obstacle_feedback
        )
        self.interactive_server.applyChanges()

    def _obstacle_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        """Apply an RViz obstacle pose to the Planning Scene after dragging."""
        # feedback.pose is a geometry_msgs/Pose in the interactive marker frame.
        if feedback.event_type not in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            return
        # POSE_UPDATE keeps the stored marker pose current while dragging.
        with self._obstacle_lock:
            self.obstacle_pose = copy.deepcopy(feedback.pose)
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            # Commit one Planning Scene update when the user releases the marker.
            obstacle = self._make_box(
                self.obstacle_id,
                self.world_frame,
                self.obstacle_size,
                feedback.pose,
                CollisionObject.ADD,
            )
            self.apply_planning_scene([obstacle], wait=False)
            self.get_logger().info("RViz obstacle pose applied to Planning Scene")

    def _make_hand_markers(self) -> list[Marker]:
        """Return the three virtual-hand markers for the current I/O state."""
        markers = []
        # Marker pose/scale fields are metres in tip_link.
        # The BoolIO state selects open_gap or closed_gap as the clear inner-face distance.
        gap = self.closed_gap if self._hand_closed else self.open_gap
        for marker_id, direction in ((1, 1.0), (2, -1.0)):
            finger = Marker()
            finger.header.frame_id = self.tip_link
            finger.frame_locked = True
            finger.ns = "virtual_hand"
            finger.id = marker_id
            finger.type = Marker.CUBE
            finger.action = Marker.ADD
            finger.pose.position.x = self.finger_length / 2.0
            # open_gap/closed_gap represent the clear distance between inner faces.
            finger.pose.position.y = direction * (gap + self.finger_width) / 2.0
            finger.pose.orientation.w = 1.0
            finger.scale.x = self.finger_length
            finger.scale.y = self.finger_width
            finger.scale.z = self.finger_height
            finger.color.r = 0.0
            finger.color.g = 0.0
            finger.color.b = 0.0
            finger.color.a = 1.0
            markers.append(finger)

        # The palm remains fixed while the two finger markers move laterally.
        palm = Marker()
        palm.header.frame_id = self.tip_link
        palm.frame_locked = True
        palm.ns = "virtual_hand"
        palm.id = 3
        palm.type = Marker.CUBE
        palm.action = Marker.ADD
        palm.pose.position.x = 0.015
        palm.pose.orientation.w = 1.0
        palm.scale.x = 0.03
        palm.scale.y = self.open_gap + 2.0 * self.finger_width
        palm.scale.z = self.finger_height * 1.4
        palm.color.r = 0.0
        palm.color.g = 0.0
        palm.color.b = 0.0
        palm.color.a = 1.0
        markers.append(palm)
        return markers

    def _make_tcp_path_marker(self) -> Marker:
        """Return the complete planned TCP-path marker."""
        # Copy shared points before constructing a ROS marker outside the lock.
        with self._visualization_lock:
            tcp_path_points = list(self._tcp_path_points)
        tcp_path = Marker()
        tcp_path.header.frame_id = self.world_frame
        tcp_path.frame_locked = True
        tcp_path.ns = "planned_tcp_path"
        tcp_path.id = 20
        tcp_path.type = Marker.LINE_STRIP
        tcp_path.action = Marker.ADD
        tcp_path.pose.orientation.w = 1.0
        tcp_path.scale.x = self.tcp_path_line_width
        # LINE_STRIP scale.x is the line width in metres.
        (
            tcp_path.color.r,
            tcp_path.color.g,
            tcp_path.color.b,
            tcp_path.color.a,
        ) = self.tcp_path_color
        tcp_path.points = tcp_path_points
        return tcp_path

    def _make_tcp_progress_markers(self) -> list[Marker]:
        """Return the traversed TCP line and its current-point marker."""
        # The progress line and sphere use the same latest point list.
        with self._visualization_lock:
            tcp_progress_points = list(self._tcp_progress_points)
        tcp_progress = Marker()
        tcp_progress.header.frame_id = self.world_frame
        tcp_progress.frame_locked = True
        tcp_progress.ns = "tcp_path_progress"
        tcp_progress.id = 21
        tcp_progress.type = Marker.LINE_STRIP
        tcp_progress.action = Marker.ADD
        tcp_progress.pose.orientation.w = 1.0
        tcp_progress.scale.x = self.tcp_progress_line_width
        (
            tcp_progress.color.r,
            tcp_progress.color.g,
            tcp_progress.color.b,
            tcp_progress.color.a,
        ) = self.tcp_progress_color
        tcp_progress.points = tcp_progress_points

        # Use DELETE for the current-point sphere when the point list is empty.
        progress_point = Marker()
        progress_point.header.frame_id = self.world_frame
        progress_point.frame_locked = True
        progress_point.ns = "tcp_path_progress"
        progress_point.id = 22
        progress_point.type = Marker.SPHERE
        progress_point.action = Marker.ADD if tcp_progress_points else Marker.DELETE
        progress_point.pose.orientation.w = 1.0
        if tcp_progress_points:
            progress_point.pose.position = tcp_progress_points[-1]
        progress_point.scale.x = self.tcp_progress_point_scale
        progress_point.scale.y = self.tcp_progress_point_scale
        progress_point.scale.z = self.tcp_progress_point_scale
        (
            progress_point.color.r,
            progress_point.color.g,
            progress_point.color.b,
            progress_point.color.a,
        ) = self.tcp_progress_color
        return [tcp_progress, progress_point]

    def _make_status_marker(self) -> Marker:
        """Return the current task-state text marker."""
        # TEXT_VIEW_FACING position and scale.z are world-frame metres.
        status = Marker()
        status.header.frame_id = self.world_frame
        status.ns = "task_status"
        status.id = 10
        status.type = Marker.TEXT_VIEW_FACING
        status.action = Marker.ADD
        (
            status.pose.position.x,
            status.pose.position.y,
            status.pose.position.z,
        ) = self.status_text_position
        status.pose.orientation.w = 1.0
        status.scale.z = self.status_text_scale
        status.color.r = 1.0
        status.color.g = 1.0
        status.color.b = 0.2
        status.color.a = 1.0
        status.text = self._status_text
        return status

    def _publish_visual_marker_components(self, *components: str) -> None:
        """Publish only marker components whose state changed."""
        # components accepts any subset of hand, path, progress, and status.
        for component in components:
            # Each component has its own transient-local publisher and sample.
            markers = MarkerArray()
            if component == "hand":
                markers.markers.extend(self._make_hand_markers())
            elif component == "path":
                markers.markers.append(self._make_tcp_path_marker())
            elif component == "progress":
                markers.markers.extend(self._make_tcp_progress_markers())
            elif component == "status":
                markers.markers.append(self._make_status_marker())
            else:
                raise ValueError(f"unknown marker component: {component}")
            self.marker_publishers[component].publish(markers)

    def publish_visual_markers(self) -> None:
        """Publish the hand, TCP paths, progress point, and task state."""
        self._publish_visual_marker_components(
            "hand",
            "path",
            "progress",
            "status",
        )

    def shutdown_visualization(self) -> None:
        """Stop accepting Interactive Marker input."""
        # Disable callback sources before destroying their subscriptions.
        self.trajectory_progress_timer.cancel()
        self._set_hand_state_callbacks_enabled(False)
        self._set_trajectory_state_callbacks_enabled(False)
        for attribute in (
            "hand_state_subscription",
            "trajectory_state_subscription",
        ):
            subscription = getattr(self, attribute)
            setattr(self, attribute, None)
            if subscription is not None:
                self.destroy_subscription(subscription)
        self.interactive_server.shutdown()


class PickPlaceApplication:
    """Provide reusable planning operations and the Pick & Place sequence."""

    def __init__(self, node: PickPlaceNode, robot: MoveItPy) -> None:
        self.node = node
        self.robot = robot

        # Reuse one PlanningComponent, PlanningSceneMonitor, RobotModel, and JointModelGroup for all setup motions and cycles.
        self.planning_component = robot.get_planning_component(node.group_name)
        self.planning_scene_monitor = robot.get_planning_scene_monitor()
        self.robot_model = robot.get_robot_model()
        self.joint_model_group = self.robot_model.get_joint_model_group(node.group_name)
        # PlanRequestParameters holds one named profile.
        # MultiPipeline parameters hold one request object for each parallel OMPL profile.
        self.ompl_parameters = PlanRequestParameters(robot, node.ompl_profile)
        self.unconstrained_ompl_parameters = MultiPipelinePlanRequestParameters(
            robot, node.ompl_unconstrained_parallel_profiles
        )
        self.constrained_ompl_parameters = MultiPipelinePlanRequestParameters(
            robot, node.ompl_constrained_parallel_profiles
        )
        self.linear_parameters = PlanRequestParameters(robot, node.linear_profile)
        # Copy seconds budgets, attempt counts, and unitless scaling factors from application parameters into the reusable request objects.
        self.configure_plan_request_parameters(
            self.ompl_parameters,
            node.ompl_unconstrained_planning_time,
            node.ompl_planning_attempts,
            node.ompl_velocity_scaling,
            node.ompl_acceleration_scaling,
            "OMPL",
        )
        self.configure_multi_plan_request_parameters(
            self.unconstrained_ompl_parameters,
            node.ompl_unconstrained_planning_time,
            node.ompl_planning_attempts,
            node.ompl_velocity_scaling,
            node.ompl_acceleration_scaling,
            "unconstrained OMPL",
        )
        self.configure_multi_plan_request_parameters(
            self.constrained_ompl_parameters,
            node.ompl_constrained_planning_time,
            node.ompl_planning_attempts,
            node.ompl_velocity_scaling,
            node.ompl_acceleration_scaling,
            "constrained OMPL",
        )
        self.configure_plan_request_parameters(
            self.linear_parameters,
            node.linear_planning_time,
            node.linear_planning_attempts,
            node.linear_velocity_scaling,
            node.linear_acceleration_scaling,
            "LIN",
        )
        self.home_orientation = None

        # Validate list lengths, value ranges, and cross-field relationships once.
        if len(node.workspace_min) != 3 or len(node.workspace_max) != 3:
            raise ValueError("planning workspace min/max must each contain xyz")
        if node.ompl_max_retries < 0:
            raise ValueError("ompl.max_retries must be zero or positive")
        if not node.ompl_unconstrained_parallel_profiles:
            raise ValueError(
                "motion.ompl.unconstrained.parallel_profiles must not be empty"
            )
        if len(set(node.ompl_unconstrained_parallel_profiles)) != len(
            node.ompl_unconstrained_parallel_profiles
        ):
            raise ValueError(
                "motion.ompl.unconstrained.parallel_profiles must be unique"
            )
        if not node.ompl_constrained_parallel_profiles:
            raise ValueError(
                "motion.ompl.constrained.parallel_profiles must not be empty"
            )
        if len(set(node.ompl_constrained_parallel_profiles)) != len(
            node.ompl_constrained_parallel_profiles
        ):
            raise ValueError("motion.ompl.constrained.parallel_profiles must be unique")
        if node.ompl_unconstrained_planning_time <= 0.0:
            raise ValueError(
                "motion.ompl.unconstrained.planning_time_sec must be positive"
            )
        if node.ompl_constrained_planning_time <= 0.0:
            raise ValueError(
                "motion.ompl.constrained.planning_time_sec must be positive"
            )
        if node.ompl_goal_joint_tolerance <= 0.0:
            raise ValueError("motion.ompl.goal_joint_tolerance_rad must be positive")
        if len(node.ompl_joint_travel_weights) != len(node.joint_names) or any(
            float(value) <= 0.0 for value in node.ompl_joint_travel_weights
        ):
            raise ValueError(
                "motion.ompl.joint_travel_weights must contain one positive "
                "value per joint"
            )
        if (
            not node.ompl_priority_joint_names
            or len(set(node.ompl_priority_joint_names))
            != len(node.ompl_priority_joint_names)
            or any(
                name not in node.joint_names for name in node.ompl_priority_joint_names
            )
        ):
            raise ValueError(
                "motion.ompl.priority_joint_names must contain unique "
                "configured joint names"
            )
        # Convert priority-joint names to configured joint-array indices once.
        self.ompl_priority_joint_indices = [
            node.joint_names.index(name) for name in node.ompl_priority_joint_names
        ]
        if node.ompl_priority_joint_early_acceptance <= 0.0:
            raise ValueError(
                "motion.ompl.priority_joint_early_acceptance_rad must be positive"
            )
        if node.ompl_priority_joint_max_travel <= 0.0:
            raise ValueError(
                "motion.ompl.priority_joint_max_travel_rad must be positive"
            )
        if (
            node.ompl_priority_joint_early_acceptance
            > node.ompl_priority_joint_max_travel
        ):
            raise ValueError(
                "motion.ompl.priority_joint_early_acceptance_rad must not "
                "exceed motion.ompl.priority_joint_max_travel_rad"
            )
        if node.ompl_priority_joint_additional_retries < 0:
            raise ValueError(
                "motion.ompl.priority_joint_additional_retries must be zero or positive"
            )
        if node.max_joint_travel <= 0.0:
            raise ValueError("max_joint_travel_rad must be positive")
        if node.max_trajectory_joint_step <= 0.0:
            raise ValueError("max_trajectory_joint_step_rad must be positive")
        if node.max_trajectory_joint_travel <= 0.0:
            raise ValueError("max_trajectory_joint_travel_rad must be positive")
        if node.ik_collision_free_attempts <= 0:
            raise ValueError("ik.collision_free_attempts must be positive")
        if node.ik_transfer_goal_candidates <= 0:
            raise ValueError("ik.transfer_goal_candidates must be positive")
        if node.ik_constrained_collision_free_attempts <= 0:
            raise ValueError("ik.constrained.collision_free_attempts must be positive")
        if node.ik_constrained_transfer_goal_candidates <= 0:
            raise ValueError("ik.constrained.transfer_goal_candidates must be positive")
        if node.ik_candidate_separation <= 0.0:
            raise ValueError("ik.candidate_separation_rad must be positive")
        if len(node.tcp_path_tip_offset) != 3:
            raise ValueError("tcp_path.tip_offset_in_flange must contain xyz")
        if len(node.tcp_path_color) != 4 or len(node.tcp_progress_color) != 4:
            raise ValueError("TCP path colors must contain [r, g, b, a]")
        if node.tcp_path_line_width <= 0.0 or node.tcp_progress_line_width <= 0.0:
            raise ValueError("TCP path line widths must be positive")
        if node.tcp_progress_point_scale <= 0.0:
            raise ValueError("tcp_path.progress_point_scale must be positive")
        if node.tcp_progress_publish_period <= 0.0:
            raise ValueError("tcp_path.progress_publish_period_sec must be positive")
        if node.tcp_progress_joint_tolerance <= 0.0:
            raise ValueError("tcp_path.progress_joint_tolerance_rad must be positive")
        if node.execution_goal_tolerance <= 0.0:
            raise ValueError("execution.goal_tolerance_rad must be positive")
        if node.execution_stopped_velocity_tolerance <= 0.0:
            raise ValueError(
                "execution.stopped_velocity_tolerance_rad_per_sec must be positive"
            )
        if node.execution_goal_stable_duration < 0.0:
            raise ValueError(
                "execution.goal_stable_duration_sec must be zero or positive"
            )
        if node.execution_goal_wait_timeout <= 0.0:
            raise ValueError("execution.goal_wait_timeout_sec must be positive")
        # Map each solution object ID to cumulative per-joint travel [rad].
        self._parallel_solution_travel: dict[int, np.ndarray] = {}

        # set_workspace takes min xyz then max xyz, all in world-frame metres.
        self.planning_component.set_workspace(
            *[float(value) for value in node.workspace_min],
            *[float(value) for value in node.workspace_max],
        )
        node.configure_planning_scene_service(self.planning_scene_monitor)

    @staticmethod
    def configure_plan_request_parameters(
        parameters: PlanRequestParameters,
        planning_time: float,
        planning_attempts: int,
        velocity_scaling: float,
        acceleration_scaling: float,
        name: str,
    ) -> None:
        """Validate and assign limits for one MoveIt planning request."""
        # planning_time is seconds; scaling factors are unitless fractions in (0, 1];
        # planning_attempts is a positive request count.
        if planning_time <= 0.0:
            raise ValueError(f"{name} planning time must be positive")
        if planning_attempts <= 0:
            raise ValueError(f"{name} planning attempts must be positive")
        if not 0.0 < velocity_scaling <= 1.0:
            raise ValueError(f"{name} velocity scaling must be in (0, 1]")
        if not 0.0 < acceleration_scaling <= 1.0:
            raise ValueError(f"{name} acceleration scaling must be in (0, 1]")
        # MoveItPy request objects are mutable and reused for every call.
        parameters.planning_time = planning_time
        parameters.planning_attempts = planning_attempts
        parameters.max_velocity_scaling_factor = velocity_scaling
        parameters.max_acceleration_scaling_factor = acceleration_scaling

    @classmethod
    def configure_multi_plan_request_parameters(
        cls,
        parameters: MultiPipelinePlanRequestParameters,
        planning_time: float,
        planning_attempts: int,
        velocity_scaling: float,
        acceleration_scaling: float,
        name: str,
    ) -> None:
        """Assign the same request limits to every parallel planner profile."""
        # MultiPipeline parameters hold one request object per configured profile.
        requests = list(parameters.multi_plan_request_parameters)
        if not requests:
            raise ValueError(f"{name} profiles must not be empty")
        for request in requests:
            cls.configure_plan_request_parameters(
                request,
                planning_time,
                planning_attempts,
                velocity_scaling,
                acceleration_scaling,
                name,
            )

    def current_robot_state(self) -> RobotState:
        """Return a copy of the current Planning Scene robot state."""
        # Return a detached RobotState with joint positions in radians.
        with self.planning_scene_monitor.read_only() as scene:
            return copy.deepcopy(scene.current_state)

    def make_joint_target(self, positions: list[float]) -> RobotState:
        """Return a joint goal after validating the position count."""
        # positions must follow node.joint_names and use radians for revolute joints.
        if len(positions) != len(self.node.joint_names):
            raise ValueError("joint target positions must match joint_names")
        # Construct a complete RobotState goal in the configured joint order.
        state = RobotState(self.robot_model)
        state.set_joint_group_positions(
            self.node.group_name, np.asarray(positions, dtype=float)
        )
        state.update()
        return state

    def make_pose(self, position: list[float]) -> PoseStamped:
        """Return a world-frame pose with the saved home orientation."""
        if self.home_orientation is None:
            raise RuntimeError("home orientation has not been captured")
        if len(position) != 3:
            raise ValueError("position must contain [x, y, z]")
        # PoseStamped position uses [x, y, z] metres; orientation is the copied unit quaternion captured at HOME.
        target = PoseStamped()
        target.header.frame_id = self.node.world_frame
        target.header.stamp = self.node.get_clock().now().to_msg()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = (
            position
        )
        target.pose.orientation = copy.deepcopy(self.home_orientation)
        return target

    def is_state_collision_free(self, state: RobotState) -> bool:
        """Return whether a robot state is collision-free in the current scene."""
        # Check the complete current Planning Scene for the configured group.
        with self.planning_scene_monitor.read_only() as scene:
            return not scene.is_state_colliding(
                state,
                self.node.group_name,
                False,
            )

    def solve_collision_free_ik_candidates(
        self,
        target: PoseStamped,
        maximum_candidates: int,
        maximum_attempts: Optional[int] = None,
        *,
        enforce_priority_joint_limit: bool = False,
    ) -> list[RobotState]:
        """Return distinct collision-free IK solutions ordered by joint travel."""
        # target is a world-frame PoseStamped in metres/quaternion form.
        # maximum_candidates limits accepted states; maximum_attempts limits seeds.
        if maximum_candidates <= 0:
            raise ValueError("maximum_candidates must be positive")
        attempt_limit = (
            self.node.ik_collision_free_attempts
            if maximum_attempts is None
            else maximum_attempts
        )
        if attempt_limit <= 0:
            raise ValueError("maximum_attempts must be positive")

        # Measure wall/process seconds around the IK candidate search only.
        wall_start = time.perf_counter()
        cpu_start = time.process_time()
        current_state = self.current_robot_state()
        current = np.asarray(
            current_state.get_joint_group_positions(self.node.group_name), dtype=float
        )
        # Keep the lexicographic score, RobotState, and radians vector together.
        candidates: list[tuple[tuple[float, ...], RobotState, np.ndarray]] = []
        solved_count = 0
        colliding_count = 0
        joint_delta_rejected_count = 0
        duplicate_count = 0

        for attempt in range(attempt_limit):
            candidate = copy.deepcopy(current_state)
            if attempt > 0:
                # Generate a new random joint seed for each additional IK attempt.
                candidate.set_to_random_positions(self.joint_model_group)
            if not candidate.set_from_ik(
                self.node.group_name,
                target.pose,
                self.node.tip_link,
                self.node.ik_timeout,
            ):
                continue
            solved_count += 1
            candidate.update()
            solution = np.asarray(
                candidate.get_joint_group_positions(self.node.group_name), dtype=float
            )
            try:
                # Validate radians branch limits before the Planning Scene check.
                self.validate_joint_branch(current, solution, "IK")
                if enforce_priority_joint_limit:
                    self.validate_priority_joint_travel(
                        np.abs(solution - current),
                        "transfer IK",
                    )
            except RuntimeError:
                joint_delta_rejected_count += 1
                continue
            if not self.is_state_collision_free(candidate):
                colliding_count += 1
                continue
            if any(
                float(np.linalg.norm(solution - accepted_solution))
                < self.node.ik_candidate_separation
                for _, _, accepted_solution in candidates
            ):
                # Reject candidates closer than ik_candidate_separation radians.
                duplicate_count += 1
                continue
            # Score absolute per-joint travel [rad] with the plan-selection key.
            score = self.joint_travel_score(
                np.abs(solution - current),
                planning_time=0.0,
            )
            candidates.append((score, candidate, solution))
            if len(candidates) >= maximum_candidates:
                break

        wall_sec = time.perf_counter() - wall_start
        cpu_sec = time.process_time() - cpu_start
        if not candidates:
            self.node.get_logger().info(
                "[IK_METRIC] success=false "
                f"attempts_used={attempt_limit} wall_sec={wall_sec:.6f} "
                f"cpu_sec={cpu_sec:.6f} avg_cores={cpu_sec / wall_sec:.3f}"
            )
            raise RuntimeError(
                "collision-free IK failed for "
                f"xyz=({target.pose.position.x:.3f}, "
                f"{target.pose.position.y:.3f}, {target.pose.position.z:.3f}); "
                f"attempts={attempt_limit}, "
                f"solved={solved_count}, colliding={colliding_count}, "
                f"joint_delta_rejected={joint_delta_rejected_count}"
            )
        candidates.sort(key=lambda candidate: candidate[0])
        # Return only the requested number of best collision-free branches.
        selected = candidates[:maximum_candidates]
        self.node.get_logger().info(
            f"[IK] goal_candidates={len(selected)} "
            f"attempts_used={attempt + 1} solved={solved_count} "
            f"colliding={colliding_count} duplicates={duplicate_count} "
            f"joint_delta_rejected={joint_delta_rejected_count} "
            f"wall_sec={wall_sec:.6f} cpu_sec={cpu_sec:.6f} "
            f"avg_cores={cpu_sec / wall_sec:.3f}"
        )
        return [candidate[1] for candidate in selected]

    def solve_ik_near_current(self, target: PoseStamped) -> RobotState:
        """Return the nearest accepted collision-free IK solution."""
        return self.solve_collision_free_ik_candidates(target, 1)[0]

    def make_joint_goal_constraints(
        self, candidate_states: list[RobotState]
    ) -> list[Constraints]:
        """Convert IK solutions to alternative OMPL joint goals."""
        # construct_joint_constraint produces one Constraints message per IK branch using the configured endpoint tolerance [rad].
        return [
            construct_joint_constraint(
                robot_state=candidate,
                joint_model_group=self.joint_model_group,
                tolerance=self.node.ompl_goal_joint_tolerance,
            )
            for candidate in candidate_states
        ]

    def validate_joint_branch(
        self, start: np.ndarray, goal: np.ndarray, source: str
    ) -> None:
        """Reject a goal whose joint displacement exceeds the configured limit."""
        expected_shape = (len(self.node.joint_names),)
        if start.shape != expected_shape or goal.shape != expected_shape:
            raise RuntimeError(
                f"{source} joint positions must contain one value per joint"
            )
        if not np.all(np.isfinite(start)) or not np.all(np.isfinite(goal)):
            raise RuntimeError(f"{source} joint positions must be finite")
        differences = np.abs(goal - start)
        # Compare direct per-joint displacement [rad] against max_joint_travel.
        if np.any(differences > self.node.max_joint_travel + 1e-9):
            index = int(np.argmax(differences))
            raise RuntimeError(
                f"{source} rejected joint-travel branch: "
                f"{self.node.joint_names[index]} delta={differences[index]:.3f} rad"
            )

    def validate_priority_joint_travel(self, travel: np.ndarray, source: str) -> None:
        """Reject configured priority-joint travel above its hard limit."""
        # travel contains cumulative radians in node.joint_names order.
        if travel.shape != (len(self.node.joint_names),):
            raise ValueError("joint travel must contain one value per joint")
        for index in self.ompl_priority_joint_indices:
            if travel[index] > self.node.ompl_priority_joint_max_travel + 1e-9:
                raise RuntimeError(
                    f"{source} rejected priority-joint travel: "
                    f"{self.node.joint_names[index]}={travel[index]:.3f} rad "
                    f"exceeds {self.node.ompl_priority_joint_max_travel:.3f} rad"
                )

    def orientation_constraint(self) -> Constraints:
        """Return a path constraint for the saved flange orientation."""
        # OrientationConstraint uses world_frame, tip_link, the HOME quaternion, three absolute axis tolerances [rad], and unitless weight 1.0.
        constraint = Constraints()
        orientation = OrientationConstraint()
        orientation.header.frame_id = self.node.world_frame
        orientation.link_name = self.node.tip_link
        orientation.orientation = copy.deepcopy(self.home_orientation)
        orientation.absolute_x_axis_tolerance = self.node.orientation_tolerance
        orientation.absolute_y_axis_tolerance = self.node.orientation_tolerance
        orientation.absolute_z_axis_tolerance = self.node.orientation_tolerance
        orientation.weight = 1.0
        constraint.orientation_constraints = [orientation]
        return constraint

    def joint_travel_score(
        self, travel: np.ndarray, planning_time: float
    ) -> tuple[float, ...]:
        """Return the configured joint-travel and planning-time sort key."""
        # travel is per-joint radians; planning_time is seconds.
        # Unitless positive weights scale only the travel terms.
        if travel.shape != (len(self.node.joint_names),):
            raise ValueError("joint travel must contain one value per joint")
        if not np.all(np.isfinite(travel)) or np.any(travel < 0.0):
            raise ValueError("joint travel values must be finite and nonnegative")
        weights = np.asarray(self.node.ompl_joint_travel_weights, dtype=float)
        weighted_travel = weights * travel
        # Lexicographic order is priority joints, max, total, then plan time.
        priority = tuple(
            float(weighted_travel[index]) for index in self.ompl_priority_joint_indices
        )
        return priority + (
            float(np.max(weighted_travel)),
            float(np.sum(weighted_travel)),
            float(planning_time),
        )

    def select_minimum_joint_travel_solution(self, solutions: list[Any]) -> Any:
        """Select the valid parallel result with the lowest configured score."""
        # Each truthy response supplies trajectory, planner_id, and planning_time seconds.
        if not solutions:
            raise RuntimeError("parallel planner returned no solution responses")
        successful = [solution for solution in solutions if solution]
        if not successful:
            return solutions[0]

        # Validate and score every successful candidate before selection.
        candidates = []
        for solution in successful:
            travel = self._parallel_solution_travel.get(id(solution))
            if travel is None:
                try:
                    travel = self.validate_trajectory(
                        solution.trajectory,
                        enforce_priority_joint_limit=True,
                    )
                except RuntimeError as error:
                    self.node.get_logger().warning(
                        f"[PLAN_SELECTION] rejected planner="
                        f"{solution.planner_id}: {error}"
                    )
                    continue
                self._parallel_solution_travel[id(solution)] = travel
            score = self.joint_travel_score(
                travel,
                planning_time=solution.planning_time,
            )
            candidates.append((score, solution, travel))

        if not candidates:
            # Return a successful response so the caller can report rejection.
            return successful[0]

        # min() applies the lexicographic score produced above.
        _score, selected, travel = min(
            candidates,
            key=lambda candidate: candidate[0],
        )
        formatted_travel = ",".join(
            f"{name}={value:.3f}" for name, value in zip(self.node.joint_names, travel)
        )
        priority_names = ",".join(self.node.ompl_priority_joint_names)
        priority_candidates = ";".join(
            f"{candidate_solution.planner_id}("
            + ",".join(
                f"{self.node.joint_names[index]}={candidate_travel[index]:.3f}"
                for index in self.ompl_priority_joint_indices
            )
            + ")"
            for _, candidate_solution, candidate_travel in candidates
        )
        self.node.get_logger().info(
            f"[PLAN_SELECTION] candidates={len(candidates)} "
            f"planner={selected.planner_id} joint_travel_rad=[{formatted_travel}] "
            f"priority_joints=[{priority_names}] "
            f"priority_candidates_rad=[{priority_candidates}]"
        )
        return selected

    def is_priority_joint_travel_acceptable(self, travel: np.ndarray) -> bool:
        """Return whether selected joints are within the acceptance limit."""
        if travel.shape != (len(self.node.joint_names),):
            raise ValueError("joint travel must contain one value per joint")
        return all(
            travel[index] <= self.node.ompl_priority_joint_early_acceptance + 1e-9
            for index in self.ompl_priority_joint_indices
        )

    def validate_trajectory(
        self,
        trajectory,
        *,
        enforce_priority_joint_limit: bool = False,
    ) -> np.ndarray:
        """Validate a trajectory and return cumulative configured-joint travel."""
        # RobotTrajectory.joint_trajectory points carry ordered joint positions in radians for the configured revolute joints.
        message = trajectory.get_robot_trajectory_msg()
        joint_trajectory = message.joint_trajectory
        points = joint_trajectory.points
        # Resolve every configured joint into each point's joint_names order.
        if not points:
            raise RuntimeError("planner returned an empty joint trajectory")
        if len(set(joint_trajectory.joint_names)) != len(joint_trajectory.joint_names):
            raise RuntimeError("trajectory contains duplicate joint names")
        try:
            indices = [
                joint_trajectory.joint_names.index(name)
                for name in self.node.joint_names
            ]
            positions = np.asarray(
                [[point.positions[index] for index in indices] for point in points],
                dtype=float,
            )
        except (IndexError, ValueError) as error:
            raise RuntimeError(
                "trajectory points do not contain every configured joint"
            ) from error
        if positions.shape != (len(points), len(self.node.joint_names)):
            raise RuntimeError(
                "trajectory points do not contain every configured joint"
            )
        if not np.all(np.isfinite(positions)):
            raise RuntimeError("trajectory joint positions must be finite")
        start = positions[0]
        previous = start
        cumulative_travel = np.zeros_like(start)
        for point_number, current in enumerate(positions[1:], start=1):
            # Accumulate absolute point-to-point radians for every joint.
            step = np.abs(current - previous)
            if np.any(step > self.node.max_trajectory_joint_step + 1e-9):
                joint = int(np.argmax(step))
                raise RuntimeError(
                    f"trajectory rejected: point {point_number} jumps "
                    f"{self.node.joint_names[joint]} by {step[joint]:.3f} rad"
                )
            cumulative_travel += step
            if enforce_priority_joint_limit:
                # Transfer trajectories enforce the hard priority-joint bound.
                self.validate_priority_joint_travel(
                    cumulative_travel,
                    "transfer trajectory cumulative",
                )
            if np.any(
                cumulative_travel >= self.node.max_trajectory_joint_travel - 1e-9
            ):
                joint = int(np.argmax(cumulative_travel))
                raise RuntimeError(
                    f"trajectory rejected: cumulative "
                    f"{self.node.joint_names[joint]} travel="
                    f"{cumulative_travel[joint]:.3f} rad"
                )
            # Every waypoint must stay on the same branch as the first point.
            self.validate_joint_branch(start, current, "trajectory")
            previous = current
        return cumulative_travel

    @staticmethod
    def transform_local_point(pose: Pose, offset: list[float]) -> Point:
        """Transform a pose-local point into world coordinates."""
        # pose/offset translations are metres and pose.orientation is a unit quaternion.
        # The returned Point uses the pose's parent frame.
        if len(offset) != 3:
            raise ValueError("offset must contain [x, y, z]")
        qx = float(pose.orientation.x)
        qy = float(pose.orientation.y)
        qz = float(pose.orientation.z)
        qw = float(pose.orientation.w)
        ox, oy, oz = (float(value) for value in offset)
        # Expand the quaternion rotation matrix and add pose translation.
        point = Point()
        point.x = float(pose.position.x) + (
            (1.0 - 2.0 * (qy * qy + qz * qz)) * ox
            + 2.0 * (qx * qy - qz * qw) * oy
            + 2.0 * (qx * qz + qy * qw) * oz
        )
        point.y = float(pose.position.y) + (
            2.0 * (qx * qy + qz * qw) * ox
            + (1.0 - 2.0 * (qx * qx + qz * qz)) * oy
            + 2.0 * (qy * qz - qx * qw) * oz
        )
        point.z = float(pose.position.z) + (
            2.0 * (qx * qz - qy * qw) * ox
            + 2.0 * (qy * qz + qx * qw) * oy
            + (1.0 - 2.0 * (qx * qx + qy * qy)) * oz
        )
        return point

    def make_tcp_path(self, trajectory_message: Any) -> list[Point]:
        """Return the TCP path obtained from trajectory forward kinematics."""
        # Input points contain joint positions [rad].
        # Output Points contain world-frame TCP positions [m], one per trajectory point.
        trajectory = trajectory_message.joint_trajectory
        # Resolve configured joint indices once for all trajectory points.
        if len(set(trajectory.joint_names)) != len(trajectory.joint_names):
            raise RuntimeError("trajectory contains duplicate joint names")
        if list(trajectory.joint_names) == self.node.joint_names:
            indices = None
        else:
            try:
                indices = [
                    trajectory.joint_names.index(name) for name in self.node.joint_names
                ]
            except ValueError as error:
                raise RuntimeError(
                    "trajectory joint names do not match configured joint_names"
                ) from error
        # Run forward kinematics at every waypoint, then apply the TCP offset [m].
        points = []
        state = RobotState(self.robot_model)
        for trajectory_point in trajectory.points:
            try:
                positions = (
                    trajectory_point.positions
                    if indices is None
                    else [trajectory_point.positions[index] for index in indices]
                )
            except IndexError as error:
                raise RuntimeError(
                    "trajectory points do not contain every configured joint"
                ) from error
            positions_array = np.asarray(positions, dtype=float)
            if positions_array.shape != (len(self.node.joint_names),):
                raise RuntimeError(
                    "trajectory points do not contain every configured joint"
                )
            if not np.all(np.isfinite(positions_array)):
                raise RuntimeError("trajectory joint positions must be finite")
            state.set_joint_group_positions(
                self.node.group_name,
                positions_array,
            )
            state.update()
            flange_pose = state.get_pose(self.node.tip_link)
            points.append(
                self.transform_local_point(
                    flange_pose,
                    self.node.tcp_path_tip_offset,
                )
            )
        return points

    def plan_and_execute(
        self,
        target: RobotState | PoseStamped | list[Constraints],
        parameters: PlanRequestParameters | MultiPipelinePlanRequestParameters,
        description: str,
        preserve_orientation: bool,
        max_retries: Optional[int] = None,
        parallel_planning: bool = False,
    ) -> RobotTrajectory:
        """Plan, validate, select, and execute a trajectory to the supplied goal."""
        # target is a PoseStamped [m/quaternion], alternative Constraints list, or RobotState [rad].
        # parameters selects single or multi-profile planning.
        target_is_pose = isinstance(target, PoseStamped)
        target_is_constraints = isinstance(target, list)
        goal_positions = None
        if not target_is_pose and not target_is_constraints:
            goal_positions = np.asarray(
                target.get_joint_group_positions(self.node.group_name), dtype=float
            )
        if target_is_constraints and not target:
            raise RuntimeError(f"no joint goal candidates: {description}")

        # Reset per-motion selection caches and retained valid responses.
        self._parallel_solution_travel.clear()
        attempt = 0
        result = None
        start_positions = None
        priority_retries_used = 0
        retained_results = []
        retained_start_positions = {}
        while rclpy.ok():
            # Every loop iteration submits one complete planning request.
            attempt += 1
            validation_error = None
            constraint_goal = None
            if target_is_constraints:
                # Submit alternative IK branches as ordered, one-branch goals.
                # Every request still runs all configured planning profiles.
                constraint_goal = [target[(attempt - 1) % len(target)]]
            current = self.current_robot_state()
            # Capture start positions [rad] for validation and RViz display.
            start_positions = np.asarray(
                current.get_joint_group_positions(self.node.group_name), dtype=float
            )
            if goal_positions is not None:
                self.validate_joint_branch(start_positions, goal_positions, description)

            # Set the latest measured start and exactly one supported goal form.
            self.planning_component.set_start_state_to_current_state()
            if target_is_pose:
                self.planning_component.set_goal_state(
                    pose_stamped_msg=target,
                    pose_link=self.node.tip_link,
                )
            elif target_is_constraints:
                self.planning_component.set_goal_state(
                    motion_plan_constraints=constraint_goal
                )
            else:
                self.planning_component.set_goal_state(robot_state=target)
            self.planning_component.set_path_constraints(
                self.orientation_constraint() if preserve_orientation else Constraints()
            )

            # Measure wall/process seconds around PlanningComponent.plan().
            self.node.set_status(f"PLAN_{description}")
            planning_wall_start = time.perf_counter()
            planning_cpu_start = time.process_time()
            if parallel_planning:
                # Submit all configured transfer profiles through multi_plan_parameters.
                result = self.planning_component.plan(
                    multi_plan_parameters=parameters,
                    solution_selection_function=(
                        self.select_minimum_joint_travel_solution
                    ),
                )
            else:
                # Submit one named profile for HOME, approach, or Pilz LIN.
                result = self.planning_component.plan(single_plan_parameters=parameters)
            planning_wall_sec = time.perf_counter() - planning_wall_start
            planning_cpu_sec = time.process_time() - planning_cpu_start
            planning_mode = "constrained" if preserve_orientation else "unconstrained"
            self.node.get_logger().info(
                f"[PLAN_METRIC] description={description} attempt={attempt} "
                f"mode={planning_mode} "
                f"parallel={str(parallel_planning).lower()} "
                f"success={str(bool(result)).lower()} "
                f"wall_sec={planning_wall_sec:.6f} "
                f"cpu_sec={planning_cpu_sec:.6f} "
                f"avg_cores={planning_cpu_sec / planning_wall_sec:.3f}"
            )
            if result:
                # Reuse selection-time validation when the callback supplied it.
                try:
                    travel = self._parallel_solution_travel.get(id(result))
                    if travel is None:
                        travel = self.validate_trajectory(
                            result.trajectory,
                            enforce_priority_joint_limit=target_is_constraints,
                        )
                except RuntimeError as error:
                    validation_error = str(error)
                    result = None
            if result and target_is_constraints:
                # Retain each valid transfer during optional preference retries.
                retained_results.append(result)
                retained_start_positions[id(result)] = start_positions.copy()
                if self.is_priority_joint_travel_acceptable(travel):
                    # Select retained candidates when priority travel is acceptable.
                    if len(retained_results) > 1:
                        result = self.select_minimum_joint_travel_solution(
                            retained_results
                        )
                        start_positions = retained_start_positions[id(result)]
                    break
                if (
                    priority_retries_used
                    < self.node.ompl_priority_joint_additional_retries
                ):
                    # Submit one additional request for a lower priority-joint path.
                    priority_retries_used += 1
                    priority_travel = ",".join(
                        f"{self.node.joint_names[index]}={travel[index]:.3f}"
                        for index in self.ompl_priority_joint_indices
                    )
                    self.node.get_logger().warning(
                        f"[PREFERENCE_RETRY] {description} "
                        f"priority_joint_travel_rad=[{priority_travel}] "
                        f"additional_request={priority_retries_used}/"
                        f"{self.node.ompl_priority_joint_additional_retries}"
                    )
                    self.node.set_status(f"PREFERENCE_RETRY_{description}")
                    result = None
                    continue
                # Choose the best retained path after preference requests finish.
                result = self.select_minimum_joint_travel_solution(retained_results)
                start_positions = retained_start_positions[id(result)]
                break
            if result:
                break
            if retained_results:
                # A later request failed, but an earlier valid transfer is safe.
                if (
                    priority_retries_used
                    < self.node.ompl_priority_joint_additional_retries
                ):
                    priority_retries_used += 1
                    self.node.get_logger().warning(
                        f"[PREFERENCE_RETRY] {description} planning_failed "
                        f"additional_request={priority_retries_used}/"
                        f"{self.node.ompl_priority_joint_additional_retries}"
                    )
                    self.node.set_status(f"PREFERENCE_RETRY_{description}")
                    continue
                result = self.select_minimum_joint_travel_solution(retained_results)
                start_positions = retained_start_positions[id(result)]
                break
            if max_retries is None:
                # Surface the first plan/validation error for non-retrying motions.
                if validation_error is not None:
                    raise RuntimeError(validation_error)
                raise RuntimeError(f"planning failed: {description}")
            if max_retries > 0 and attempt > max_retries:
                raise RuntimeError(
                    f"planning failed after {attempt} attempts: {description}"
                )
            retry_limit = "unlimited" if max_retries == 0 else str(max_retries)
            reason = (
                f"rejected={validation_error}"
                if validation_error is not None
                else "planning_failed"
            )
            self.node.get_logger().warning(
                f"[RETRY] {description} attempt={attempt} {reason}; "
                f"max_retries={retry_limit}"
            )
            self.node.set_status(f"RETRY_{description}")
            # Wait ompl_retry_delay seconds before the next request.
            time.sleep(self.node.ompl_retry_delay)
        if not rclpy.ok():
            raise KeyboardInterrupt

        # Only a validated selected result reaches the execution function.
        self.execute_trajectory(result.trajectory, description, start_positions)
        return result.trajectory

    def execute_trajectory(
        self,
        trajectory: RobotTrajectory,
        description: str,
        start_positions: np.ndarray,
    ) -> None:
        """Display and execute one already selected trajectory."""
        # Convert the selected RobotTrajectory to messages, publish it to RViz, compute TCP points [m], and register the final measured goal [rad].
        preparation_wall_start = time.perf_counter()
        preparation_cpu_start = time.process_time()
        trajectory_message = trajectory.get_robot_trajectory_msg()
        self.node.publish_planned_trajectory(
            start_positions.tolist(),
            trajectory_message,
        )
        tcp_points = self.make_tcp_path(trajectory_message)
        self.node.set_tcp_path(tcp_points, publish=False)
        self.node.start_trajectory_progress(
            trajectory_message,
            tcp_points,
            publish=False,
        )
        preparation_wall_sec = time.perf_counter() - preparation_wall_start
        preparation_cpu_sec = time.process_time() - preparation_cpu_start
        self.node.get_logger().info(
            f"[EXEC_PREP_METRIC] description={description} "
            f"points={len(tcp_points)} "
            f"wall_sec={preparation_wall_sec:.6f} "
            f"cpu_sec={preparation_cpu_sec:.6f} "
            f"avg_cores={preparation_cpu_sec / preparation_wall_sec:.3f}"
        )

        # Publish RUN status, planned path, and initial progress before execute().
        self.node.set_status(f"RUN_{description}", publish=False)
        self.node.publish_visual_markers()
        completed = False
        try:
            # Execute through MoveIt's configured trajectory controller manager.
            execution_status = self.robot.execute(trajectory, controllers=[])
            if not bool(execution_status):
                raise RuntimeError(
                    f"trajectory execution failed: {description} "
                    f"status={execution_status.status}"
                )
            # After action success, require measured position/stopped stability.
            self.node.wait_for_execution_goal(description)
            completed = True
        finally:
            self.node.finish_trajectory_progress(completed=completed)

    def move_joint_space_to_state(
        self,
        target_state: RobotState,
        description: str,
        preserve_orientation: bool = True,
    ) -> None:
        """Plan and execute collision-aware OMPL motion to a joint state."""
        # HOME and initial approach call the single-profile OMPL request here.
        self.plan_and_execute(
            target_state,
            self.ompl_parameters,
            f"OMPL_{description}",
            preserve_orientation,
            self.node.ompl_max_retries,
        )

    def move_joint_space_to_pose(
        self,
        target: PoseStamped,
        description: str,
        preserve_orientation: bool = True,
    ) -> None:
        """Plan and execute one complete transfer to accepted IK goals."""
        # Select constrained or unconstrained IK limits from the YAML mode.
        candidates = self.solve_collision_free_ik_candidates(
            target,
            (
                self.node.ik_constrained_transfer_goal_candidates
                if preserve_orientation
                else self.node.ik_transfer_goal_candidates
            ),
            (
                self.node.ik_constrained_collision_free_attempts
                if preserve_orientation
                else self.node.ik_collision_free_attempts
            ),
            enforce_priority_joint_limit=True,
        )
        # Try one ordered IK branch per request with all configured profiles.
        self.plan_and_execute(
            self.make_joint_goal_constraints(candidates),
            (
                self.constrained_ompl_parameters
                if preserve_orientation
                else self.unconstrained_ompl_parameters
            ),
            f"OMPL_{description}",
            preserve_orientation,
            self.node.ompl_max_retries,
            parallel_planning=True,
        )

    def move_linear_to_pose(self, target: PoseStamped, description: str) -> None:
        """Plan and execute a straight Cartesian segment with Pilz LIN."""
        # Convert the Cartesian pose [m/quaternion] to one collision-free joint endpoint [rad], then submit a single Pilz LIN request.
        target_state = self.solve_ik_near_current(target)
        self.plan_and_execute(
            target_state,
            self.linear_parameters,
            f"LIN_{description}",
            False,
        )

    def capture_home_orientation(self) -> None:
        """Save the current flange orientation for later Cartesian goals."""
        # Copy the measured HOME flange unit quaternion after HOME completion.
        state = self.current_robot_state()
        self.home_orientation = copy.deepcopy(
            state.get_pose(self.node.tip_link).orientation
        )
        self.node.get_logger().info(
            "Home flange orientation captured for every task-space target"
        )

    def run(self) -> None:
        """Initialize the scene and repeat complete A-to-B-to-A cycles."""
        if self.node.max_cycles < 0:
            raise ValueError("max_cycles must be zero (endless) or a positive integer")

        # Setup 1: Move to HOME and save its flange orientation.
        self.node.set_status("HOME")
        home = self.make_joint_target(self.node.home_joint_positions)
        self.move_joint_space_to_state(home, "HOME", preserve_orientation=False)
        self.capture_home_orientation()

        # Setup 2: Open the hand and add the configured Planning Scene.
        self.node.set_status("OPEN_HAND")
        self.node.command_hand(False)
        self.node.add_initial_scene()

        # Map A/B to approach/task/work xyz positions [m] for both leg directions.
        locations = {
            "A": {
                "approach": self.node.pick_approach,
                "task": self.node.pick_position,
                "work": self.node.work_pick_position,
            },
            "B": {
                "approach": self.node.place_approach,
                "task": self.node.place_position,
                "work": self.node.work_place_position,
            },
        }

        # Setup 3: Move to the A approach pose before the first cycle.
        self.node.set_status("APPROACH_A")
        initial_approach = self.solve_ik_near_current(
            self.make_pose(locations["A"]["approach"])
        )
        self.move_joint_space_to_state(initial_approach, "APPROACH_A")

        cycle = 0
        while rclpy.ok() and (
            self.node.max_cycles == 0 or cycle < self.node.max_cycles
        ):
            # One cycle is the complete A-to-B-to-A round trip.
            cycle += 1
            self.node.get_logger().info(f"[CYCLE] {cycle:03d} A_TO_B_TO_A")
            for leg, (source_name, destination_name) in enumerate(
                (("A", "B"), ("B", "A")),
                start=1,
            ):
                # The same seven documented steps run with source/destination swapped.
                source = locations[source_name]
                destination = locations[destination_name]
                self.node.get_logger().info(
                    f"[LEG] cycle={cycle:03d} leg={leg}/2 "
                    f"{source_name}_TO_{destination_name}"
                )

                # Step 1: Descend from the source approach to Pick.
                self.move_linear_to_pose(
                    self.make_pose(source["task"]),
                    f"PICK_{source_name}",
                )
                # Step 2: Close the hand and attach the workpiece.
                self.node.set_status(f"CLOSE_HAND_{source_name}")
                self.node.command_hand(True)
                self.node.attach_work()

                # Step 3: Lift back to the source approach with Pilz LIN.
                self.move_linear_to_pose(
                    self.make_pose(source["approach"]),
                    f"LIFT_{source_name}",
                )
                # Step 4: Transfer to the destination approach with OMPL.
                self.move_joint_space_to_pose(
                    self.make_pose(destination["approach"]),
                    f"TRANSFER_{source_name}_TO_{destination_name}",
                    preserve_orientation=(self.node.ompl_preserve_transfer_orientation),
                )
                # Step 5: Descend to the destination Place pose.
                self.move_linear_to_pose(
                    self.make_pose(destination["task"]),
                    f"PLACE_{destination_name}",
                )

                # Step 6: Open the hand and return the workpiece to the world.
                self.node.set_status(f"OPEN_HAND_{destination_name}")
                self.node.command_hand(False)
                self.node.detach_work_at(destination["work"])
                # Step 7: Retreat to the destination approach with Pilz LIN.
                self.move_linear_to_pose(
                    self.make_pose(destination["approach"]),
                    f"RETREAT_{destination_name}",
                )

        self.node.set_status(f"COMPLETE_{cycle}_CYCLES")


def main(args: Optional[list[str]] = None) -> None:
    """Run ROS callbacks and the MoveItPy application."""
    rclpy.init(args=args)
    node = PickPlaceNode()

    # Use two executor workers; callback groups admit only requested samples.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    def spin_callbacks() -> None:
        """Spin rclpy callbacks until shutdown."""
        try:
            executor.spin()
        except Exception:
            if rclpy.ok():
                raise

    spin_thread = None
    robot = None
    exit_code = 0
    try:
        # Let controllers start before constructing the heavier MoveItPy context.
        time.sleep(node.startup_delay)
        robot = MoveItPy(node_name="fanuc_pick_and_place_moveit_py")
        application = PickPlaceApplication(node, robot)

        # Start rclpy callbacks only after MoveItPy initialization finishes.
        spin_thread = threading.Thread(target=spin_callbacks, daemon=True)
        spin_thread.start()
        node.wait_for_trajectory_controller()
        if node.run_demo:
            # Normal mode executes HOME followed by repeated Pick & Place cycles.
            application.run()
        else:
            # Scene-only mode adds the Planning Scene and publishes READY.
            node.add_initial_scene()
            node.set_status("READY")
        while node.keep_alive and rclpy.ok():
            time.sleep(0.25)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        node.set_status("FAILED")
        node.get_logger().error(f"{type(error).__name__}: {error}")
        exit_code = 1
    finally:
        try:
            # Always emit the total metric and stop callback sources before teardown.
            node.log_total_cpu_metric()
            node.shutdown_visualization()
            executor.shutdown(timeout_sec=2.0)
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        finally:
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except KeyboardInterrupt:
                    pass
            if spin_thread is not None:
                spin_thread.join(timeout=2.0)
            if robot is not None:
                # MoveItPy owns background rclcpp objects that may outlive Python.
                sys.stdout.flush()
                sys.stderr.flush()
                os._exit(exit_code)
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
