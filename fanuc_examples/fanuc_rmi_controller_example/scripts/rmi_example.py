#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

# Please see the main() function for the overall flow.

import rclpy
import time
import math
import copy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from controller_manager_msgs.srv import SwitchController
from fanuc_msgs.msg import RMIStatus
from fanuc_msgs.srv import GetUToolData
from rmi_msgs.srv import (
    CallCommand,
    AddSetUTool,
    AddSetUFrame,
    AddMotionInstruction,
    AddCall,
    AddWaitTime,
)
from rmi_msgs.msg import (
    Command,
    MotionType,
    MotionInstructionOptions,
    Representation,
    SpeedType,
    TermType,
    Joint,
    Cartesian,
    CallParam,
)


def main(args=None):
    rclpy.init(args=args)
    node = RMIExampleApp()

    node.checkConfiguration()

    node.startup()
    # Switching controllers.
    # Initializing RMI.

    node.setUToolAndUFrame()
    # Adding instructions such as SetUtool.

    node.simpleJointMotion()
    # An simple joint (J) motion.
    # How to convert ROS 2 joint angles into FANUC's ones.
    # How to confirm that the sent instruction has finished.

    node.simplePickAndPlace()
    # Linear (L) motions connected by the CNT term_type.
    # How to convert tf from URDF into the caresian position for TP Programs.
    # Adding optional parameters, such as ACC20, to motion instructions.

    node.writeCircle()
    # Writing a circle with C instructions.
    # Relative motion from the last position (INCR)

    node.callProgram()
    # The CALL instruction to call KARELs and TP programs with parameters.

    node.finish()
    # Finishing RMI control.
    # Restoring the controllers.

    node.destroy_node()
    rclpy.shutdown()


class RMIExampleApp(Node):
    def __init__(self):
        super().__init__("rmi_example_app")

        # Prepare service requests
        self.switch_req = SwitchController.Request()
        self.command = CallCommand.Request()
        self.motion = AddMotionInstruction.Request()
        self.add_call = AddCall.Request()
        self.wait_time = AddWaitTime.Request()
        self.set_utool = AddSetUTool.Request()
        self.set_uframe = AddSetUFrame.Request()
        self.get_utool = GetUToolData.Request()

        # Subscribers
        self.subscriber_rs = self.create_subscription(
            RMIStatus, "/fanuc_gpio_controller/rmi_status", self.callback_rs, 10
        )

        # tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Service clients
        self.switch_controller_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.call_command_client = self.create_client(
            CallCommand, "/fanuc_rmi_controller/call_command"
        )
        self.add_motion_client = self.create_client(
            AddMotionInstruction, "/fanuc_rmi_controller/add_motion_instruction"
        )
        self.add_call_client = self.create_client(
            AddCall, "/fanuc_rmi_controller/add_call_instruction"
        )

        self.add_wait_client = self.create_client(
            AddWaitTime, "/fanuc_rmi_controller/add_wait_time_instruction"
        )

        self.set_utool_client = self.create_client(
            AddSetUTool, "/fanuc_rmi_controller/add_set_utool_instruction"
        )

        self.set_uframe_client = self.create_client(
            AddSetUFrame, "/fanuc_rmi_controller/add_set_uframe_instruction"
        )

        self.get_utool_client = self.create_client(
            GetUToolData, "/fanuc_gpio_controller/get_utool_data"
        )

        while not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        while not self.call_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        while not self.get_utool_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    def callback_rs(self, msg):
        self.rmi_status = msg

    def checkConfiguration(self):
        # Check the robot controller's tool setting.
        # This example expects that the all values in UTool[1] are 0.

        # Most Command packets are prepared as the fanuc_gpio_controller's service.
        # They are available even when the fanuc_rmi_controller does not have motion control.
        self.get_utool.utool_number = 1

        future = self.get_utool_client.call_async(self.get_utool)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.result != 0:
            self.get_logger().error("Getting UToolData failed.")
            return

        self.get_logger().info(
            "\nUTool[%d] group: %d\nx: %f y: %f z: %f\nw: %f p: %f r: %f"
            % (
                response.utool_number,
                response.group,
                response.frame.x,
                response.frame.y,
                response.frame.z,
                response.frame.w,
                response.frame.p,
                response.frame.r,
            )
        )

        if (
            (response.frame.x != 0.0)
            or (response.frame.y != 0.0)
            or (response.frame.z != 0.0)
            or (response.frame.w != 0.0)
            or (response.frame.p != 0.0)
            or (response.frame.r != 0.0)
        ):
            self.get_logger().error("Please configure UTool[1] to 0 (=Flange).")
            raise ValueError("This example expects all values in UTool[1] are 0.")
            return

    def startup(self):
        # == Switch controllers ==
        # Deactivate joint_trajectory_controller, and activate fanuc_rmi_controller.
        # The trajectory control from MoveIt will be deactivated, and RMI control will be activated.
        self.switch_req.activate_controllers = ["fanuc_rmi_controller"]
        self.switch_req.deactivate_controllers = ["joint_trajectory_controller"]
        self.switch_req.strictness = SwitchController.Request.AUTO
        self.switch_req.activate_asap = False
        self.switch_req.timeout.sec = 5

        future = self.switch_controller_client.call_async(self.switch_req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if not response.ok:
            self.get_logger().error("Controller switching failed.: " + response.message)
            return

        # == Start RMI ==
        # Send FRC_Initialize to start RMI control.
        self.command.command.type = Command.INIT
        # Note: FRC_Initialize has some optional parameters.
        #       Specify like following way when using them.
        # self.command.options.use_pltzmode = True
        # self.command.options.pltzmode = CommandOptions.PLTZ_ZERODN

        future = self.call_command_client.call_async(self.command)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.result != 0:
            self.get_logger().error("FRC_Initialize failed.")
            return

        # Check program execution status
        time.sleep(0.1)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.rmi_status.program_status == 0:
                self.get_logger().info("RMI Program started.")
                break
            time.sleep(0.01)

    def setUToolAndUFrame(self):
        # Set UToool and UFrame numbers.
        # This example uses UTool[1] and UFrame[0] (=World).

        self.set_utool.tool_number = 1
        self.set_uframe.frame_number = 0

        future = self.set_utool_client.call_async(self.set_utool)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.result != 0:
            self.get_logger().error("FRC_SetUTool failed.")
            return

        future = self.set_uframe_client.call_async(self.set_uframe)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.result != 0:
            self.get_logger().error("FRC_SetFrame failed.")
            return

        # When the instruction has finished, the last_sequence in the rmi_status reaches to the sequence_id in the action's response.
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.rmi_status.last_sequence == response.sequence_id:
                self.get_logger().info("Sequence number reached the command's value.")
                break
            time.sleep(0.01)

    def simpleJointMotion(self):
        # Send a simple Joint motion
        # J P[1] 10% FINE
        # P[1] = {0.0, 0.0, 0.0, 0.0, -90.0, 0.0}
        self.get_logger().info("Joint motion")

        # The following three elements defines the packet type
        self.motion.motion_type.type = MotionType.J  # JointMotion
        self.motion.incremental = False  # Not Relative
        self.motion.representation.representation = Representation.J  # JRep
        # Fill "joint" when the representation is joint.
        self.motion.joint = self.toFANUCJoints(
            [0.0, 0.0, 0.0, 0.0, -math.pi / 2.0, 0.0]
        )
        self.motion.speed_type.type = SpeedType.PERCENT
        self.motion.speed_value = 10
        self.motion.term_type.type = TermType.FINE
        # The term_value is ignored when the term_type is FINE.

        future = self.add_motion_client.call_async(self.motion)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.result != 0:
            self.get_logger().error("FRC_JointMotionJRep failed.")
            return

        # When the instruction has finished, the last_sequence in the rmi_status reaches to the sequence_id in the action's response.
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.rmi_status.last_sequence == response.sequence_id:
                self.get_logger().info("Sequence number reached the command's value.")
                break
            time.sleep(0.01)

    def toFANUCJoints(self, ros_joints):
        # Conversion from ROS 2's joints to FANUC's ones
        fanuc_joints = Joint()
        fanuc_joints.positions = [
            math.degrees(j) for j in ros_joints
        ]  # Convert to degrees
        # FANUC's J3 has special definition
        # https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/troubleshooting/troubleshooting.html#j3-value-differs-from-the-value-on-the-teach-pendant
        fanuc_joints.positions[2] = math.degrees(ros_joints[2] - ros_joints[1])
        return fanuc_joints

    def simplePickAndPlace(self):
        # == How to create cartesian position from tf ==
        # Get the robot's flange position from tf.
        try:
            rclpy.spin_once(self)
            t = self.tf_buffer.lookup_transform(
                "wbase",  # A dummy link in URDF to specify FANUC robot controller's world frame
                "fanuc_flange",  # A dummy link in URDF to specify FANUC robot's flange frame
                rclpy.time.Time(),
            )
        except TransformException as ex:
            self.get_logger().info(f"Could not transform wbase to fanuc_flange: {ex}")
            return

        self.get_logger().info("Linear motion")

        self.motion.motion_type.type = MotionType.L  # LinearMotion
        self.motion.incremental = False  # Not Relative
        self.motion.representation.representation = Representation.CART  # Cartesian
        # Fill "cartesian" when the representation is cartesian.
        self.motion.cartesian = self.tf2FANUCCartesian(t.transform)
        # Note: When the speed value is faster than the collaborative speed,
        #       the robot controller will automatically reduces the speed.
        self.motion.speed_type.type = SpeedType.MMSEC  # [mm/sec]
        self.motion.speed_value = 250  # 250 [mm/sec]
        self.motion.term_type.type = TermType.CNT
        self.motion.term_value = 100

        # MotionInstructions have some optional parameters.
        # To include them into the RMI packets, set "options.use_***" to True
        # and set "options.***" to the desired value.
        # Here is an example to add ACC20 to the liner motion.
        self.motion.options.use_acc = True
        self.motion.options.acc = 20

        # Send a simple Pick & Place self.motion
        self.motion.cartesian.y -= 100.0
        self.motion.cartesian.z -= 100.0
        self.motion.term_type.type = TermType.CNT
        self.motion.term_value = 100
        future = self.add_motion_client.call_async(self.motion)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        self.motion.options = (
            MotionInstructionOptions()
        )  # Reset options for the later instructions.
        self.motion.cartesian.z -= 100.0
        self.motion.term_type.type = TermType.FINE
        future = self.add_motion_client.call_async(self.motion)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        # Note: When the term_type is CNT, the sequence number won't be updated until the next instruction finishes.
        #       Be sure that the term_type is FINE when waiting for the latest instruction.v
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.rmi_status.last_sequence == response.sequence_id:
                self.get_logger().info("Sequence number reached the command's value.")
                break
            time.sleep(0.01)

        self.motion.cartesian.z += 100.0
        self.motion.term_type.type = TermType.CNT
        self.motion.term_value = 100
        future = self.add_motion_client.call_async(self.motion)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        self.motion.cartesian.y += 200.0
        self.motion.term_type.type = TermType.CNT
        self.motion.term_value = 100
        future = self.add_motion_client.call_async(self.motion)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        self.motion.cartesian.z -= 100.0
        self.motion.term_type.type = TermType.FINE
        future = self.add_motion_client.call_async(self.motion)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        while rclpy.ok():
            rclpy.spin_once(self)
            if self.rmi_status.last_sequence == response.sequence_id:
                self.get_logger().info("Sequence number reached the command's value.")
                break
            time.sleep(0.01)

        self.motion.cartesian.z += 100.0
        self.motion.term_type.type = TermType.CNT
        self.motion.term_value = 100
        future = self.add_motion_client.call_async(self.motion)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        self.motion.cartesian.y -= 100.0
        self.motion.term_type.type = TermType.FINE
        future = self.add_motion_client.call_async(self.motion)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        while rclpy.ok():
            rclpy.spin_once(self)
            if self.rmi_status.last_sequence == response.sequence_id:
                self.get_logger().info("Sequence number reached the command's value.")
                break
            time.sleep(0.01)

    def tf2FANUCCartesian(self, tf):
        fanuc_cart = Cartesian()
        # Configurations
        fanuc_cart.utool = 1
        fanuc_cart.uframe = 0
        fanuc_cart.front = 1
        fanuc_cart.up = 1
        fanuc_cart.left = 0  # 6axes robot does not have R/L configuration
        fanuc_cart.turn4 = 0
        fanuc_cart.turn5 = 0
        fanuc_cart.turn6 = 0

        # m to mm
        fanuc_cart.x = tf.translation.x * 1000.0
        fanuc_cart.y = tf.translation.y * 1000.0
        fanuc_cart.z = tf.translation.z * 1000.0
        # Convert Quaternion to Euler angles
        roll, pitch, yaw = self.q_to_euler(tf.rotation)
        fanuc_cart.w = math.degrees(roll)
        fanuc_cart.p = math.degrees(pitch)
        fanuc_cart.r = math.degrees(yaw)

        return fanuc_cart

    def q_to_euler(self, rotation):
        x = rotation.x
        y = rotation.y
        z = rotation.z
        w = rotation.w
        roll = math.atan2(2.0 * (w * x + y * z), (1.0 - 2.0 * (x * x + y * y)))
        pitch_tmp = 2.0 * (w * y - z * x)
        if abs(pitch_tmp) > 1.0:
            pitch = math.copysign(math.pi / 2.0, pitch_tmp)
        else:
            pitch = math.asin(pitch_tmp)
        yaw = math.atan2(2.0 * (w * z + x * y), (1.0 - 2.0 * (y * y + z * z)))

        return roll, pitch, yaw

    def writeCircle(self):
        # Circular motion
        self.get_logger().info("Circular motion")

        self.motion.motion_type.type = MotionType.C  # CircularMotion
        self.motion.incremental = True  # Relative
        self.motion.representation.representation = Representation.CART  # Cartesian
        # Set relative position
        self.motion.cartesian.x = 0.0
        self.motion.cartesian.y = 0.0
        self.motion.cartesian.z = -200.0
        self.motion.cartesian.w = 0.0
        self.motion.cartesian.p = 0.0
        self.motion.cartesian.r = 0.0
        # Circular motion additionally requires `via_` position.
        self.motion.via_representation = copy.deepcopy(self.motion.representation)
        self.motion.via_cartesian = copy.deepcopy(self.motion.via_cartesian)
        self.motion.via_cartesian.y = 100.0
        self.motion.via_cartesian.z = -100.0
        self.motion.speed_type.type = SpeedType.MMSEC  # [mm/sec]
        self.motion.speed_value = 250  # 250 [mm/sec]
        self.motion.term_type.type = TermType.CNT
        self.motion.term_value = 100

        future = self.add_motion_client.call_async(self.motion)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        self.motion.cartesian.y = 0.0
        self.motion.cartesian.z = 200.0
        self.motion.via_cartesian.y = -100.0
        self.motion.via_cartesian.z = 100.0
        self.motion.term_type.type = TermType.FINE

        future = self.add_motion_client.call_async(self.motion)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        while rclpy.ok():
            rclpy.spin_once(self)
            if self.rmi_status.last_sequence == response.sequence_id:
                self.get_logger().info("Sequence number reached the command's value.")
                break
            time.sleep(0.01)

    def callProgram(self):
        # An example to call KAREL with a constant integer argument.
        # CALL SET_LOW_FORCE_LIMIT_SENSITIVITY(1)
        self.get_logger().info("Call a KAREL")

        # Specify the program name to call.
        self.add_call.program_name = (
            "SET_LOW_FORCE_LIMIT_SENSITIVITY"  # A built-in KAREL for CRX
        )

        # Add parameters if needed.
        param = CallParam()
        param.type = CallParam.INT
        param.int_value = 1  # SET_LOW_FORCE_SENSITIVITY accepts 0 or 1
        self.add_call.params.append(
            param
        )  # The params array accepts up to 10 parameters.

        future = self.add_call_client.call_async(self.add_call)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.result != 0:
            self.get_logger().error("FRC_Call failed.")
            return

        time.sleep(1.0)

        # CALL SET_LOW_FORCE_LIMIT_SENSITIVITY(0)
        self.add_call.params[0].int_value = 0
        future = self.add_call_client.call_async(self.add_call)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.result != 0:
            self.get_logger().error("FRC_Call failed.")
            return

        # Note: The sequence number for FRC_Call won't be updated until the next instruction finishes the same as CNT instruction.
        #       In such case, it is effective to add WaitTime instruction with very short time.
        self.wait_time.time = 0.1
        future = self.add_wait_client.call_async(self.wait_time)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.result != 0:
            self.get_logger().error("FRC_WaitTime failed.")
            return

        while rclpy.ok():
            rclpy.spin_once(self)
            if self.rmi_status.last_sequence == response.sequence_id:
                self.get_logger().info("Sequence number reached the command's value.")
                break
            time.sleep(0.01)

    def finish(self):
        # Send FRC_Abort to finish RMI control
        self.command.command.type = Command.ABORT
        future = self.call_command_client.call_async(self.command)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        # Check program execution status
        time.sleep(0.1)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.rmi_status.program_status == 2:
                self.get_logger().info("RMI Program has been finished.")
                break

        # Switch the active controller from RMI to SJTC.
        # Trajectory control from MoveIt will be available again.
        self.switch_req.activate_controllers = ["joint_trajectory_controller"]
        self.switch_req.deactivate_controllers = ["fanuc_rmi_controller"]
        future = self.switch_controller_client.call_async(self.switch_req)

        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if not response.ok:
            self.get_logger().error("Controller switching failed.: " + response.message)
            return


main()
