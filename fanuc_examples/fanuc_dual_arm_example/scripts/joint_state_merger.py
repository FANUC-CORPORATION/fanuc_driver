#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMerger(Node):
    def __init__(self):
        super().__init__("joint_state_merger")

        self.joint_states = {}

        self.subscriber_rarm = self.create_subscription(
            JointState, "/rarm/joint_states", self.callback_rarm, 10
        )
        self.subscriber_larm = self.create_subscription(
            JointState, "/larm/joint_states", self.callback_larm, 10
        )

        self.publisher_merged = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(0.05, self.publish_merged_joint_states)

    def callback_rarm(self, msg):
        self.joint_states[0] = msg

    def callback_larm(self, msg):
        self.joint_states[1] = msg

    def publish_merged_joint_states(self):
        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()

        for js in self.joint_states.values():
            merged.name.extend(js.name)
            merged.position.extend(js.position)
            merged.velocity.extend(js.velocity)
            merged.effort.extend(js.effort)

        self.publisher_merged.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


main()
