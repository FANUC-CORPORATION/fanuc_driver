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

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time


class SineWavePublisher(Node):
    """
    A ROS2 node that publishes sine wave commands to the forward_command_controller.

    This node generates sine wave trajectories for a single joint at a time and publishes
    them to the forward_position_controller's commands topic.
    """

    def __init__(self):
        super().__init__("sine_wave_publisher")

        # Hardcoded parameters
        self.amplitude = math.radians(10)  # 10 degrees in radians
        self.frequency = 0.1  # Hz
        self.publish_rate = 500.0  # Hz (matches controller manager update rate)

        # Create publisher with larger queue size
        self.publisher = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 100
        )

        # Create timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_command)

        # Initialize time
        self.start_time = time.time()

        self.get_logger().info("Sine wave publisher started:")
        self.get_logger().info("  Joints: All 6 joints (J1-J6)")
        self.get_logger().info(f"  Amplitude: {self.amplitude} radians")
        self.get_logger().info(f"  Frequency: {self.frequency} Hz")
        self.get_logger().info(f"  Publish rate: {self.publish_rate} Hz")
        self.get_logger().info("  Publishing to: /forward_position_controller/commands")

    def publish_command(self):
        """Publish trajectory-style sine wave command for all joints."""
        # Calculate current time
        current_time = time.time() - self.start_time

        # Calculate phase (similar to send_trajectory.py)
        period = 1.0 / self.frequency  # Convert frequency to period
        phase = (2.0 * math.pi / period) * current_time

        # Calculate trajectory-style sine wave: amplitude * (1.0 - cos(phase))
        # This creates a smooth rise from 0 to 2*amplitude and back to 0
        trajectory_value = self.amplitude * (1.0 - math.cos(phase))

        # Create message with 6 joint positions (all joints get the same trajectory value)
        msg = Float64MultiArray()
        msg.data = [trajectory_value] * 6  # Apply trajectory wave to all joints

        # Publish message
        self.publisher.publish(msg)

        # Log current value (only every 50th message to avoid spam)
        if int(current_time * self.publish_rate) % 50 == 0:
            self.get_logger().info(f"All joints: {trajectory_value:.3f} rad")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = SineWavePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
