#!/usr/bin/env python3
"""Safe open-loop hardware abstraction for the 3-wheel Omni robot.

This node is intentionally hardware-neutral. It accepts wheel angular
velocity commands and republishes them as feedback for simulation/testing.
For real hardware, replace the publish path with the ESP32/micro-ROS driver
while keeping the same topic contract and wheel order.

Wheel order:
  1 = Front Left
  2 = Front Right
  3 = Rear Center
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class OmniHardwareInterface(Node):
    def __init__(self):
        super().__init__('omni_hardware_interface')

        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('max_wheel_velocity', 20.0)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('simulation_mode', True)

        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.max_wheel_velocity = float(self.get_parameter('max_wheel_velocity').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.simulation_mode = bool(self.get_parameter('simulation_mode').value)

        self.wheel_command = [0.0, 0.0, 0.0]
        self.last_command_time = self.get_clock().now()

        self.command_sub = self.create_subscription(
            Float64MultiArray,
            '/wheel_velocity_cmd',
            self.command_callback,
            10,
        )
        self.feedback_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocity_feedback',
            10,
        )
        self.timer = self.create_timer(1.0 / max(publish_rate, 1.0), self.update)

        mode = 'SIMULATION' if self.simulation_mode else 'HARDWARE PLACEHOLDER'
        self.get_logger().info(
            f'Omni hardware interface ready [{mode}] | '
            'W1=Front Left, W2=Front Right, W3=Rear'
        )

    def command_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 3:
            self.get_logger().warning(
                f'Expected 3 wheel commands, received {len(msg.data)}'
            )
            return

        limit = abs(self.max_wheel_velocity)
        self.wheel_command = [
            max(-limit, min(limit, float(value))) for value in msg.data
        ]
        self.last_command_time = self.get_clock().now()

        # Real hardware integration point:
        #   set_motor_velocity(1, self.wheel_command[0])
        #   set_motor_velocity(2, self.wheel_command[1])
        #   set_motor_velocity(3, self.wheel_command[2])

    def update(self):
        elapsed = (
            self.get_clock().now() - self.last_command_time
        ).nanoseconds / 1e9

        if elapsed > self.command_timeout:
            feedback = [0.0, 0.0, 0.0]
        elif self.simulation_mode:
            # Open-loop test only. Real hardware must publish encoder-derived
            # angular velocity through the same feedback topic.
            feedback = list(self.wheel_command)
        else:
            # Never pretend that hardware feedback exists when hardware mode
            # is selected but no driver has been connected yet.
            feedback = [0.0, 0.0, 0.0]

        msg = Float64MultiArray()
        msg.data = feedback
        self.feedback_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OmniHardwareInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
