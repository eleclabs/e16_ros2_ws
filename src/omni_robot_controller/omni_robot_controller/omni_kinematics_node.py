#!/usr/bin/env python3
"""ROS 2 controller for a 3-wheel Kiwi/omni drive.

Wheel order is intentionally identical to omni_robot_description:
  wheel_1_front_left  : theta = +60 deg
  wheel_2_front_right : theta = -60 deg
  wheel_3_rear        : theta = 180 deg

The wheel angles describe the radial position of each wheel around base_link.
The drive direction is tangent to that circle.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node


class OmniKinematicsNode(Node):
    def __init__(self):
        super().__init__('omni_kinematics_node')

        self.declare_parameter('wheel_radius', 0.032)
        self.declare_parameter('wheel_base_radius', 0.115)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('max_wheel_velocity', 20.0)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.robot_radius = float(self.get_parameter('wheel_base_radius').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.max_wheel_velocity = abs(float(self.get_parameter('max_wheel_velocity').value))

        # Wheel order: FL, FR, Rear.
        self.wheel_angles = (
            math.radians(60.0),
            math.radians(-60.0),
            math.radians(180.0),
        )

        self.last_cmd_time = self.get_clock().now()
        self.last_wheel_velocity = [0.0, 0.0, 0.0]

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_cmd', 10
        )
        self.timer = self.create_timer(0.05, self.watchdog_callback)

        self.get_logger().info(
            'Omni kinematics ready: Wheel 1=Front Left, '
            'Wheel 2=Front Right, Wheel 3=Rear'
        )

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        wheel_angular = compute_wheel_angular_velocities(
            vx, vy, wz, self.wheel_angles, self.wheel_radius, self.robot_radius
        )

        # Preserve the commanded motion direction when limiting speed.
        wheel_angular = limit_wheel_velocities(
            wheel_angular, self.max_wheel_velocity
        )
        self.publish_wheel_velocity(wheel_angular)
        self.last_wheel_velocity = wheel_angular
        self.last_cmd_time = self.get_clock().now()

    def publish_wheel_velocity(self, velocities):
        msg = Float64MultiArray()
        msg.data = velocities
        self.wheel_pub.publish(msg)

    def watchdog_callback(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout and any(abs(v) > 1e-9 for v in self.last_wheel_velocity):
            self.publish_wheel_velocity([0.0, 0.0, 0.0])
            self.last_wheel_velocity = [0.0, 0.0, 0.0]


def compute_wheel_angular_velocities(vx, vy, wz, wheel_angles, wheel_radius, robot_radius):
    """Convert body twist [m/s, m/s, rad/s] to wheel angular velocity [rad/s]."""
    if wheel_radius <= 0.0 or robot_radius < 0.0:
        raise ValueError('wheel_radius must be > 0 and robot_radius must be >= 0')

    wheel_linear = [
        -math.sin(theta) * vx + math.cos(theta) * vy + robot_radius * wz
        for theta in wheel_angles
    ]
    return [v / wheel_radius for v in wheel_linear]


def limit_wheel_velocities(velocities, max_velocity):
    """Scale all wheels together so no wheel exceeds max_velocity."""
    if max_velocity <= 0.0:
        return [0.0 for _ in velocities]
    peak = max((abs(v) for v in velocities), default=0.0)
    if peak <= max_velocity:
        return list(velocities)
    scale = max_velocity / peak
    return [v * scale for v in velocities]


def main(args=None):
    rclpy.init(args=args)
    node = OmniKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
