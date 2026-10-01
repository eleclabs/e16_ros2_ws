#!/usr/bin/env python3
"""Odometry for the 3-wheel Kiwi/omni robot.

Wheel order:
  1 = front-left
  2 = front-right
  3 = rear-center

Input /wheel_velocity_feedback contains wheel angular velocity [rad/s].
The node converts wheel velocity to robot body velocity, integrates x/y/yaw,
and publishes nav_msgs/Odometry plus odom -> base_link TF.
"""

import math

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class OmniOdometryNode(Node):
    def __init__(self):
        super().__init__('omni_odometry_node')

        self.declare_parameter('wheel_radius', 0.032)
        self.declare_parameter('wheel_base_radius', 0.115)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)

        self.r = float(self.get_parameter('wheel_radius').value)
        self.R = float(self.get_parameter('wheel_base_radius').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        # Wheel order: FL, FR, Rear.
        self.angles = tuple(math.radians(a) for a in (60.0, -60.0, 180.0))

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()
        self.last_wheel = None

        self.feedback_sub = self.create_subscription(
            Float64MultiArray,
            '/wheel_velocity_feedback',
            self.feedback_callback,
            20,
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.get_logger().info(
            'Omni odometry ready: Wheel 1=Front Left, '
            'Wheel 2=Front Right, Wheel 3=Rear'
        )

    def feedback_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 3:
            self.get_logger().warn(
                f'Expected 3 wheel velocities, received {len(msg.data)}'
            )
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        # Convert angular wheel velocity [rad/s] to tangential velocity [m/s].
        wheel_v = [self.r * float(v) for v in msg.data]

        # v_i = -sin(theta_i)*vx + cos(theta_i)*vy + R*wz
        # For the symmetric 3-wheel layout, A^T A is diagonal.
        vx = sum(-math.sin(t) * v for t, v in zip(self.angles, wheel_v)) * (2.0 / 3.0)
        vy = sum(math.cos(t) * v for t, v in zip(self.angles, wheel_v)) * (2.0 / 3.0)
        wz = sum(wheel_v) / (3.0 * self.R)

        # Integrate body velocity into odom/world coordinates.
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        self.x += (c * vx - s * vy) * dt
        self.y += (s * vx + c * vy) * dt
        self.yaw += wz * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        self.publish(now, vx, vy, wz)

    def publish(self, stamp, vx, vy, wz):
        q = Quaternion()
        q.z = math.sin(self.yaw / 2.0)
        q.w = math.cos(self.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        # Non-zero covariance is important when this odometry is fused by EKF.
        odom.pose.covariance[0] = 0.02 * 0.02   # x
        odom.pose.covariance[7] = 0.02 * 0.02   # y
        odom.pose.covariance[35] = 0.05 * 0.05  # yaw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.twist.covariance[0] = 0.05 * 0.05
        odom.twist.covariance[7] = 0.05 * 0.05
        odom.twist.covariance[35] = 0.10 * 0.10
        self.odom_pub.publish(odom)

        # In the real robot, robot_localization EKF owns odom -> base_link TF.
        # Keep this optional for standalone odometry testing only.
        if self.publish_tf:
            from geometry_msgs.msg import TransformStamped
            from tf2_ros import TransformBroadcaster
            if not hasattr(self, '_tf_broadcaster'):
                self._tf_broadcaster = TransformBroadcaster(self)
            tf = TransformStamped()
            tf.header.stamp = stamp.to_msg()
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = q
            self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OmniOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
