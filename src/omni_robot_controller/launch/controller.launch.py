from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('omni_robot_controller'),
        'config',
        'controller.yaml',
    )

    use_simulation = LaunchConfiguration('use_simulation')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_simulation',
            default_value='false',
            description='Start the open-loop simulation hardware interface. Keep false for ESP32 micro-ROS hardware.',
        ),
        Node(
            package='omni_robot_controller',
            executable='omni_kinematics_node',
            name='omni_kinematics_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='omni_robot_controller',
            executable='omni_hardware_interface',
            name='omni_hardware_interface',
            output='screen',
            parameters=[config, {'simulation_mode': use_simulation}],
            condition=IfCondition(use_simulation),
        ),
        Node(
            package='omni_robot_controller',
            executable='omni_odometry_node',
            name='omni_odometry_node',
            output='screen',
            parameters=[config],
        ),
    ])
