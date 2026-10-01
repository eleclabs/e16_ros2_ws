from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    controller_share = Path(get_package_share_directory('omni_robot_controller'))
    localization_share = Path(get_package_share_directory('omni_robot_localization'))

    controller_launch = controller_share / 'launch' / 'controller.launch.py'
    ekf_config = localization_share / 'config' / 'ekf.yaml'

    use_simulation = LaunchConfiguration('use_simulation')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_simulation',
            default_value='false',
            description='Use open-loop wheel simulation. false = real ESP32/micro-ROS feedback.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(controller_launch)),
            launch_arguments={'use_simulation': use_simulation}.items(),
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[str(ekf_config)],
        ),
    ])
