"""
Teleop launch — base nodes + keyboard teleop.
Remaps teleop output to /cmd_vel_raw so PID can process it.

Usage:
    ros2 launch rc_car_bringup teleop.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rc_car_bringup'),
                'launch', 'base.launch.py'
            )
        )
    )

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        remappings=[('/cmd_vel', '/cmd_vel_raw')],
        output='screen',
        prefix='xterm -e',   # opens in new terminal
    )

    return LaunchDescription([
        base_launch,
        teleop_node,
    ])
