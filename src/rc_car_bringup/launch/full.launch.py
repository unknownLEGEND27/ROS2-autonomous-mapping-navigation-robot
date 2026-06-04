"""
Full launch — base + robot_state_publisher with URDF.
LiDAR and SLAM nodes can be added here later.

Usage:
    ros2 launch rc_car_bringup full.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Base nodes
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rc_car_bringup'),
                'launch', 'base.launch.py'
            )
        )
    )

    # Process URDF via xacro
    xacro_file = os.path.join(
        get_package_share_directory('rc_car_description'),
        'urdf', 'rc_car.urdf.xacro'
    )
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
    )

    return LaunchDescription([
        base_launch,
        robot_state_pub,
        joint_state_pub,
    ])
