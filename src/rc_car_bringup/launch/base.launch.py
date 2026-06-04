"""
Base launch — starts motor driver, IMU, and PID controller.
This is the minimum to get the car moving with heading correction.

Usage:
    ros2 launch rc_car_bringup base.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motor_config = os.path.join(
        get_package_share_directory('rc_car_driver'),
        'config', 'motor_params.yaml'
    )
    imu_config = os.path.join(
        get_package_share_directory('rc_car_imu'),
        'config', 'imu_params.yaml'
    )
    pid_config = os.path.join(
        get_package_share_directory('rc_car_control'),
        'config', 'pid_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='rc_car_driver',
            executable='motor_node',
            name='motor_node',
            parameters=[motor_config],
            output='screen',
        ),
        Node(
            package='rc_car_imu',
            executable='imu_node',
            name='imu_node',
            parameters=[imu_config],
            output='screen',
        ),
        Node(
            package='rc_car_control',
            executable='pid_node',
            name='pid_node',
            parameters=[pid_config],
            output='screen',
        ),
    ])
