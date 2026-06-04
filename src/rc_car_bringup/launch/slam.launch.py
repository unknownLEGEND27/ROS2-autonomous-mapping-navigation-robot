import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    motor_config = os.path.join(get_package_share_directory('rc_car_driver'), 'config', 'motor_params.yaml')
    imu_config = os.path.join(get_package_share_directory('rc_car_imu'), 'config', 'imu_params.yaml')
    pid_config = os.path.join(get_package_share_directory('rc_car_control'), 'config', 'pid_params.yaml')
    slam_config = os.path.join(get_package_share_directory('rc_car_bringup'), 'config', 'slam_params.yaml')

    return LaunchDescription([
        Node(package='rc_car_driver', executable='motor_node', name='motor_node', parameters=[motor_config], output='screen'),
        Node(package='rc_car_imu', executable='imu_node', name='imu_node', parameters=[imu_config], output='screen'),
        Node(package='rc_car_control', executable='pid_node', name='pid_node', parameters=[pid_config], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py')), launch_arguments={'serial_baudrate': '460800', 'serial_port': '/dev/ttyUSB0'}.items()),
        Node(package='tf2_ros', executable='static_transform_publisher', name='static_tf_laser', arguments=['0','0','0.1','0','0','0','base_link','laser'], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='static_tf_imu', arguments=['0','0','0.03','0','0','0','base_link','imu_link'], output='screen'),
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node', name='rf2o_laser_odometry', output='screen', remappings=[('/laser_scan','/scan')], parameters=[{'laser_scan_topic':'/scan','odom_topic':'/odom','publish_tf':True,'base_frame_id':'base_link','odom_frame_id':'odom','init_pose_from_topic':'','freq':10.0}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')), launch_arguments={'slam_params_file': slam_config}.items()),
    ])
