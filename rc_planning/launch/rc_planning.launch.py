import os
from pathlib import Path
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc_planning')
    launch_dir = os.path.join(bringup_dir, 'launch')

    imu_config = Path(get_package_share_directory(
        'rc_planning'), 'config', 'imu.yaml')

    robot_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'ekf.launch.py')),
    )

    robot_mode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'robot.launch.py'))
    )

    base_to_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_link',
        arguments=['0', '0', '0', '0', '0',
                   '0', 'base_footprint', 'base_link'],
    )
    base_to_gyro = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gyro',
        arguments=['0', '0', '0', '0', '0',
                   '0', 'base_footprint', 'gyro_link'],
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_config]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    rc_planning = Node(package='rc_planning',
                       executable='rc_planning',
                       output='screen',
                       namespace='',
                       parameters=[{
                           'receiver_interface': 'can0',
                           'sender_interface': 'can0',
                           'use_bus_time': False,
                           'interval_sec': 0.01,
                           'timeout_sec': 0.01,
                           'filters': '0:0',
                           'odom_frame_id': 'odom_combined',
                           'robot_frame_id': 'base_footprint',
                           'imu_frame_id': 'gyro_link'
                       }])

    ld = LaunchDescription()

    ld.add_action(base_to_link)
    ld.add_action(base_to_gyro)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(imu_filter_node)
    ld.add_action(robot_ekf)
    ld.add_action(rc_planning)
    ld.add_action(robot_mode)

    return ld
