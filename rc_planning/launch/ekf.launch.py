import os
from pathlib import Path
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    ekf_config = Path(get_package_share_directory(
        'rc_planning'), 'config', 'ekf.yaml')
    ekf_carto_config = Path(get_package_share_directory(
        'rc_planning'), 'config', 'ekf_carto.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='carto_ekf_filter_node',
            parameters=[ekf_carto_config],
            remappings=[('/odometry/filtered', 'odom_combined')]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config],
            remappings=[('/odometry/filtered', 'odom_combined')]
        ),
    ])
