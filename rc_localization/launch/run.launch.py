from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile
import os
import yaml


def generate_launch_description():

    localization_dir = get_package_share_directory('rc_localization')
    
    config_path = os.path.join(localization_dir, 'config', 'config.yaml')

    with open(config_path, 'r') as file:
        config_param = yaml.safe_load(file)['localization_node']['ros__parameters']

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='mid360',
            description='localization algorithm type: mid360'
        ),
        Node(
            package='rc_localization',
            executable='localization_node',
            name='l_node',
            output='screen',
            parameters=[
                config_param
            ],
            condition=LaunchConfigurationEquals('model', '2d')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('fast_lio_localization_sc_qn'),
                    'launch',
                    'run.launch.py'
                ])
            ]),
            condition=LaunchConfigurationEquals('model', 'mid360'),
            launch_arguments={'rviz': 'true',
                              'lidar': 'mid360'}.items()
        )
    ])