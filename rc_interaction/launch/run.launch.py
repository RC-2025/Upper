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

    interaction_dir = get_package_share_directory('rc_interaction')
    
    sbus_config_path = os.path.join(interaction_dir, 'config', 'sbus.yaml')
    xbox_config_path = os.path.join(interaction_dir, 'config', 'xbox.yaml')


    with open(sbus_config_path, 'r') as file:
        sbus_param = yaml.safe_load(file)['sbus_node']['ros__parameters']
    with open(xbox_config_path, 'r') as file:
        xbox_param = yaml.safe_load(file)['xbox_node']['ros__parameters']

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='sbus',
            description='model type: sbus,xbox'
        ),
        Node(
            package='rc_interaction',
            executable='sbus_node',
            name='sbus_node',
            output='screen',
            parameters=[
                sbus_param
            ],
            condition=LaunchConfigurationEquals('model', 'sbus')
        ),
        Node(
            package='rc_interaction',
            executable='xbox_node',
            name='xbox_node',
            output='screen',
            parameters=[
                xbox_param
            ],
            condition=LaunchConfigurationEquals('model', 'xbox')
        ),
    ])
