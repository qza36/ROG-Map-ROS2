from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_name',
            default_value='config.yaml',
            description='Config file name in config directory'
        ),
        Node(
            package='rog_map',
            executable='rogastar',
            name='rog_astar_node',
            output='screen',
            parameters=[{
                'config_name': LaunchConfiguration('config_name')
            }]
        ),
    ])
