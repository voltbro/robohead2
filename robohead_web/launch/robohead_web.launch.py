from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8080'),
        Node(
            package='robohead_web',
            executable='server',
            name='robohead_web',
            output='screen',
            parameters=[{'host': LaunchConfiguration('host'), 'port': LaunchConfiguration('port')}],
        ),
    ])
