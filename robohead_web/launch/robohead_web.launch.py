from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from launch.actions import SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    """Return a ROS2 launch description that starts the web server node."""

    config_dir = PathJoinSubstitution([
        FindPackageShare('robohead_controller'),
        'config'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8080'),

        SetEnvironmentVariable('ROBOHEAD_CONFIG_DIR', config_dir),

        Node(
            package='robohead_web',
            executable='server',
            name='robohead_web',
            output='screen',
            parameters=[{'host': LaunchConfiguration('host'), 'port': LaunchConfiguration('port')}],
        ),
    ])
