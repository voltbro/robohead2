from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='speakers_driver',
            executable='main',
            name='speakers_driver',
            parameters=[PathJoinSubstitution([
                FindPackageShare('speakers_driver'), 'config', 'speakers_driver.yaml'])
            ],
        ),
    ])