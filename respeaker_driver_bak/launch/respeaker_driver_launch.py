from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='respeaker_driver',
            executable='main',
            name='respeaker_driver',
            parameters=[PathJoinSubstitution([
                FindPackageShare('respeaker_driver'), 'config', 'respeaker_driver.yaml'])
            ],
        ),
    ])