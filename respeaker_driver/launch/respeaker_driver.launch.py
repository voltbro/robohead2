from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='respeaker_driver',
            executable='respeaker_driver_node',
            name='respeaker_driver',
            output='screen',
            namespace='respeaker_driver',
            parameters=[PathJoinSubstitution([
                FindPackageShare('respeaker_driver'), 'config', 'respeaker_driver.yaml'])
            ]
        )
    ])
