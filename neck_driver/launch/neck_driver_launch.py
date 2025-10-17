from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='neck_driver',
            executable='main',
            name='neck_driver',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('neck_driver'), 'config', 'neck_driver.yaml'])
            ]
        )
    ])
