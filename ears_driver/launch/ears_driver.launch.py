from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ears_driver',
            executable='ears_driver_node',
            name='ears_driver',
            output='screen',
            namespace='ears_driver',
            parameters=[PathJoinSubstitution([
                FindPackageShare('ears_driver'), 'config', 'ears_driver.yaml'])
            ]
        )
    ])
