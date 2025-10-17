from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_driver',
            executable='main',
            name='sensor_driver',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('sensor_driver'), 'config', 'sensor_driver.yaml'])
            ]
        )
    ])
