from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='speech_recognizer',
            executable='main',
            name='speech_recognizer',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('speech_recognizer'), 'config', 'speech_recognizer.yaml'])
            ]
        )
    ])
