from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='speech_recognizer',
            executable='speech_recognizer_node',
            name='speech_recognizer',
            output='screen',
            namespace='speech_recognizer',
            parameters=[PathJoinSubstitution([
                FindPackageShare('speech_recognizer'),
                'config', 'speech_recognizer.yaml'
            ])]
        )
    ])