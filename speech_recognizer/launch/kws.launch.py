from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='speech_recognizer',
            executable='speech_recognizer_kws_node',
            name='speech_recognizer_kws_node',
            output='screen',
            namespace='kws',
            parameters=[PathJoinSubstitution([
                FindPackageShare('speech_recognizer'),
                'config', 'speech_recognizer_kws.yaml'
            ])]
        )
    ])