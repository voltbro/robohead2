from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='silero_tts',
            executable='silero_tts_node',
            name='silero_tts_node',
            output='screen',
            namespace='silero_tts',
            parameters=[PathJoinSubstitution([
                FindPackageShare('silero_tts'),
                'config', 'silero_tts.yaml'
            ])]
        )
    ])