from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='media_driver',
            executable='touch_publisher_node',
            name='touch_publisher_node',
            namespace='media_driver',
            parameters=[PathJoinSubstitution([
                FindPackageShare('media_driver'), 'config', 'media_driver.yaml'])
            ],
        ),
    ])