from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_robohead_controller = FindPackageShare('robohead_controller_ai')

    return LaunchDescription([
        # 1. Запуск зависимостей (драйверов)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_robohead_controller,
                    'launch',
                    'dependencies.launch.py'
                ])
            ])
        ),

        # 2. Запуск основного контроллера с параметрами
        Node(
            package='robohead_controller_ai',
            executable='main',  # ← из entry_points в setup.py
            name='robohead_controller_ai_node',
            output='screen',
            namespace='/robohead/robohead_controller_ai',
            parameters=[
               PathJoinSubstitution([
                    pkg_robohead_controller,
                    'config',
                    'robohead_controller_ai.yaml'
                ]),
            ]
        ),
    ])