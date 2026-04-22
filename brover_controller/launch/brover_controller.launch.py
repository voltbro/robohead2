from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_brover_controller = FindPackageShare('brover_controller')

    return LaunchDescription([
        # 1. Запуск зависимостей (драйверов)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_brover_controller,
                    'launch',
                    'dependencies.launch.py'
                ])
            ])
        ),

        # 2. Запуск основного контроллера с параметрами
        Node(
            package='brover_controller',
            executable='main',  # ← из entry_points в setup.py
            name='brover_controller_node',
            output='screen',
            namespace='/robohead/brover_controller',
            parameters=[
               PathJoinSubstitution([
                    pkg_brover_controller,
                    'config',
                    'brover_controller.yaml'
                ]),
            ]
        ),
    ])