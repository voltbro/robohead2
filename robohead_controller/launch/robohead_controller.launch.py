from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_robohead_controller = FindPackageShare('robohead_controller')

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
            package='robohead_controller',
            executable='main',  # ← из entry_points в setup.py
            name='robohead_controller',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    pkg_robohead_controller,
                    'config',
                    'robohead_controller.yaml'
                ])
            ]
        ),
    ])