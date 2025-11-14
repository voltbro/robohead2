from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Найдём путь к пакету gyrobro_controller (для config-файлов)
    pkg_gyrobro_controller = FindPackageShare('gyrobro_controller')

    return LaunchDescription([
        # Группа с пространством имён "gyrobro_controller"
        GroupAction(
            actions=[
                PushRosNamespace('robohead_controller'),

                # === media_driver ===
                Node(
                    package='media_driver',
                    executable='main',  # ← должно быть объявлено в setup.py как console_script
                    name='media_driver',
                    output='screen',
                    parameters=[
                        PathJoinSubstitution([
                            pkg_gyrobro_controller,
                            'config',
                            'media_driver.yaml'
                        ])
                    ]
                ),

                # # === neck_driver ===
                Node(
                    package='neck_driver',
                    executable='main',
                    name='neck_driver',
                    output='screen',
                    parameters=[
                        PathJoinSubstitution([
                            pkg_gyrobro_controller,
                            'config',
                            'neck_driver.yaml'
                        ])
                    ]
                ),

                # # === sensor_driver ===
                Node(
                    package='sensor_driver',
                    executable='main',
                    name='sensor_driver',
                    output='screen',
                    parameters=[
                        PathJoinSubstitution([
                            pkg_gyrobro_controller,
                            'config',
                            'sensor_driver.yaml'
                        ])
                    ]
                ),

                # === ears_driver ===
                Node(
                    package='ears_driver',
                    executable='main',
                    name='ears_driver',
                    output='screen',
                    parameters=[
                        PathJoinSubstitution([
                            pkg_gyrobro_controller,
                            'config',
                            'ears_driver.yaml'
                        ])
                    ]
                ),

                # # === usb_cam ===
                Node(
                    package='usb_cam',
                    executable='usb_cam_node_exe',  # ← стандартное имя в ROS 2 usb_cam
                    name='usb_cam',
                    output='screen',
                    parameters=[{
                        # 'video_device': '/dev/video0',
                        # 'image_width': 640,
                        # 'image_height': 480,
                        # 'framerate': 25.0,
                        # 'pixel_format': 'mjpeg',
                        # 'color_format': 'rgb8',
                        # 'io_method': 'mmap'
                    }]
                ),

                # === respeaker_driver ===
                Node(
                    package='respeaker_driver',
                    executable='main',
                    name='respeaker_driver',
                    output='screen',
                    parameters=[
                        PathJoinSubstitution([
                            pkg_gyrobro_controller,
                            'config',
                            'respeaker_driver.yaml'
                        ])
                    ]
                ),

                # === speech_recognizer ===
                # Предполагается, что у вас один узел speech_recognizer с режимами
                Node(
                    package='speech_recognizer',
                    executable='main',
                    name='speech_recognizer',
                    output='screen',
                    parameters=[
                        PathJoinSubstitution([
                            pkg_gyrobro_controller,
                            'config',
                            'speech_recognizer.yaml'  # ← объединённый конфиг
                        ])
                    ]
                ),
            ]
        )
    ])