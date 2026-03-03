from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Найдём путь к пакету robohead_controller (для config-файлов)
    pkg_robohead_controller = FindPackageShare('robohead_controller')

    return LaunchDescription([
        # Группа с пространством имён "robohead_controller"
        GroupAction(
            actions=[
                PushRosNamespace('robohead'),

                # === media_driver ===
                Node(
                    package='media_driver',
                    executable='media_driver_node',  # ← должно быть объявлено в setup.py как console_script
                    name='media_driver_node',
                    output='screen',
                    namespace='media_driver',
                    parameters=[
                        PathJoinSubstitution([pkg_robohead_controller, 'config','media_driver.yaml'])
                    ]
                ),

                # # # === neck_driver ===
                Node(
                    package='neck_driver',
                    executable='neck_driver_node',
                    name='neck_driver',
                    output='screen',
                    namespace='neck_driver',
                    parameters=[PathJoinSubstitution([
                        FindPackageShare('robohead_controller'), 'config', 'neck_driver.yaml'])
                    ]
                ),

                # # # === sensor_driver ===
                Node(
                    package='sensor_driver',
                    executable='sensor_driver_node',
                    name='sensor_driver',
                    output='screen',
                    namespace='sensor_driver',
                    parameters=[ PathJoinSubstitution([ FindPackageShare('robohead_controller'), 'config', 'sensor_driver.yaml' ]) ]
                ),

                # # === ears_driver ===
                Node(
                    package='ears_driver',
                    executable='ears_driver_node',
                    name='ears_driver',
                    output='screen',
                    namespace='ears_driver',
                    parameters=[PathJoinSubstitution([
                        FindPackageShare('robohead_controller'), 'config', 'ears_driver.yaml'])
                    ]
                ),

                # # # === usb_cam ===
                Node(
                    package='usb_cam',
                    executable='usb_cam_node_exe',  # ← стандартное имя в ROS 2 usb_cam
                    name='usb_cam',
                    output='screen',
                    # arguments=['--ros-args', '--log-level', 'FATAL'],
                    namespace='usb_cam',
                    parameters=[{
                            'video_device': '/dev/video0',
                            'image_width': 640,
                            'image_height': 480,
                            'framerate': 30.0,
                            'pixel_format': 'mjpeg2rgb',      # ← MJPEG вместо yuyv, mjpeg2rgb, raw_mjpeg, yuyv2rgb
                            # 'color_format': 'rgb24',
                            'io_method': 'mmap',
                            # 'camera_name': 'default_cam',
                            'frame_id': 'front_camera',
                            'camera_info_url': '',        # ← Отключаем калибровку
                            # ПОДДЕРЖИВАЕМЫЕ параметры (проверьте через `v4l2-ctl --list-ctrls`):
                            # 'brightness': 50,             # ← Только если поддерживается
                            # 'contrast': 50,               # ← Только если поддерживается
                            # 'saturation': 50,             # ← Только если поддерживается
                            # УБРАТЬ неподдерживаемые:
                            # 'white_balance_temperature_auto': 1,  # ← УДАЛИТЬ
                            # 'exposure_auto': 3,                   # ← УДАЛИТЬ
                            # 'focus_auto': 0,                      # ← УДАЛИТЬ
                    }]
                ),

                # # === respeaker_driver ===
                Node(
                    package='respeaker_driver',
                    executable='respeaker_driver_node',
                    name='respeaker_driver',
                    output='screen',
                    namespace='respeaker_driver',
                    parameters=[PathJoinSubstitution([
                        FindPackageShare('robohead_controller'), 'config', 'respeaker_driver.yaml'])
                    ]
                ),

                # # === speech_recognizer ===
                # # Предполагается, что у вас один узел speech_recognizer с режимами
                GroupAction(
                    actions=[
                        PushRosNamespace('speech_recognizer'),

                        Node(
                            package='speech_recognizer',
                            executable='speech_recognizer_kws_node',
                            name='speech_recognizer_kws_node',
                            output='screen',
                            namespace='kws',
                            parameters=[PathJoinSubstitution([
                                FindPackageShare('robohead_controller'),
                                'config', 'speech_recognizer_kws.yaml'
                            ])]
                        ),

                        Node(
                            package='speech_recognizer',
                            executable='speech_recognizer_asr_node',
                            name='speech_recognizer_asr_node',
                            output='screen',
                            namespace='asr',
                            parameters=[PathJoinSubstitution([
                                FindPackageShare('robohead_controller'),
                                'config', 'speech_recognizer_asr.yaml'
                            ])]
                        )
                    ]
                )
            ]
        )
    ])