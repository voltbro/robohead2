from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Найдём путь к пакету robohead_controller (для config-файлов)
    pkg_robohead_controller = FindPackageShare('turtlebro_controller')

    return LaunchDescription([
        # Группа с пространством имён "robohead_controller"
        GroupAction(
            actions=[
                PushRosNamespace('robohead'),

                # === silero_tts ===
                Node(
                    package='silero_tts',
                    executable='silero_tts_node',  # ← должно быть объявлено в setup.py как console_script
                    name='silero_tts_node',
                    output='screen',
                    namespace='silero_tts',
                    parameters=[
                        PathJoinSubstitution([pkg_robohead_controller, 'config','silero_tts.yaml'])
                    ]
                ),

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
                Node(
                    package='media_driver',
                    executable='touch_publisher_node',
                    name='touch_publisher_node',
                    namespace='media_driver',
                    output='screen',
                    parameters=[PathJoinSubstitution([
                        pkg_robohead_controller, 'config', 'media_driver.yaml'])
                    ],
                ),

                # # # === neck_driver ===
                Node(
                    package='neck_driver',
                    executable='neck_driver_node',
                    name='neck_driver',
                    output='screen',
                    namespace='neck_driver',
                    parameters=[PathJoinSubstitution([
                        FindPackageShare('turtlebro_controller'), 'config', 'neck_driver.yaml'])
                    ]
                ),

                # # # === sensor_driver ===
                Node(
                    package='sensor_driver',
                    executable='sensor_driver_node',
                    name='sensor_driver',
                    output='screen',
                    namespace='sensor_driver',
                    parameters=[ PathJoinSubstitution([ FindPackageShare('turtlebro_controller'), 'config', 'sensor_driver.yaml' ]) ]
                ),

                # # === ears_driver ===
                Node(
                    package='ears_driver',
                    executable='ears_driver_node',
                    name='ears_driver',
                    output='screen',
                    namespace='ears_driver',
                    parameters=[PathJoinSubstitution([
                        FindPackageShare('turtlebro_controller'), 'config', 'ears_driver.yaml'])
                    ]
                ),

                # # # === usb_cam ===
                Node(
                    package='usb_cam',
                    executable='usb_cam_node_exe',
                    name='usb_cam',
                    output='screen',
                    namespace='usb_cam',
                    parameters=[{
                            'video_device': '/dev/video0',
                            'image_width': 640,
                            'image_height': 480,
                            'framerate': 30.0,
                            'pixel_format': 'mjpeg2rgb',
                            'io_method': 'mmap',
                            'frame_id': 'front_camera',
                            'camera_info_url': '',
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
                        FindPackageShare('turtlebro_controller'), 'config', 'respeaker_driver.yaml'])
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
                                FindPackageShare('turtlebro_controller'),
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
                                FindPackageShare('turtlebro_controller'),
                                'config', 'speech_recognizer_asr.yaml'
                            ])]
                        )
                    ]
                )
            ]
        )
    ])