#!/usr/bin/env python3

import os
import rclpy
import importlib.util
# import asyncio
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from robohead_interfaces.msg import AudioData, ColorArray
from robohead_interfaces.srv import Color, ColorPalette, Move, PlayMedia, SimpleCommand
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from sensor_msgs.msg import Image
import json
import sys
import threading
# from ament_index_python.packages import get_package_share_directory

import time

# PACKAGE_SOURCE_DIR = os.path.abspath(os.path.dirname(__file__))
# --- Динамические импорты будут ниже ---
sys.path.append('/home/pi/robohead_ws/src/robohead2/gyrobro_controller/actions')

class RoboheadController(Node):

    def __init__(self):
        super().__init__('robohead_controller')

        # Параметры
        self.declare_parameter('low_voltage_threshold', 7.0)
        self.declare_parameter('low_voltage_hysteresis', 0.5)
        self.declare_parameter('actions_match', "{}")
        # self.declare_parameter('actions_match')
        # print("BROOOOO: ", self.list_parameters([],depth=10))

        self._current_action_cancel = None  # threading.Event or None
        self._action_lock = threading.Lock()

        self.low_voltage_threshold = self.get_parameter('low_voltage_threshold').value
        self.low_voltage_hysteresis = self.get_parameter('low_voltage_hysteresis').value
        self.is_allow_work = True
        actions_str:str = self.get_parameter('actions_match').value
        self.get_logger().warn(f"---------------------------------- action_match: \n{actions_str}")
        
        self.action_paths:dict = json.loads(actions_str)
        # print("Action_math UUUUUUUUU: ",actions_match)

        # self.action_paths = self.get_action_paths(actions_match)
        # print("Action_paths UUUUUUUUU: ",self.action_paths)

        # Инициализация переменных состояния
        self.display_driver_touchscreen_xy = (0.0, 0.0)
        self.sensor_driver_bat_voltage = 8.42
        self.sensor_driver_bat_current = -2.1
        self.respeaker_driver_doa_angle = 0
        self.respeaker_driver_msg_audio_main = None
        self.usb_cam_image_raw = None
        self.gyrobro_pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel_external", 1)

        # self._init_timer = self.create_timer(0.01, self.connect)

    def connect(self):
        # self._init_timer.cancel()
          # --- Подключение к драйверам ---
        self.get_logger().info("gyrobro_controller: start_connect")
        self._connect_media_driver()
        self.get_logger().info("gyrobro_controller: media_driver connected")

        self._connect_ears_driver()
        self.get_logger().warn("gyrobro_controller: ears_driver connected")

        self._connect_neck_driver()
        self.get_logger().warn("gyrobro_controller: neck_driver connected")

        self._connect_sensor_driver()
        self.get_logger().warn("gyrobro_controller: sensor_driver connected")

        self._connect_respeaker_driver()
        self.get_logger().warn("gyrobro_controller: respeaker_driver connected")


        self._connect_speech_recognizer()
        self.get_logger().warn("gyrobro_controller: speech_recognizer connected")

        self._connect_usb_cam()
        self.get_logger().warn("gyrobro_controller: usb_cam connected")



        self.get_logger().warn("gyrobro_controller: all packages connected")

    # def get_action_paths(self, actions_match:dict) -> dict:
    #     # Получаем путь к пакету
    #     pkg_share = get_package_share_directory('gyrobro_controller')
    #     self.actions_dir = os.path.join(pkg_share, 'actions')  # ← папка actions рядом с setup.py

    #     # Загружаем маппинг
    #     # self.actions_match = self.get_parameter('gyrobro_controller_actions_match').value

    #     # Создаём словарь: имя действия -> полный путь к action.py
    #     action_paths = {}
    #     for cmd_name, folder_name in actions_match.items():
    #         action_path = os.path.join(self.actions_dir, folder_name, 'action.py')
    #         if not os.path.exists(action_path):
    #             self.get_logger().error(f"Action file not found: {action_path}")
    #             continue
    #         action_paths[cmd_name] = action_path
    #     return action_paths

    
  

    def _connect_media_driver(self):
        # Получаем параметры
        self.declare_parameter('~/media_driver/play_media', "~/media_driver/play_media")
        # self.declare_parameter('~display_driver/topic_PlayMedia_name', "~/media_driver/play_media")

        service_name_play_media = self.get_parameter('~/media_driver/play_media').value
        # topic_name = self.get_parameter('~display_driver/topic_PlayMedia_name').value
        # touchscreen_topic = self.get_parameter('~display_driver/topic_touchscreen_name').value

        # Создаем клиент и публикатор
        self.media_driver_srv_play_media = self.create_client(PlayMedia, service_name_play_media)

        self.media_driver_pub_stream = self.create_publisher(Image, "/robohead_controller/media_driver/stream", 1)
        # self.display_driver_sub_touchscreen = self.create_subscription(
            # Pose2D, touchscreen_topic, self._display_driver_touchsreen_callback, 10
        # )

        # Ждем сервиса
        while not self.media_driver_srv_play_media.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service %s not available, waiting...' % service_name_play_media)

        # Запускаем начальный медиафайл
        msg = PlayMedia.Request()
        msg.path_to_media_file = '/home/pi/robohead_ws/src/robohead2/gyrobro_controller/gyrobro_controller/loading_splash.mp4'
        msg.is_block = False
        msg.is_cycle = True
        self.get_logger().info('Service CALL START' )
        future = self.media_driver_srv_play_media.call_async(msg)
     
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        response = future.result()
        self.get_logger().info('Service CALL END' )


    def _connect_ears_driver(self):
        self.declare_parameter('srv_ears_set_angle_name', '/robohead_controller/ears_driver/ears_set_angle')
        service_name = self.get_parameter('srv_ears_set_angle_name').value


        self.get_logger().info(f"Ears driver connect: {service_name}")
        self.ears_driver_srv_ears_set_angle = self.create_client(Move, service_name)

        while not self.ears_driver_srv_ears_set_angle.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Ears service not available, waiting...')
        self.get_logger().info("Ears driver connected")

    def _connect_neck_driver(self):
        self.declare_parameter('~neck_driver/srv_neck_set_angle_name', '/robohead_controller/neck_driver/neck_set_angle')
        service_name = self.get_parameter('~neck_driver/srv_neck_set_angle_name').value
        self.neck_driver_srv_neck_set_angle = self.create_client(Move, service_name)
        while not self.neck_driver_srv_neck_set_angle.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Neck service not available, waiting...')
        self.get_logger().info("Neck driver connected")



    def _connect_sensor_driver(self):
        self.declare_parameter('~sensor_driver/topic_name','/robohead_controller/sensor_driver/battery')
        topic_name = self.get_parameter('~sensor_driver/topic_name').value
        self.sensor_driver_sub_battery = self.create_subscription(BatteryState, topic_name, self._sensor_driver_battery_callback, 10)
        # Ждем первого сообщения
        self.get_logger().info("Waiting for first battery message...")
        # msg = self._wait_for_message(BatteryState, topic_name, timeout_sec=5.0)
        # if msg:
        #     self.get_logger().info("Sensor driver connected")
        # else:
        #     self.get_logger().error("No battery message received")

    def _connect_respeaker_driver(self):
        # Топики аудио
        self.declare_parameter('~respeaker_driver/ros/topic_name/audio_main')
        self.declare_parameter('~respeaker_driver/ros/topic_name/audio_channel_0')
        self.declare_parameter('~respeaker_driver/ros/topic_name/audio_channel_1')
        self.declare_parameter('~respeaker_driver/ros/topic_name/audio_channel_2')
        self.declare_parameter('~respeaker_driver/ros/topic_name/audio_channel_3')
        self.declare_parameter('~respeaker_driver/ros/topic_name/audio_channel_4')
        self.declare_parameter('~respeaker_driver/ros/topic_name/audio_channel_5')

        topic_name_audio_main = self.get_parameter('~respeaker_driver/ros/topic_name/audio_main').value
        topic_name_audio_channel_0 = self.get_parameter('~respeaker_driver/ros/topic_name/audio_channel_0').value
        topic_name_audio_channel_1 = self.get_parameter('~respeaker_driver/ros/topic_name/audio_channel_1').value
        topic_name_audio_channel_2 = self.get_parameter('~respeaker_driver/ros/topic_name/audio_channel_2').value
        topic_name_audio_channel_3 = self.get_parameter('~respeaker_driver/ros/topic_name/audio_channel_3').value
        topic_name_audio_channel_4 = self.get_parameter('~respeaker_driver/ros/topic_name/audio_channel_4').value
        topic_name_audio_channel_5 = self.get_parameter('~respeaker_driver/ros/topic_name/audio_channel_5').value

        # doa_topic = self.get_parameter('~respeaker_driver/ros/topic_doa_angle_name').value

        # Сервисы LED
        self.declare_parameter('~respeaker_driver/ros/service_name/set_brightness')
        self.declare_parameter('~respeaker_driver/ros/service_name/set_color_all')
        self.declare_parameter('~respeaker_driver/ros/service_name/set_color_palette')
        self.declare_parameter('~respeaker_driver/ros/service_name/set_mode')
        self.declare_parameter('~respeaker_driver/ros/topic_name/set_color_manual')

        service_name_set_brightness = self.get_parameter('~respeaker_driver/ros/service_name/set_brightness').value
        service_name_set_color_all = self.get_parameter('~respeaker_driver/ros/service_name/set_color_all').value
        service_name_set_color_palette = self.get_parameter('~respeaker_driver/ros/service_name/set_color_palette').value
        service_name_set_mode = self.get_parameter('~respeaker_driver/ros/service_name/set_mode').value
        topic_name_set_color_manual = self.get_parameter('~respeaker_driver/ros/topic_name/set_color_manual').value

        # Подписки
        # self.respeaker_driver_sub_audio_main = self.create_subscription(AudioData, topic_name_audio_main, self._respeaker_driver_audio_main_callback, 10)
        # self.respeaker_driver_sub_audio_channel_0 = self.create_subscription(AudioData, topic_name_audio_channel_0, self._respeaker_driver_audio_channel_0_callback, 10)
        # self.respeaker_driver_sub_audio_channel_1 = self.create_subscription(AudioData, topic_name_audio_channel_1, self._respeaker_driver_audio_channel_1_callback, 10)
        # self.respeaker_driver_sub_audio_channel_2 = self.create_subscription(AudioData, topic_name_audio_channel_2, self._respeaker_driver_audio_channel_2_callback, 10)
        # self.respeaker_driver_sub_audio_channel_3 = self.create_subscription(AudioData, topic_name_audio_channel_3, self._respeaker_driver_audio_channel_3_callback, 10)
        # self.respeaker_driver_sub_audio_channel_4 = self.create_subscription(AudioData, topic_name_audio_channel_4, self._respeaker_driver_audio_channel_4_callback, 10)
        # self.respeaker_driver_sub_audio_channel_5 = self.create_subscription(AudioData, topic_name_audio_channel_5, self._respeaker_driver_audio_channel_5_callback, 10)

        # ... подписки на другие каналы
        # self.respeaker_driver_sub_doa_angle = self.create_subscription(Int16, doa_topic, self._respeaker_driver_doa_angle_callback, 10)

        # Публикатор
        # self.respeaker_driver_pub_set_color_manual = self.create_publisher(ColorArray, topic_name_set_color_manual, 1)

        # # Клиенты сервисов
        # self.respeaker_driver_srv_set_brightness = self.create_client(SimpleCommand, service_name_set_brightness)
        # self.respeaker_driver_srv_set_color_all = self.create_client(Color, service_name_set_color_all)
        # self.respeaker_driver_srv_set_color_palette = self.create_client(ColorPalette, service_name_set_color_palette)
        # self.respeaker_driver_srv_set_mode = self.create_client(SimpleCommand, service_name_set_mode)

        # # Ждем сервисов
        # services = [
        #     self.respeaker_driver_srv_set_brightness,
        #     self.respeaker_driver_srv_set_color_all,
        #     self.respeaker_driver_srv_set_color_palette,
        #     self.respeaker_driver_srv_set_mode
        # ]
        # for srv in services:
        #     while not srv.wait_for_service(timeout_sec=1.0):
        #         self.get_logger().info(f'Respeaker LED service {srv.srv_type} not available, waiting...')

        # Ждем сообщений
        # self._wait_for_message(AudioData, topic_name_audio_main, timeout_sec=5.0)
        # self._wait_for_message(AudioData, topic_name_audio_channel_0, timeout_sec=5.0)
        # self._wait_for_message(AudioData, topic_name_audio_channel_1, timeout_sec=5.0)
        # self._wait_for_message(AudioData, topic_name_audio_channel_2, timeout_sec=5.0)
        # self._wait_for_message(AudioData, topic_name_audio_channel_3, timeout_sec=5.0)
        # self._wait_for_message(AudioData, topic_name_audio_channel_4, timeout_sec=5.0)
        # self._wait_for_message(AudioData, topic_name_audio_channel_5, timeout_sec=5.0)
        # self._wait_for_message(AudioData, topic_name_doa, timeout_sec=5.0)

    def _connect_speech_recognizer(self):
        self.declare_parameter('~/speech_recognizer/ros/topic_name/wake_phrases', '/robohead_controller/speech_recognizer/wake_phrases')
        self.declare_parameter('~/speech_recognizer/ros/topic_name/commands', '/robohead_controller/speech_recognizer/commands')
        self.declare_parameter('~/speech_recognizer/ros/service_name/set_mode', '/robohead_controller/speech_recognizer/set_mode')
        self.get_logger().info("Voice recognizer connection: 1")
        
        topic_name_wake_phrases = self.get_parameter('~/speech_recognizer/ros/topic_name/wake_phrases').value
        topic_name_commands = self.get_parameter('~/speech_recognizer/ros/topic_name/commands').value
        service_name_set_mode = self.get_parameter('~/speech_recognizer/ros/service_name/set_mode').value 
        self.get_logger().info("Voice recognizer connection: 2")
        self.speech_recognizer_sub_wake_phrases = self.create_subscription(String, topic_name_wake_phrases, self._speech_recognizer_wake_phrases_callback, 10)
        self.speech_recognizer_sub_commands = self.create_subscription(String, topic_name_commands, self._speech_recognizer_commands_callback, 10)
        self.speech_recognizer_srv_set_mode = self.create_client(SimpleCommand, service_name_set_mode)

        self.get_logger().info("Voice recognizer connection: 3")

 
        while not self.speech_recognizer_srv_set_mode.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Voice recognizer service not available, waiting...')
        self.get_logger().info("Voice recognizer connection: 4")

        # Выключаем распознавание по умолчанию
        msg = SimpleCommand.Request()
        msg.data = 0 # off detection

        future = self.speech_recognizer_srv_set_mode.call_async(msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        response = future.result()

        # response = None
        # while rclpy.ok():
        #     rclpy.spin_once(self)
        #     if future.done():
        #         response = future.result()
        #         break


        self.get_logger().info("Voice recognizer connected")

    def _connect_usb_cam(self):
        self.declare_parameter('~/usb_cam/topic_name/image_raw', '/robohead_controller/image_raw')
        topic_name_image_raw = self.get_parameter('~/usb_cam/topic_name/image_raw').value
        self.usb_cam_sub_image_raw = self.create_subscription(Image, topic_name_image_raw, self._usb_cam_image_raw_callback, 10)
        # msg = self._wait_for_message(Image, topic_name_image_raw, timeout_sec=5.0)
        # if msg:
        #     self.get_logger().info("USB cam connected")
        # else:
        #     self.get_logger().error("No image from USB cam")
    def _on_action_complete(self):
        """Вызывается, когда действие завершилось успешно (не было прервано)."""
        self.get_logger().info("Action completed → starting std_wait")
        # Запускаем std_wait БЕЗ колбэка (иначе будет рекурсия)
        self._execute_action("std_wait", on_complete=None)

    def _execute_action(self, name: str, on_complete=None):
        """Запускает действие с возможностью прерывания и колбэком завершения."""
        with self._action_lock:
            if self._current_action_cancel is not None:
                self._current_action_cancel.set()

            self._current_action_cancel = threading.Event()

        try:
            action_path = self.action_paths.get(name)
            if not action_path:
                self.get_logger().error(f"Action '{name}' not found")
                return

            spec = importlib.util.spec_from_file_location(name, action_path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)

            if not hasattr(module, 'run'):
                self.get_logger().error(f"Action {name} has no 'run' function")
                return

            # Запускаем в потоке
            thread = threading.Thread(
                target=module.run,
                args=(self, name, self._current_action_cancel, on_complete),
                daemon=True
            )
            self.get_logger().error(f"RC: Action {name} start")
            thread.start()

        except Exception as e:
            self.get_logger().error(f"Failed to start action {name}: {e}")


    def _sensor_driver_battery_callback(self, msg:BatteryState):
        self.sensor_driver_battery_voltage = msg.voltage
        self.sensor_driver_battery_current = msg.current
        if self.sensor_driver_bat_voltage < self.low_voltage_threshold:
            self.is_allow_work = False
            # asyncio.create_task(self._execute_action('low_bat_action'))
            self._execute_action('low_bat_action')
            self.get_logger().error("Low voltage on battery!")

        elif not self.is_allow_work and self.sensor_driver_bat_voltage >= self.low_voltage_threshold + self.low_voltage_hysteresis:
            self.is_allow_work = True
            # asyncio.create_task(self._execute_action('wait_action'))
            self._execute_action('std_wait')
            

    def _respeaker_driver_audio_main_callback(self, msg):
        self.respeaker_driver_msg_audio_main = msg

    def _respeaker_driver_audio_channel_0_callback(self, msg):
        self.respeaker_driver_msg_audio_channel_0 = msg
    def _respeaker_driver_audio_channel_1_callback(self, msg):
        self.respeaker_driver_msg_audio_channel_1 = msg
    def _respeaker_driver_audio_channel_2_callback(self, msg):
        self.respeaker_driver_msg_audio_channel_2 = msg
    def _respeaker_driver_audio_channel_3_callback(self, msg):
        self.respeaker_driver_msg_audio_channel_3 = msg
    def _respeaker_driver_audio_channel_4_callback(self, msg):
        self.respeaker_driver_msg_audio_channel_4 = msg
    def _respeaker_driver_audio_channel_5_callback(self, msg):
        self.respeaker_driver_msg_audio_channel_5 = msg
    # ... остальные callbacks (doa)

    def _speech_recognizer_wake_phrases_callback(self, msg:String):
        self.get_logger().error(f"callback wake: {msg.data}")
        if not self.is_allow_work:
            return
        new_msg = SimpleCommand.Request()
        new_msg.data = 2 # commands recognition
        # self.speech_recognizer_srv_set_mode.call(msg)
        future = self.speech_recognizer_srv_set_mode.call_async(new_msg)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        # asyncio.create_task(self._execute_action(msg.data)
        self._execute_action(msg.data, None)

    def _speech_recognizer_commands_callback(self, msg:String):
        self.get_logger().error(f"callback command: {msg.data}")
        if not self.is_allow_work:
            return
        new_msg = SimpleCommand.Request()

        new_msg.data = 0 # off recognition
        # self.speech_recognizer_srv_set_mode.call(msg)
        future = self.speech_recognizer_srv_set_mode.call_async(new_msg)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        # asyncio.create_task(self._execute_action(msg.data))
        self._execute_action(msg.data, self._on_action_complete)
        # asyncio.create_task(self._execute_action('wait_action'))
        # self._execute_action('std_wait')
        new_msg.data = 1 # kws recognition
        # self.speech_recognizer_srv_set_mode.call(new_msg)
        future = self.speech_recognizer_srv_set_mode.call_async(new_msg)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

    def _usb_cam_image_raw_callback(self, msg:Image):
        self.usb_cam_image_raw = msg

    def start(self):
        """Запуск после инициализации"""

        

        script_path = os.path.dirname(os.path.abspath(__file__)) + '/'
        msg = PlayMedia.Request()
        msg.path_to_media_file = "/home/pi/robohead_ws/src/robohead2/gyrobro_controller/actions/std_wait/wait.png"
        msg.path_to_override_audio_file = "/home/pi/file.mp3"
        msg.is_block = True
        msg.is_cycle = False
        future = self.media_driver_srv_play_media.call_async(msg)

        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        response = future.result()

        # self._execute_action("std_wait")

        new_msg = SimpleCommand.Request()
        new_msg.data = 1 # on kws
        future = self.speech_recognizer_srv_set_mode.call_async(new_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)



def main(args=None):
    rclpy.init(args=args)
    node = RoboheadController()
    time.sleep(1)
    node.connect()
    
    # Запускаем основной цикл
    node.start()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    # rclpy.spin(node)
    # Запускаем асинхронный цикл в фоне
    
    rclpy.shutdown()
    # executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(node)

    # try:
    #     executor.spin()
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()

if __name__ == '__main__':
    main()