import rclpy
from rclpy.node import Node
from .drivers.media_driver import MediaDriverConnector
from .drivers.ears_driver import EarsDriverConnector
from .drivers.neck_driver import NeckDriverConnector
from .drivers.sensor_driver import SensorDriverConnector
from .drivers.respeaker_driver import RespeakerDriverConnector
from .drivers.speech_recognizer_asr import SpeechRecognizerAsrConnector
from .drivers.speech_recognizer_kws import SpeechRecognizerKwsConnector
from .drivers.usb_cam import UsbCamConnector
from .drivers.silero_tts import SileroTTSConnector

from .core.battery_monitor import BatteryMonitor
from .core.action_manager import ActionManager
from .core.commander import Commander

import json
import os
import time

class RoboheadController(Node):
    def __init__(self):
        super().__init__('robohead_controller')
        
        # Загрузка параметров
        self.low_voltage_threshold = self.declare_parameter('low_voltage_threshold', 0.0).value
        self.low_voltage_hysteresis = self.declare_parameter('low_voltage_hysteresis', 0.0).value
        self.wait_timeout = self.declare_parameter('wait_timeout', 1.0).value
        self.actions_match = json.loads(self.declare_parameter('actions_match', "{}").value)
        package_dir = os.path.dirname(os.path.abspath(__file__))
        for key in self.actions_match.keys():
            self.actions_match[key] = os.path.join(package_dir, 'actions', self.actions_match[key])
        # self.get_logger().info(f"RoboheadController actions match: {self.actions_match}")
        # for key, val in self.actions_match.items():
        #     self.get_logger().info(f"{key} : {val}")

        self.is_allow_work = False
        self.action_manager = None
        self.queue_wake_phrases = list()
        self.queue_commands = list()
        self.queue_frees = list()
        self.queue_fast_commands = list()
        self.rate_ = self.create_rate(100)
        
        # Инициализация драйверов
        self.media_driver = MediaDriverConnector(self)
        self.ears_driver = EarsDriverConnector(self)
        self.neck_driver = NeckDriverConnector(self)
        self.sensor_driver = SensorDriverConnector(self)
        self.respeaker_driver = RespeakerDriverConnector(self)
        self.speech_recognizer_asr = SpeechRecognizerAsrConnector(self)
        self.speech_recognizer_kws = SpeechRecognizerKwsConnector(self)
        self.usb_cam = UsbCamConnector(self)
        self.silero_tts = SileroTTSConnector(self)

        # Инициализация компонентов
        self.action_manager = ActionManager(self)
        self.battery_monitor = BatteryMonitor(self)
        self.commander = Commander(self)
        
        self.get_logger().info("RoboheadController INITED")
    
    def connect_all_drivers(self):
        self.get_logger().info("Starting driver connections...")
        
        self.media_driver.connect()
        self.ears_driver.connect()
        self.neck_driver.connect()
        self.sensor_driver.connect()
        self.respeaker_driver.connect()
        self.speech_recognizer_asr.connect()
        self.speech_recognizer_kws.connect()
        self.usb_cam.connect()
        self.silero_tts.connect()
        
        self.get_logger().info("All drivers connected successfully")
    
    def sleep(self, cancel_event, duration:float):
        start_time = self.get_clock().now()

        while not cancel_event.is_set() and (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.rate_.sleep()
    
    def start(self):
        self.startup_timer.cancel()

        self.action_manager.execute_action("std_startup", None, False)

        self.speech_recognizer_kws.set_mode(1)

        self.get_logger().info("Controller started and ready")