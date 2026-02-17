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


# from .drivers.ears_driver import EarsDriverConnector
# ... импорт других драйверов ...
# from .core.actions import ActionManager
# from .core.battery_monitor import BatteryMonitor
# from .core.speech_handler import SpeechHandler

class RoboheadController(Node):
    def __init__(self):
        super().__init__('robohead_controller')
        
        # Загрузка параметров
        # self.declare_parameter('low_voltage_threshold', 7.0)
        self.low_voltage_threshold = self.declare_parameter('low_voltage_threshold', 0.0).value
        self.low_voltage_hysteresis = self.declare_parameter('low_voltage_hysteresis', 0.0).value
        self.wait_timeout = self.declare_parameter('wait_timeout', 1.0).value



        # self.declare_parameter('low_voltage_hysteresis', 0.5)
        # self.declare_parameter('actions_match', "{}")
        
        # Инициализация компонентов
        # self.action_manager = ActionManager(self)
        
        # self.action_manager.load_actions(actions_str)
        
        # self.battery_monitor = BatteryMonitor(
        #     self,
        #     self.get_parameter('low_voltage_threshold').value,
        #     self.get_parameter('low_voltage_hysteresis').value
        # )
        
        # self.speech_handler = SpeechHandler(self, self.action_manager, self.battery_monitor)
        
        # Инициализация драйверов
        self.media_driver = MediaDriverConnector(self)
        self.ears_driver = EarsDriverConnector(self)
        self.neck_driver = NeckDriverConnector(self)
        self.sensor_driver = SensorDriverConnector(self)
        self.respeaker_driver = RespeakerDriverConnector(self)
        self.speech_recognizer_asr = SpeechRecognizerAsrConnector(self)
        self.speech_recognizer_kws = SpeechRecognizerKwsConnector(self)
        self.usb_cam = UsbCamConnector(self)
        
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
        
        self.get_logger().info("All drivers connected successfully")
    
    def start(self):
        """Запуск основной логики после подключения"""
        # Пример: воспроизведение приветственного видео
        self.media_driver.play_media(
            video_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/hello.mp4",
            audio_path="",
            loop=True
        )
        self.media_driver.play_media(
            video_path="",
            audio_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/start.mp3",
            loop=False
        )

        self.ears_driver.set_angle(-90, 30)
        self.speech_recognizer_kws.set_mode(1)

        # Активация распознавания ключевых слов
        # self.speech_handler.enable_wake_word_detection()
        # self.battery_monitor.is_allow_work = True
        
        self.get_logger().info("Controller started and ready")