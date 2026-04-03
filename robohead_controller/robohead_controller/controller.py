from __future__ import annotations
from typing import TYPE_CHECKING, Optional, cast
import threading
import json
import os

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

if TYPE_CHECKING:
    from rclpy.timer import Timer, Rate

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


class RoboheadController(Node):
    def __init__(self):
        super().__init__("robohead_controller")

        self._low_voltage_threshold: float = cast(
            float, self.declare_parameter("low_voltage_threshold", 0.0).value
        )
        self._low_voltage_hysteresis: float = cast(
            float, self.declare_parameter("low_voltage_hysteresis", 0.0).value
        )
        self._wait_timeout: float = cast(
            float, self.declare_parameter("wait_timeout", 1.0).value
        )

        actions_match_raw: str = cast(
            str, self.declare_parameter("actions_match", "{}").value
        )
        self._actions_match: dict[str, str] = self._load_actions_match(
            actions_match_raw
        )

        self._is_allow_work: bool = False

        self.queue_wake_phrases: list[str] = []
        self.queue_commands: list[str] = []
        self.queue_frees: list[str] = []
        self.queue_fast_commands: list[str] = []

        self._rate: Rate = self.create_rate(100)
        self._reentrant_callback_group: ReentrantCallbackGroup = (
            ReentrantCallbackGroup()
        )

        self._startup_timer: Optional[Timer] = None

        self._media_driver: MediaDriverConnector = MediaDriverConnector(self)
        self._ears_driver: EarsDriverConnector = EarsDriverConnector(self)
        self._neck_driver: NeckDriverConnector = NeckDriverConnector(self)
        self._sensor_driver: SensorDriverConnector = SensorDriverConnector(self)
        self._respeaker_driver: RespeakerDriverConnector = RespeakerDriverConnector(
            self
        )
        self._speech_recognizer_asr: SpeechRecognizerAsrConnector = (
            SpeechRecognizerAsrConnector(self)
        )
        self._speech_recognizer_kws: SpeechRecognizerKwsConnector = (
            SpeechRecognizerKwsConnector(self)
        )
        self._usb_cam: UsbCamConnector = UsbCamConnector(self)
        self._silero_tts: SileroTTSConnector = SileroTTSConnector(self)

        self._action_manager: ActionManager = ActionManager(self)
        self._battery_monitor: BatteryMonitor = BatteryMonitor(self)
        self._commander: Commander = Commander(self)

        self.get_logger().info("RoboheadController INITED")

    def _load_actions_match(self, actions_match_raw: str) -> dict[str, str]:
        """
        Загружает и обрабатывает маппинг действий.

        Args:
            actions_match_raw (str): JSON-строка с маппингом действий

        Returns:
            dict[str, str]: словарь {имя_действия: полный_путь_к_файлу}
        """
        actions_match: dict[str, str] = json.loads(actions_match_raw)
        package_dir: str = os.path.dirname(os.path.abspath(__file__))

        for key in actions_match.keys():
            actions_match[key] = os.path.join(
                package_dir, "actions", actions_match[key]
            )

        return actions_match

    @property
    def low_voltage_threshold(self) -> float:
        """Порог низкого напряжения."""
        return self._low_voltage_threshold

    @property
    def low_voltage_hysteresis(self) -> float:
        """Гистерезис низкого напряжения."""
        return self._low_voltage_hysteresis

    @property
    def wait_timeout(self) -> float:
        """Таймаут ожидания сервисов."""
        return self._wait_timeout

    @property
    def actions_match(self) -> dict[str, str]:
        """Маппинг действий {имя: путь}."""
        return self._actions_match

    @property
    def is_allow_work(self) -> bool:
        """Разрешена ли работа контроллера."""
        return self._is_allow_work

    @is_allow_work.setter
    def is_allow_work(self, value: bool) -> None:
        """Устанавливает разрешение на работу контроллера."""
        self._is_allow_work = value

    @property
    def rate_(self) -> Rate:
        """Rate для ожидания."""
        return self._rate

    @property
    def reentrant_callback_group(self) -> ReentrantCallbackGroup:
        """Callback группа для параллельных вызовов."""
        return self._reentrant_callback_group

    @property
    def media_driver(self) -> MediaDriverConnector:
        """Драйвер медиа."""
        return self._media_driver

    @property
    def ears_driver(self) -> EarsDriverConnector:
        """Драйвер ушей."""
        return self._ears_driver

    @property
    def neck_driver(self) -> NeckDriverConnector:
        """Драйвер шеи."""
        return self._neck_driver

    @property
    def sensor_driver(self) -> SensorDriverConnector:
        """Драйвер сенсоров."""
        return self._sensor_driver

    @property
    def respeaker_driver(self) -> RespeakerDriverConnector:
        """Драйвер ReSpeaker."""
        return self._respeaker_driver

    @property
    def speech_recognizer_asr(self) -> SpeechRecognizerAsrConnector:
        """Коннектор распознавателя речи ASR."""
        return self._speech_recognizer_asr

    @property
    def speech_recognizer_kws(self) -> SpeechRecognizerKwsConnector:
        """Коннектор распознавателя ключевых слов KWS."""
        return self._speech_recognizer_kws

    @property
    def usb_cam(self) -> UsbCamConnector:
        """Коннектор USB-камеры."""
        return self._usb_cam

    @property
    def silero_tts(self) -> SileroTTSConnector:
        """Коннектор Silero TTS."""
        return self._silero_tts

    @property
    def action_manager(self) -> ActionManager:
        """Менеджер действий."""
        return self._action_manager

    @property
    def battery_monitor(self) -> BatteryMonitor:
        """Монитор батареи."""
        return self._battery_monitor

    @property
    def commander(self) -> Commander:
        """Командер."""
        return self._commander

    def connect_all_drivers(self) -> None:
        """Подключение всех драйверов к их сервисам и топикам."""
        self.get_logger().info("Starting driver connections...")

        self._media_driver.connect()
        self._ears_driver.connect()
        self._neck_driver.connect()
        self._sensor_driver.connect()
        self._respeaker_driver.connect()
        self._speech_recognizer_asr.connect()
        self._speech_recognizer_kws.connect()
        self._usb_cam.connect()
        self._silero_tts.connect()

        self.get_logger().info("All drivers connected successfully")

    def sleep(self, cancel_event: threading.Event, duration: float) -> None:
        """
        Ожидание с возможностью прерывания.

        Args:
            cancel_event (threading.Event): событие для прерывания ожидания
            duration (float): длительность ожидания в секундах
        """
        start_time = self.get_clock().now()

        while (
            not cancel_event.is_set()
            and (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration
        ):
            self._rate.sleep()

    def start(self) -> None:
        """Запуск контроллера после инициализации."""
        if self._startup_timer is not None:
            self._startup_timer.cancel()

        # Предзагрузка всех действий
        # self._action_manager.preload_all_actions()

        # Запуск начального действия
        self._action_manager.execute_action("std_startup", None, False)

        self.get_logger().info("Controller started and ready")
