# robohead_controller/controller.py

from __future__ import annotations
from typing import TYPE_CHECKING

# Только импорты типов (не создаются объекты, не запускаются узлы)
from rclpy.node import Node

# Сообщения и сервисы
from robohead_interfaces.srv import PlayMedia, SimpleCommand, Move
from robohead_interfaces.msg import AudioData, ColorArray
from sensor_msgs.msg import BatteryState, Image
from std_msgs.msg import String

if TYPE_CHECKING:
    # Эти импорты нужны только для анализа типов (IDE / mypy)
    pass

class RoboheadController(Node):
    """
    Сигнатура класса для type hinting.
    НЕ ИСПОЛЬЗУЕТСЯ В РАНТАЙМЕ — только для автодополнения в IDE.
    """

    # === Сервис-клиенты ===
    media_driver_srv_play_media: PlayMedia
    # media_driver_srv_stop_playback: SimpleCommand

    ears_driver_srv_ears_set_angle: Move
    neck_driver_srv_neck_set_angle: Move

    speech_recognizer_srv_set_mode: SimpleCommand

    # === Подписки и данные состояния ===
    sensor_driver_bat_voltage: float
    sensor_driver_bat_current: float
    is_allow_work: bool

    # === Управление выполнением действий ===
    # current_action_task: object  # asyncio.Task или None
    # is_waiting_for_command: bool

    # === Конструктор (заглушка) ===
    def __init__(self) -> None:
        raise NotImplementedError(
            "This class is for type hinting only. "
            "The real implementation is in robohead_controller/node.py"
        )