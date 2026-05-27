# system_charging
# действие, выполняющееся при подключении зарядного устройства
# действие системное, изменяйте с осторожностью

from __future__ import annotations
from typing import TYPE_CHECKING
import os

if TYPE_CHECKING:
    from robohead_controller.controller import RoboheadController
    import threading


def run(
    controller: RoboheadController, action_name: str, cancel_event: threading.Event
):
    """
    Args:
        controller: Ссылка на контроллер
        action_name: Команда, по которой было вызвано действие
        cancel_event: threading.Event для проверки отмены
    """
    action_dir = os.path.dirname(os.path.abspath(__file__))

    logger = controller.get_logger()
    logger.info(f"[{action_name}] start")

    if controller.media_driver.is_idle_audio(cancel_event=cancel_event):
        controller.silero_tts.say(cancel_event=cancel_event, text="Зарядное устройство подключено")

    logger.info(f"[{action_name}] finish")
