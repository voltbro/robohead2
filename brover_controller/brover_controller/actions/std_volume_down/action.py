# std_volume_down
# действие, выполняющееся при команде "Тише"

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

    volume = controller.media_driver.get_volume(cancel_event=cancel_event)
    if volume == None:
        return

    if volume - 10 >= 0:
        controller.media_driver.set_volume(
            cancel_event=cancel_event, volume=volume - 10
        )

        if controller.media_driver.is_idle_audio(cancel_event=cancel_event):
            controller.media_driver.play_audio(
                cancel_event=cancel_event,
                audio_path=os.path.join(action_dir, "set_vol.mp3"),
                loop=False,
                block=False,
            )

    logger.info(f"[{action_name}] finish")
