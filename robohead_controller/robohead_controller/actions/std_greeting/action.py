# std_greeting
# действие, выполняющееся при команде "Поздоровайся"

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

    # Переключаем режим микрофона ReSpeaker
    controller.respeaker_driver.set_led_brightness(cancel_event=cancel_event, value=100)
    controller.respeaker_driver.set_led_color_all(
    cancel_event=cancel_event, red=255, green=0, blue=0
    )
    controller.respeaker_driver.set_led_mode(cancel_event=cancel_event, mode=3)

    # Выводим анимацию greeting.mp4
    controller.media_driver.play_display(
        cancel_event=cancel_event,
        video_path=os.path.join(action_dir, "greeting.mp4"),
        loop=True,
        block=False,
    )

    # Проигрываем звук greeting.mp3
    controller.media_driver.play_audio(
        cancel_event=cancel_event,
        audio_path=os.path.join(action_dir, "greeting.mp3"),
        loop=False,
        block=False,
    )

    # Поворачиваем уши
    controller.ears_driver.set_angle(
        cancel_event=cancel_event,
        left=-90,
        right=90,
        duration=0.5,
        block=False,
    )

    for k in range(10):
        # Трясём головой
        controller.neck_driver.set_angle(
            cancel_event=cancel_event,
            horizontal=0,
            vertical=15 + 10 * (-1) ** k,
            duration=0.4,
            block=True,
        )

    logger.info(f"[{action_name}] finish")
