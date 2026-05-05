# std_left_ear
# действие, выполняющееся при команде "Покажи левое ухо"

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
    cancel_event=cancel_event, red=0, green=0, blue=255
    )
    controller.respeaker_driver.set_led_mode(cancel_event=cancel_event, mode=3)

    # Выводим картинку left_ear.png
    controller.media_driver.play_display(
        cancel_event=cancel_event,
        video_path=os.path.join(action_dir, "left_ear.png"),
        loop=True,
        block=False,
    )

    # Проигрываем звук left_ear.mp3
    controller.media_driver.play_audio(
        cancel_event=cancel_event,
        audio_path=os.path.join(action_dir, "left_ear.mp3"),
        loop=False,
        block=False,
    )

    # Поворачиваем голову влево
    controller.neck_driver.set_angle(
        cancel_event=cancel_event,
        horizontal=-15,
        vertical=15,
        duration=1.0,
        block=False,
    )

    for k in range(5):
        # Поворачиваем уши
        controller.ears_driver.set_angle(
            cancel_event=cancel_event,
            left=90 * (-1) ** k,
            right=0,
            duration=0.5,
            block=True,
        )

    logger.info(f"[{action_name}] finish")
