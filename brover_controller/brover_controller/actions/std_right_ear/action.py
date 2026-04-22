# std_right_ear
# действие, выполняющееся при команде "Покажи правое ухо"

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
    # controller.respeaker_driver.set_led_brightness(cancel_event=cancel_event, value=30)
    # controller.respeaker_driver.set_led_color_all(
    # cancel_event=cancel_event, red=255, green=255, blue=255
    # )
    # controller.respeaker_driver.set_led_mode(cancel_event=cancel_event, mode=3)

    # Выводим картинку right_ear.png
    controller.media_driver.play_display(
        cancel_event=cancel_event,
        video_path=os.path.join(action_dir, "right_ear.png"),
        loop=True,
        block=False,
    )

    # Проигрываем звук right_ear.mp3
    controller.media_driver.play_audio(
        cancel_event=cancel_event,
        audio_path=os.path.join(action_dir, "right_ear.mp3"),
        loop=False,
        block=False,
    )

    # Поворачиваем голову вправо
    controller.neck_driver.set_angle(
        cancel_event=cancel_event,
        horizontal=15,
        vertical=15,
        duration=1.0,
        block=False,
    )

    for k in range(5):
        # Поворачиваем уши
        controller.ears_driver.set_angle(
            cancel_event=cancel_event,
            left=0,
            right=90 * (-1) ** k,
            duration=0.5,
            block=True,
        )

    logger.info(f"[{action_name}] finish")
