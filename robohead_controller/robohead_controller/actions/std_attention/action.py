# std_attention
# действие, срабатывающее на wake_phrase ("Слушай, Робот!")

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
    controller.respeaker_driver.set_led_brightness(cancel_event=cancel_event, value=60)
    controller.respeaker_driver.set_led_mode(cancel_event=cancel_event, mode=2)

    # Выводим анимацию attention.mp4
    controller.media_driver.play_display(
        cancel_event=cancel_event,
        video_path=os.path.join(action_dir, "attention.mp4"),
        loop=True,
        block=False,
    )

    # Произносим фразу "Чего надо?"
    controller.silero_tts.say(cancel_event=cancel_event, text="Чего надо?", block=False)

    # Получаем угол, откуда пришёл голос и ограничиваем его в диапазоне от -30 до +30
    h_angle = max(-30, min(30, -controller.respeaker_driver.doa))  # type: ignore

    # Поворачиваем голову в сторону, откуда пришёл звук
    controller.neck_driver.set_angle(
        cancel_event=cancel_event,
        horizontal=h_angle,
        vertical=30,
        duration=1.5,
        block=False,
    )

    # Поворачиваем уши
    controller.ears_driver.set_angle(
        cancel_event=cancel_event, left=-30, right=30, duration=1.5, block=False
    )

    logger.info(f"[{action_name}] finish")
