# std_startup
# действие, выполняющееся один раз при запуске

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
    controller.respeaker_driver.set_led_brightness(cancel_event=cancel_event, value=30)
    controller.respeaker_driver.set_led_mode(cancel_event=cancel_event, mode=1)

    # Выводим анимацию hello.mp4
    controller.media_driver.play_display(
        cancel_event=cancel_event,
        video_path=os.path.join(action_dir, "hello.mp4"),
        loop=True,
        block=False,
    )

    # Останавливаем звук
    controller.media_driver.play_audio(
        cancel_event=cancel_event,
        audio_path=controller.media_driver.stop_command,
        loop=True,
        block=False,
    )

    # Поворачиваем голову в 0,0
    controller.neck_driver.set_angle(
        cancel_event=cancel_event,
        horizontal=0,
        vertical=0,
        duration=1.5,
        block=False,
    )

    # Поворачиваем уши
    controller.ears_driver.set_angle(
        cancel_event=cancel_event, left=0, right=0, duration=1.5, block=False
    )

    # Произносим приветственную фразу
    controller.silero_tts.say(cancel_event=cancel_event, text="Ну привет!", block=True)

    # Включаем распознавание команд
    controller.speech_recognizer_kws.set_mode(cancel_event=cancel_event, mode=1)

    logger.info(f"[{action_name}] finish")
