# std_echo
# действие, выполняющееся при команде "Повтори за мной"

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

    # Поворачиваем голову
    controller.neck_driver.set_angle(
        cancel_event=cancel_event,
        horizontal=0,
        vertical=20,
        duration=0.5,
        block=False,
    )

    # Поворачиваем уши
    controller.ears_driver.set_angle(
        cancel_event=cancel_event,
        left=90,
        right=-90,
        duration=0.5,
        block=False,
    )

    # Выводим анимацию microphone.mp4
    controller.media_driver.play_display(
        cancel_event=cancel_event,
        video_path=os.path.join(action_dir, "microphone.mp4"),
        loop=True,
        block=False,
    )

    # Произносим текст "Я вас слушаю"
    controller.silero_tts.say(
        cancel_event=cancel_event, text="Я вас слушаю", block=True
    )

    # Отключить распознавание wake_phrase и fast_commands
    controller.speech_recognizer_kws.set_mode(cancel_event=cancel_event, mode=0)

    # Включить свободное распознавание
    controller.speech_recognizer_asr.set_mode(cancel_event=cancel_event, mode=2)

    # Ожидаем фразу
    while len(controller.queue_frees) == 0 and not cancel_event.is_set():
        controller.sleep(cancel_event, 0.1)
    # -> Фраза получена

    # Включаем распознавание wake_phrase и fast_commands
    controller.speech_recognizer_kws.set_mode(cancel_event=cancel_event, mode=1)
    # asr автоматически переходит в режим 0 (выкл) после распознавания

    # Поворачиваем уши
    controller.ears_driver.set_angle(
        cancel_event=cancel_event,
        left=-90,
        right=+90,
        duration=0.5,
        block=False,
    )

    # Выводим анимацию speaker.mp4
    controller.media_driver.play_display(
        cancel_event=cancel_event,
        video_path=os.path.join(action_dir, "speaker.mp4"),
        loop=True,
        block=False,
    )

    user_text = controller.queue_frees.pop(0)  # Распознанный свободный текст
    controller.queue_frees.clear()  # Очищаем очередь распознанного текста - для надёжности

    # Озвучиваем распознанный текст
    controller.silero_tts.say(cancel_event=cancel_event, text=user_text, block=True)

    # Проигрываем звук ears.mp3
    controller.media_driver.play_audio(
        cancel_event=cancel_event,
        audio_path=os.path.join(action_dir, "ears.mp3"),
        loop=False,
        block=False,
    )

    logger.info(f"[{action_name}] finish")
