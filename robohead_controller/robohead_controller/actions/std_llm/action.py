# std_llm
# действие, выполняющееся при команде "Ответь на вопрос"

from __future__ import annotations
from typing import TYPE_CHECKING
import os, requests

if TYPE_CHECKING:
    from robohead_controller.controller import RoboheadController
    import threading

MODEL_URL = "http://robohead0llm-2.local:11434/api/generate"
MODEL_NAME = "qwen3:4b-instruct"

PROMT = """
Ты — роботизированная голова «Робоголов+а»: уверенная, ироничная, умная. Разработана в НИИ Механики МГУ.
Отвечай очень кратко, живо и естественно.
Ответ будет сразу озвучен, поэтому пиши только чистый текст без смайликов.
Все числа пиши словами.
Если в неоднозначном слове нужно показать ударение, ставь '+' перед ударной гласной.
Отвечай мягко, не допускай эскалации конфликта.

Команда пользователя:
"""

REQUEST_TIMEOUT = 120


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
        right=90,
        duration=0.5,
        block=False,
    )

    controller.silero_tts.say(
        cancel_event=cancel_event, text="Думаю над ответом", block=False
    )

    user_promt = controller.queue_frees.pop(0)  # Распознанный свободный текст
    controller.queue_frees.clear()  # Очищаем очередь распознанного текста - для надёжности
    logger.info(f"[{action_name}] User promt: |{user_promt}|")
    payload = {"model": MODEL_NAME, "prompt": PROMT + user_promt, "stream": False}

    # Выводим анимацию загрузки
    controller.media_driver.play_display(
        cancel_event=cancel_event,
        video_path=os.path.join(action_dir, "loading.gif"),
        loop=True,
        block=False,
    )

    try:

        response = requests.post(
            url=MODEL_URL, json=payload, timeout=REQUEST_TIMEOUT
        )  # Отправляем post-запрос
        response.raise_for_status()  # Ожидаем ответа

        data = response.json()
        llm_answer = data["response"]
        logger.info(f"[{action_name}] LLM answer: |{llm_answer}|")

        # Выводим анимацию speaker.mp4
        controller.media_driver.play_display(
            cancel_event=cancel_event,
            video_path=os.path.join(action_dir, "speaker.mp4"),
            loop=True,
            block=False,
        )

        controller.silero_tts.say(
            cancel_event=cancel_event, text=llm_answer, block=True
        )
        controller.sleep(cancel_event=cancel_event, duration=1.0)

    except:
        controller.silero_tts.say(
            cancel_event=cancel_event, text="Нет соединения", block=True
        )
        return

    logger.info(f"[{action_name}] finish")
