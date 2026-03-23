# actions/std_echo/action.py
import requests
import json

import os
import rclpy
from robohead_controller.main import *
import time

import threading
from typing import Optional, Callable

MODEL_URL = "http://robohead0llm.local:11434/api/generate"
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

def run(controller, action_name: str, cancel_event: threading.Event):

    logger = controller.get_logger()
    logger.info(f"[{action_name}] Starting greeting action")

    controller.neck_driver.set_angle(cancel_event, horizontal=0, vertical=15, duration=0.5, block=False)

    controller.media_driver.play_display(cancel_event,
    video_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/actions/std_llm/microphone.mp4",
    loop=True, block=False
    )
    controller.silero_tts.say(cancel_event, "Я вас слушаю", block=True)

    controller.speech_recognizer_kws.set_mode(0)
    controller.speech_recognizer_asr.set_mode(2)

    while len(controller.queue_frees) == 0 and not cancel_event.is_set():
        controller.sleep(cancel_event, 0.2)

    controller.media_driver.play_display(cancel_event,
    video_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/actions/std_llm/loading.gif",
    loop=True, block=False
    )

    controller.silero_tts.say(cancel_event, "Думаю над ответом", block=False)
    user_promt = controller.queue_frees.pop(0)
    controller.queue_frees.clear()
    logger.info(f"[{action_name}] User promt: |{user_promt}|")
    payload = {
    "model": MODEL_NAME,
    "prompt": PROMT+user_promt,
    "stream": False
    }
    

    try:
        controller.speech_recognizer_kws.set_mode(1)
        # asr автоматически переход в режим 0 (выкл)

        response = requests.post(MODEL_URL, json=payload, timeout=30)
        response.raise_for_status()

        data = response.json()
        llm_answer = data["response"]
        logger.info(f"[{action_name}] LLM answer: |{llm_answer}|")


        controller.media_driver.play_display(cancel_event,
        video_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/actions/std_llm/speaker.mp4",
        loop=True, block=False
        )

        controller.silero_tts.say(cancel_event, llm_answer, block=True)
        controller.sleep(cancel_event, 0.5)

    except:
        controller.silero_tts.say(cancel_event, "Нет соединения", block=True)
        return
    

    