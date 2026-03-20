# actions/std_echo/action.py

import os
import rclpy
from robohead_controller.main import *
import time

import threading
from typing import Optional, Callable


def run(controller, action_name: str, cancel_event: threading.Event):
    """
    Args:
        controller: Ссылка на контроллер (RoboheadController)
        action_name: Имя действия ('повтори за мной')
        cancel_event: threading.Event для проверки отмены
    """

    logger = controller.get_logger()
    logger.info(f"[{action_name}] Starting greeting action")

    controller.neck_driver.set_angle(cancel_event, horizontal=0, vertical=15, duration=0.5, block=False)

    controller.media_driver.play_display(cancel_event,
    video_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/actions/std_echo/microphone.mp4",
    loop=True, block=False
    )
    controller.silero_tts.say(cancel_event, "Я вас слушаю", block=True)

    controller.speech_recognizer_kws.set_mode(0)
    controller.speech_recognizer_asr.set_mode(2)

    while len(controller.queue_frees) == 0 and not cancel_event.is_set():
        controller.sleep(cancel_event, 0.2)


    controller.speech_recognizer_kws.set_mode(1)
    # asr автоматически переход в режим 0 (выкл)

    controller.media_driver.play_display(cancel_event,
    video_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/actions/std_echo/speaker.mp4",
    loop=True, block=False
    )

    controller.silero_tts.say(cancel_event, controller.queue_frees.pop(0), block=True)
    controller.sleep(cancel_event, 0.5)

    controller.queue_frees.clear()