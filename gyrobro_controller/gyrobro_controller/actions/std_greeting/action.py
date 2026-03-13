# actions/std_ears/action.py

import os
import rclpy
import time

import threading
from typing import Optional, Callable

def sleep(duration):
    check_interval = 0.1  # Проверяем отмену каждые 100 мс
    
    elapsed = 0.0
    while elapsed < duration and not cancel_event.is_set():
        time.sleep(check_interval)
        elapsed += check_interval

def run(controller, action_name: str, cancel_event: threading.Event):
    """
    Args:
        controller: Ссылка на контроллер (RoboheadController)
        action_name: Имя действия ('greet')
        cancel_event: threading.Event для проверки отмены
        on_complete: Колбэк, вызываемый после успешного завершения
    """

    logger = controller.get_logger()
    logger.info(f"[{action_name}] Starting greeting action")

    result = controller.media_driver.play_display(cancel_event,
    video_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/actions/std_greeting/greeting.mp4",
    loop=True, block=False
    )
    result = controller.media_driver.play_audio(cancel_event,
    audio_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/actions/std_greeting/greeting.mp3",
    loop=False, block=False
    )

    controller.ears_driver.set_angle(cancel_event, left=-90, right=90, duration=1.5, block=False)
    for k in range(5):
        controller.neck_driver.set_angle(cancel_event, horizontal=0, vertical=15 + 10 * (-1)**k, duration=0.3, block=True)