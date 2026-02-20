# actions/std_ears/action.py

import os
import rclpy
from robohead_controller.main import *
import time



#!/usr/bin/env python3
"""
greet/action.py - Простое действие: воспроизведение приветствия
"""

import time
import threading
from typing import Optional, Callable

def run(controller, action_name: str, cancel_event: threading.Event):
    """
    Args:
        controller: Ссылка на контроллер (RoboheadController)
        action_name: Имя действия ('greet')
        cancel_event: threading.Event для проверки отмены
        on_complete: Колбэк, вызываемый после успешного завершения
    """
    # foo(controller, action_name)
    # if on_complete != None and cancel_event.is_set():
    #     on_complete()

    def sleep(duration):
        check_interval = 0.1  # Проверяем отмену каждые 100 мс
        
        elapsed = 0.0
        while elapsed < duration and not cancel_event.is_set():
            time.sleep(check_interval)
            elapsed += check_interval

# def foo(controller, action_name):
    logger = controller.get_logger()
    logger.info(f"[{action_name}] Starting greeting action")

    result = controller.media_driver.play_media(cancel_event,
    video_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/actions/std_attention/attention.mp4",
    loop=True
    )
    result = controller.media_driver.play_media(cancel_event,
    audio_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/actions/std_attention/attention.mp3",
    loop=False
    )

    result = controller.ears_driver.set_angle(cancel_event, 0, 0, 1)

    sleep(5)
    
    # === ШАГ 3: Проверка результата ===
    if cancel_event.is_set():
        return
    
    logger.info(f"[{action_name}] Greeting completed successfully")
