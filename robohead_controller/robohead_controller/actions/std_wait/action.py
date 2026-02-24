#!/usr/bin/env python3
"""
std_wait action: Циклическая анимация ожидания + индикация светодиодами
"""

import time
from typing import Optional
import threading

def run(controller, action_name: str="", cancel_event: threading.Event=None):
    """
    Основная функция действия.
    
    Args:
        controller: Ссылка на контроллер (RoboheadController)
        action_name: Имя действия ('std_wait')
        cancel_event: threading.Event для проверки отмены
        on_complete: Колбэк завершения (обычно None для циклических действий)
    """
    logger = controller.get_logger()
    logger.info(f"[{action_name}] Starting wait animation")

    controller.media_driver.play_media(cancel_event,
        video_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/actions/std_wait/wait_silence.mp4",
        audio_path=controller.media_driver.stop_command,
        loop=True, block=False
    )

    result = controller.ears_driver.set_angle(cancel_event, left=0, right=0, duration=1.0, block=True)
    logger.info(f"[{action_name}] Finish")


