#!/usr/bin/env python3
"""
std_wait action: Циклическая анимация ожидания + индикация светодиодами
"""

import time
from typing import Optional
import threading

def run(controller, action_name: str="", cancel_event: threading.Event=None):
    # foo(controller, action_name)
    # if on_complete != None:
    #     on_complete()


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
        video_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/actions/std_right_ear/ears.png",
        loop=True
    )
    controller.media_driver.play_media(cancel_event,
        audio_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/actions/std_right_ear/ears.mp3",
        loop=False
    )

    for k in range(5):
        result = controller.ears_driver.set_angle(cancel_event, 0, 90*(-1)**k, 0.5)
