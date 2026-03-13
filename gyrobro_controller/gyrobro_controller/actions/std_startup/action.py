
import os
import rclpy
import time
import threading
from typing import Optional, Callable

def run(controller, action_name: str, cancel_event: threading.Event):

    logger = controller.get_logger()
    logger.info(f"[{action_name}] Starting greeting action")

    # Пример: воспроизведение приветственного видео
    
    controller.media_driver.play_display(cancel_event,
        video_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/hello.mp4",
        loop=True, block=False
    )
    
    controller.media_driver.play_audio(cancel_event,
        audio_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/start.mp3",
        loop=False, block=True
    )

    logger.info(f"[{action_name}] Greeting completed successfully")
