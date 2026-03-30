# actions/std_echo/action.py
import requests
import json

import os
import rclpy
from robohead_controller.main import *
import time

import threading
from typing import Optional, Callable


def run(controller, action_name: str, cancel_event: threading.Event):

    logger = controller.get_logger()
    logger.info(f"[{action_name}] Starting loading action")


    controller.media_driver.play_display(cancel_event,
    video_path="/home/pi/robohead_ws/src/robohead2/robohead_controller_ai/robohead_controller_ai/actions/std_loading/loading.gif",
    loop=True, block=False
    )

    controller.silero_tts.say(cancel_event, "Думаю над ответом", block=True)
