# actions/std_ears/action.py

import os
import rclpy
from robohead_controller.main import *
import time

import threading
from typing import Optional, Callable

def run(controller, action_name: str, cancel_event: threading.Event):

    volume = controller.media_driver.get_volume(cancel_event)
    if volume == None:
        return

    if volume + 10 <= 100:
        controller.media_driver.set_volume(cancel_event, volume+10)
        result = controller.media_driver.play_media(cancel_event, audio_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/fast_actions/std_volume_up/set_vol.mp3", loop=False)
    else:
        result = controller.media_driver.play_media(cancel_event, audio_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/fast_actions/std_volume_up/max_vol.mp3", loop=False)
