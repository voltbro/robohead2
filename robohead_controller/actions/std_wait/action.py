# actions/std_ears/action.py

import os
import rclpy
from robohead_controller.main import *
# from typing import TYPE_CHECKING

# if TYPE_CHECKING:
#     from robohead_controller.types import *

def run(robohead:RoboheadController, cmd: str):
    # Получаем путь к текущей папке действия
    action_dir = os.path.dirname(os.path.abspath(__file__))

    # Медиафайлы лежат рядом с action.py
    image_path = os.path.join(action_dir, 'wait.png')
    # audio_path = os.path.join(action_dir, 'ears.mp3')

    # Проигрываем видео
    msg = PlayMedia.Request()
    msg.path_to_media_file = image_path  # или mp4, если есть
    msg.is_block = False
    msg.is_cycle = False
    future = robohead.media_driver_srv_play_media.call_async(msg)
    # rclpy.spin_until_future_complete(robohead, future, timeout_sec=3.0)

    # Движение ушей
    msg = Move.Request()
    msg.angle_a = 30
    msg.angle_b = 50
    msg.duration = 0.5
    msg.is_block = False

    future = robohead.ears_driver_srv_ears_set_angle.call_async(msg)
    # rclpy.spin_until_future_complete(robohead, future, timeout_sec=3.0)

    msg = Move.Request()
    msg.angle_a = 0
    msg.angle_b = 0
    msg.duration = 1.0
    msg.is_block = True
    future = robohead.neck_driver_srv_neck_set_angle.call_async(msg)
    # rclpy.spin_until_future_complete(robohead, future, timeout_sec=3.0)
    
