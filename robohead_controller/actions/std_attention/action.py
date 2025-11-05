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
    image_path = os.path.join(action_dir, 'attention.png')
    robohead.get_logger().info(f"image: {image_path}")
    audio_path = os.path.join(action_dir, 'attention.mp3')
    # audio_path = os.path.join(action_dir, 'ears.mp3')

    # Проигрываем видео
    msg = PlayMedia.Request()
    msg.path_to_media_file = image_path  # или mp4, если есть
    msg.path_to_override_audio_file = audio_path
    msg.is_block = False
    msg.is_cycle = False

    robohead.get_logger().info("call play media")
    future = robohead.media_driver_srv_play_media.call_async(msg)
    while rclpy.ok():
        rclpy.spin_once(robohead)
        if future.done():
            break
    # rclpy.spin_until_future_complete(robohead, future)

    # Движение ушей
    msg = Move.Request()
    msg.angle_a = 90
    msg.angle_b = -90
    msg.duration = 0.5
    msg.is_block = False

    robohead.get_logger().info("call ears driver")
    future = robohead.ears_driver_srv_ears_set_angle.call_async(msg)
    while rclpy.ok():
        rclpy.spin_once(robohead)
        if future.done():
            break
    # rclpy.spin_until_future_complete(robohead, future)

    msg = Move.Request()
    msg.angle_a = 30
    msg.angle_b = 30
    msg.duration = 1.5
    msg.is_block = True
    # print("call neck drier")
    robohead.get_logger().info("call neck driver")

    future = robohead.neck_driver_srv_neck_set_angle.call_async(msg)
    while rclpy.ok():
        rclpy.spin_once(robohead)
        if future.done():
            break
    # rclpy.spin_until_future_complete(robohead, future)
    
