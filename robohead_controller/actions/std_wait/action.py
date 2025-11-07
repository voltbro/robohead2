# actions/std_ears/action.py

import os
import rclpy
from robohead_controller.main import *
# from typing import TYPE_CHECKING

# if TYPE_CHECKING:
#     from robohead_controller.types import *

# def run(robohead:RoboheadController, cmd: str):
#     # Получаем путь к текущей папке действия
#     action_dir = os.path.dirname(os.path.abspath(__file__))

#     # Медиафайлы лежат рядом с action.py
#     image_path = os.path.join(action_dir, 'wait.png')
#     # audio_path = os.path.join(action_dir, 'ears.mp3')

#     # Проигрываем видео
#     msg = PlayMedia.Request()
#     msg.path_to_media_file = image_path  # или mp4, если есть
#     msg.is_block = False
#     msg.is_cycle = False
#     future = robohead.media_driver_srv_play_media.call_async(msg)
#     # while not future.done():
#         # rclpy.spin_once(robohead)
#     rclpy.spin_until_future_complete(robohead, future, timeout_sec=3.0)

#     # Движение ушей
#     msg = Move.Request()
#     msg.angle_a = 30
#     msg.angle_b = 50
#     msg.duration = 0.5
#     msg.is_block = False

#     future = robohead.ears_driver_srv_ears_set_angle.call_async(msg)
#     # while not future.done():
#     #     rclpy.spin_once(robohead)
#     rclpy.spin_until_future_complete(robohead, future, timeout_sec=3.0)

#     msg = Move.Request()
#     msg.angle_a = 0
#     msg.angle_b = 0
#     msg.duration = 1.0
#     msg.is_block = True
#     future = robohead.neck_driver_srv_neck_set_angle.call_async(msg)
#     # while not future.done():
#     #     rclpy.spin_once(robohead)
#     rclpy.spin_until_future_complete(robohead, future, timeout_sec=3.0)
    
import os
import functools

# def run(robohead:RoboheadController, cmd: str):
#     action_dir = os.path.dirname(os.path.abspath(__file__))
#     image_path = os.path.join(action_dir, 'wait.png')

#     robohead.get_logger().info(f"[std_ears] Starting action. Display: {image_path}")

#     # Шаг 1: медиа
#     req1 = PlayMedia.Request()
#     req1.path_to_media_file = image_path
#     req1.is_block = False
#     req1.is_cycle = False
#     future1 = robohead.media_driver_srv_play_media.call_async(req1)
#     future1.add_done_callback(functools.partial(_on_media_done, robohead=robohead))


# def _on_media_done(future, robohead):
#     try:
#         future.result()
#         robohead.get_logger().info("[std_ears] Media played")
#     except Exception as e:
#         robohead.get_logger().error(f"[std_ears] Media failed: {e}")
#         return

#     # Шаг 2: уши
#     req2 = Move.Request()
#     req2.angle_a = 30
#     req2.angle_b = 50
#     req2.duration = 0.5
#     req2.is_block = False
#     future2 = robohead.ears_driver_srv_ears_set_angle.call_async(req2)
#     future2.add_done_callback(functools.partial(_on_ears_done, robohead=robohead))


# def _on_ears_done(future, robohead):
#     try:
#         future.result()
#         robohead.get_logger().info("[std_ears] Ears moved")
#     except Exception as e:
#         robohead.get_logger().error(f"[std_ears] Ears failed: {e}")
#         return

#     # Шаг 3: шея
#     req3 = Move.Request()
#     req3.angle_a = 0
#     req3.angle_b = 0
#     req3.duration = 1.0
#     req3.is_block = True
#     future3 = robohead.neck_driver_srv_neck_set_angle.call_async(req3)
#     future3.add_done_callback(functools.partial(_on_neck_done, robohead=robohead))


# def _on_neck_done(future, robohead):
#     try:
#         future.result()
#         robohead.get_logger().info("[std_ears] Action completed successfully")
#     except Exception as e:
#         robohead.get_logger().error(f"[std_ears] Neck movement failed: {e}")


import threading
robohead = None
on_complete = None
def run(robohead_:RoboheadController, cmd: str, cancel_event:threading.Event, on_complete_=None):
    global robohead
    global on_complete
    robohead = robohead_
    on_complete = on_complete_
    """
    Показывает экран ожидания (wait.png).
    Прерывается, если cancel_event установлен.
    """

    action_dir = os.path.dirname(os.path.abspath(__file__))
    image_path = os.path.join(action_dir, 'wait.png')

    robohead.get_logger().info(f"[std_wait] Displaying: {image_path}")

    # Формируем запрос
    req = PlayMedia.Request()
    req.path_to_media_file = image_path
    req.is_block = False
    req.is_cycle = False

    # Отправляем асинхронно
    future = robohead.media_driver_srv_play_media.call_async(req)
    future.add_done_callback(step2)

def step2(future):
    req2 = Move.Request()
    req2.angle_a = -90
    req2.angle_b = -90
    req2.duration = 0.5
    req2.is_block = False
    future2 = robohead.ears_driver_srv_ears_set_angle.call_async(req2)
    future2.add_done_callback(step3)

def step3(future):
    req3 = Move.Request()
    req3.angle_a = -30
    req3.angle_b = 0
    req3.duration = 1.5
    req3.is_block = True
    future3 = robohead.neck_driver_srv_neck_set_angle.call_async(req3)
    future3.add_done_callback(step4)

def step4(future):

    robohead.get_logger().info(f"[std_wait]. finish")

    if on_complete:
        try:
            on_complete()
        except Exception as e:
            robohead.get_logger().error(f"[std_greeting] on_complete failed: {e}")