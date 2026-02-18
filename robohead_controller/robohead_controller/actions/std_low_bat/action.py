# actions/std_ears/action.py

import os
import rclpy
from robohead_controller.main import *
    
import os
import functools

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

    robohead.get_logger().info(f"[std_low_bat] Displaying")

    req = Move.Request()
    req.angle_a = 0
    req.angle_b = 0
    req.duration = 0.5
    req.is_block = False
    future = robohead.ears_driver_srv_ears_set_angle.call_async(req)
    future.add_done_callback(step2)



def step2(future):
    req3 = Move.Request()
    req3.angle_a = 0
    req3.angle_b = 0
    req3.duration = 1.5
    req3.is_block = False
    future3 = robohead.neck_driver_srv_neck_set_angle.call_async(req3)
    future3.add_done_callback(step3)

def step3(future):
    # Формируем запрос
    req = PlayMedia.Request()
    action_dir = os.path.dirname(os.path.abspath(__file__))
    req.path_to_media_file = os.path.join(action_dir, 'low_bat.mp4')
    req.path_to_override_audio_file = os.path.join(action_dir, 'low_bat.mp3')
    req.is_block = True
    req.is_cycle = False

    # Отправляем асинхронно
    future = robohead.media_driver_srv_play_media.call_async(req)
    future.add_done_callback(step4)

def step4(future):
    # Формируем запрос
    req = PlayMedia.Request()
    action_dir = os.path.dirname(os.path.abspath(__file__))
    req.path_to_media_file = os.path.join(action_dir, 'low_bat.mp4')
    # req.path_to_override_audio_file = os.path.join(action_dir, 'low_bat.mp3')
    req.is_block = False
    req.is_cycle = True

    # Отправляем асинхронно
    future = robohead.media_driver_srv_play_media.call_async(req)
    future.add_done_callback(step5)

def step5(future):

    robohead.get_logger().info(f"[std_low_bat]. finish")

    # if on_complete:
    #     try:
    #         on_complete()
    #     except Exception as e:
    #         robohead.get_logger().error(f"[std_greeting] on_complete failed: {e}")