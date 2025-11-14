# actions/std_ears/action.py

import os
import rclpy
from gyrobro_controller.main import *
# from typing import TYPE_CHECKING

# if TYPE_CHECKING:
#     from gyrobro_controller.types import *

# def run(robohead:RoboheadController, cmd: str):
#     # Получаем путь к текущей папке действия
#     action_dir = os.path.dirname(os.path.abspath(__file__))

#     # Медиафайлы лежат рядом с action.py
#     image_path = os.path.join(action_dir, 'attention.png')
#     robohead.get_logger().info(f"image: {image_path}")
#     audio_path = os.path.join(action_dir, 'attention.mp3')
#     # audio_path = os.path.join(action_dir, 'ears.mp3')

#     # Проигрываем видео
#     msg = PlayMedia.Request()
#     msg.path_to_media_file = image_path  # или mp4, если есть
#     msg.path_to_override_audio_file = audio_path
#     msg.is_block = False
#     msg.is_cycle = False

#     robohead.get_logger().info("call play media")
#     future = robohead.media_driver_srv_play_media.call_async(msg)
#     # while not future.done():
#     #     rclpy.spin_once(robohead)
#     rclpy.spin_until_future_complete(robohead, future)

#     # Движение ушей
#     msg = Move.Request()
#     msg.angle_a = 90
#     msg.angle_b = -90
#     msg.duration = 0.5
#     msg.is_block = False

#     robohead.get_logger().info("call ears driver")
#     future = robohead.ears_driver_srv_ears_set_angle.call_async(msg)
#     # while not future.done():
#     #     rclpy.spin_once(robohead)
#     rclpy.spin_until_future_complete(robohead, future)

#     msg = Move.Request()
#     msg.angle_a = 30
#     msg.angle_b = 30
#     msg.duration = 1.5
#     msg.is_block = True
#     # print("call neck drier")
#     robohead.get_logger().info("call neck driver")

#     future = robohead.neck_driver_srv_neck_set_angle.call_async(msg)
#     # while not future.done():
#     #     rclpy.spin_once(robohead)
#     rclpy.spin_until_future_complete(robohead, future)
    
# actions/std_ears/action.py

# def run(robohead: RoboheadController, cmd: str):
#     action_dir = os.path.dirname(os.path.abspath(__file__))
#     image_path = os.path.join(action_dir, 'attention.png')
#     audio_path = os.path.join(action_dir, 'attention.mp3')

#     def step1_done(future):
#         try:
#             future.result()
#             robohead.get_logger().info("Media played → move ears")
#             # Шаг 2: уши
#             msg = Move.Request()
#             msg.angle_a = 90
#             msg.angle_b = -90
#             msg.duration = 0.5
#             msg.is_block = False
#             future2 = robohead.ears_driver_srv_ears_set_angle.call_async(msg)
#             future2.add_done_callback(step2_done)
#         except Exception as e:
#             robohead.get_logger().error(f"Step1 failed: {e}")

#     def step2_done(future):
#         try:
#             future.result()
#             robohead.get_logger().info("Ears moved → move neck")
#             # Шаг 3: шея
#             msg = Move.Request()
#             msg.angle_a = 30
#             msg.angle_b = 30
#             msg.duration = 1.5
#             msg.is_block = True
#             future3 = robohead.neck_driver_srv_neck_set_angle.call_async(msg)
#             future3.add_done_callback(step3_done)
#         except Exception as e:
#             robohead.get_logger().error(f"Step2 failed: {e}")

#     def step3_done(future):
#         try:
#             future.result()
#             robohead.get_logger().info("Action std_ears completed")
#         except Exception as e:
#             robohead.get_logger().error(f"Step3 failed: {e}")

#     # Запуск шага 1
#     msg = PlayMedia.Request()
#     msg.path_to_media_file = image_path
#     msg.path_to_override_audio_file = audio_path
#     msg.is_block = False
#     msg.is_cycle = False
#     robohead.get_logger().info("call play media")
#     future = robohead.media_driver_srv_play_media.call_async(msg)
#     future.add_done_callback(step1_done)

import functools
robohead = None
on_complete = None
def run(robohead_:RoboheadController, cmd: str, cancel_event,on_complete_=None):
    global robohead
    global on_complete
    on_complete = on_complete_
    robohead = robohead_
    """
    Последовательное действие:
    1. Показывает attention.png + проигрывает звук
    2. Двигает ушами
    3. Двигает шеей
    Прерывается при установке cancel_event.
    """
    # if cancel_event.is_set():
    #     robohead.get_logger().info("[std_attention] Skipped (preempted before start)")
    #     return

    action_dir = os.path.dirname(os.path.abspath(__file__))
    image_path = os.path.join(action_dir, 'attention.mp4')
    audio_path = os.path.join(action_dir, 'attention.mp3')

    robohead.get_logger().info(f"[std_attention] Starting. Media: {image_path}, Audio: {audio_path}")

    req1 = PlayMedia.Request()
    req1.path_to_media_file = image_path
    req1.path_to_override_audio_file = audio_path
    req1.is_block = False
    req1.is_cycle = True

    future = robohead.media_driver_srv_play_media.call_async(req1)
    future.add_done_callback(step2)

def step2(future):
    robohead.get_logger().info(f"[std_attention]. step 2")
    req2 = Move.Request()
    req2.angle_a = 45
    req2.angle_b = -45
    req2.duration = 1.0
    req2.is_block = True
    future2 = robohead.ears_driver_srv_ears_set_angle.call_async(req2)
    future2.add_done_callback(step3)
def step3(future):
    robohead.get_logger().info(f"[std_attention]. step 2")
    req2 = Move.Request()
    req2.angle_a = 30
    req2.angle_b = 0
    req2.duration = 1.0
    req2.is_block = True
    future2 = robohead.neck_driver_srv_neck_set_angle.call_async(req2)
    future2.add_done_callback(step4)

def step4(future):
    robohead.get_logger().info(f"[std_attention]. step 3")

    if on_complete:
        try:
            on_complete()
        except Exception as e:
            robohead.get_logger().error(f"[std_greeting] on_complete failed: {e}")
    

    # while not future1.done():
    #     if cancel_event.is_set():
    #         robohead.get_logger().info("[std_attention] Cancelled during media step")
    #         return
    #     time.sleep(0.01)

    # if cancel_event.is_set():
    #     return
    # try:
    #     future1.result()
    #     robohead.get_logger().info("[std_attention] Media step OK")
    # except Exception as e:
    #     robohead.get_logger().error(f"[std_attention] Media failed: {e}")
    #     return

    # === Шаг 2: Уши ===
    # if cancel_event.is_set():
    #     return


    # future2 = robohead.ears_driver_srv_ears_set_angle.call_async(req2)

    # while not future2.done():
    #     if cancel_event.is_set():
    #         robohead.get_logger().info("[std_attention] Cancelled during ears step")
    #         return
    #     time.sleep(0.01)

    # if cancel_event.is_set():
    #     return
    # try:
    #     future2.result()
    #     robohead.get_logger().info("[std_attention] Ears step OK")
    # except Exception as e:
    #     robohead.get_logger().error(f"[std_attention] Ears failed: {e}")
    #     return

    # === Шаг 3: Шея ===
    # if cancel_event.is_set():
    #     return
    # req3 = Move.Request()
    # req3.angle_a = 30
    # req3.angle_b = 30
    # req3.duration = 1.5
    # req3.is_block = True
    # future3 = robohead.neck_driver_srv_neck_set_angle.call(req3)

    # future3 = robohead.neck_driver_srv_neck_set_angle.call_async(req3)

    # while not future3.done():
    #     if cancel_event.is_set():
    #         robohead.get_logger().info("[std_attention] Cancelled during neck step")
    #         return
    #     time.sleep(0.01)

    # if not cancel_event.is_set():
    #     robohead.get_logger().info(f"[{cmd}] Action completed successfully")
    #     # if on_complete is not None:
    #     #     on_complete()
    # else:
    #     robohead.get_logger().info(f"[{cmd}] Action was cancelled")