import os
import time
import threading
from robohead_interfaces.srv import PlayMedia, Move
from robohead_controller.main import *
robohead = None
on_complete = None

def run(robohead_:RoboheadController, cmd: str, cancel_event: threading.Event, on_complete_=None):
    global robohead
    global on_complete
    robohead = robohead_
    on_complete = on_complete_

    action_dir = os.path.dirname(os.path.abspath(__file__))
    image_path = os.path.join(action_dir, 'ears.png')
    audio_path = os.path.join(action_dir, 'ears.mp3')

    robohead.get_logger().info(f"[std_greeting] Starting. Media: {image_path}, Audio: {audio_path}")

    # Шаг 1: media (async)
    if cancel_event is not None and cancel_event.is_set():
        robohead.get_logger().info("[std_greeting] Preempted before start")
        return

    req1 = PlayMedia.Request()
    req1.path_to_media_file = image_path
    req1.path_to_override_audio_file = audio_path
    req1.is_block = False
    req1.is_cycle = False
    future1 = robohead.media_driver_srv_play_media.call_async(req1)
    future1.add_done_callback(step2)

def step2(future):
    # if cancel_event is not None and cancel_event.is_set():
    req = Move.Request()
    req.angle_a = 0
    req.angle_b = -45
    req.duration = 0.4
    req.is_block = True

    future = robohead.ears_driver_srv_ears_set_angle.call_async(req)
    future.add_done_callback(step3)

def step3(future):
    # if cancel_event is not None and cancel_event.is_set():
    req = Move.Request()
    req.angle_a = 0
    req.angle_b = 45
    req.duration = 0.4
    req.is_block = True

    future = robohead.ears_driver_srv_ears_set_angle.call_async(req)
    future.add_done_callback(step4)

def step4(future):
    # if cancel_event is not None and cancel_event.is_set():
    req = Move.Request()
    req.angle_a = 0
    req.angle_b = -45
    req.duration = 0.4
    req.is_block = True

    future = robohead.ears_driver_srv_ears_set_angle.call_async(req)
    future.add_done_callback(step5)

def step5(future):
    # if cancel_event is not None and cancel_event.is_set():
    req = Move.Request()
    req.angle_a = 0
    req.angle_b = 45
    req.duration = 0.4
    req.is_block = True

    future = robohead.ears_driver_srv_ears_set_angle.call_async(req)
    future.add_done_callback(step6)

def step6(future):
    if on_complete:
        try:
            on_complete()
        except Exception as e:
            robohead.get_logger().error(f"[std_greeting] on_complete failed: {e}")