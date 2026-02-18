import os
import time
import threading
import cv2
from cv_bridge import CvBridge
import numpy as np

from robohead_interfaces.srv import PlayMedia, Move
from robohead_controller.main import RoboheadController

robohead: RoboheadController = None
on_complete = None
cancel_event: threading.Event = None
action_dir = ""

# Глобальные для шага отсчёта
cv_bridge = CvBridge()
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 10
FONT_COLOR = (255, 255, 255)
THICKNESS = 20
LINE_TYPE = cv2.LINE_AA

def run(robohead_: RoboheadController, cmd: str, cancel_event_: threading.Event, on_complete_=None):
    global robohead, on_complete, cancel_event, action_dir
    robohead = robohead_
    on_complete = on_complete_
    cancel_event = cancel_event_
    action_dir = os.path.dirname(os.path.abspath(__file__))

    robohead.get_logger().info("[std_make_photo] Starting.")

    if cancel_event.is_set():
        robohead.get_logger().info("[std_make_photo] Preempted before start")
        return

    # Шаг 1: Опустить уши
    req = Move.Request()
    req.angle_a = -30  # left
    req.angle_b = -30  # right
    req.duration = 0.5
    req.is_block = False

    future = robohead.ears_driver_srv_ears_set_angle.call_async(req)
    future.add_done_callback(step_ears_done)


def step_ears_done(future):
    try:
        future.result()
    except Exception as e:
        robohead.get_logger().error(f"[std_make_photo] Ears move failed: {e}")
        safe_complete()
        return

    if cancel_event.is_set():
        robohead.get_logger().info("[std_make_photo] Cancelled after ears move")
        safe_complete()
        return

    # Шаг 2: Поднять шею
    req = Move.Request()
    req.angle_a = 30      # vertical
    req.angle_b = 0     # horizontal
    req.duration = 1.0
    req.is_block = False

    future = robohead.neck_driver_srv_neck_set_angle.call_async(req)
    future.add_done_callback(step_neck_done)


def step_neck_done(future):
    try:
        future.result()
    except Exception as e:
        robohead.get_logger().error(f"[std_make_photo] Neck move failed: {e}")
        safe_complete()
        return

    if cancel_event.is_set():
        robohead.get_logger().info("[std_make_photo] Cancelled after neck move")
        safe_complete()
        return

    # Шаг 3: Обратный отсчёт 3-2-1
    countdown_step(3)


def countdown_step(num: int):
    if cancel_event.is_set():
        robohead.get_logger().info(f"[std_make_photo] Cancelled during countdown ({num})")
        safe_complete()
        return

    if num <= 0:
        # Завершение отсчёта → показать финальный кадр + звук
        play_audio_and_show_final_frame()
        return

    start_time = robohead.get_clock().now()
    duration_sec = 1.0
    rate = robohead.create_rate(10)  # 10 Hz

    while (robohead.get_clock().now() - start_time).nanoseconds / 1e9 < duration_sec:
        if cancel_event.is_set():
            safe_complete()
            return

        # Получаем свежий кадр
        img_msg = robohead.usb_cam_image_raw
        if img_msg is None:
            rate.sleep()
            continue

        try:
            cv_image = cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except Exception as e:
            robohead.get_logger().warn(f"[std_make_photo] CV conversion failed: {e}")
            rate.sleep()
            continue

        cv_image = cv2.resize(cv_image, (1080, 1080))

        text = str(num)
        text_size = cv2.getTextSize(text, FONT, FONT_SCALE, THICKNESS)[0]
        pos = (
            1080 // 2 - text_size[0] // 2,
            1080 // 2 + text_size[1] // 2
        )
        cv2.putText(cv_image, text, pos, FONT, FONT_SCALE, FONT_COLOR, THICKNESS, LINE_TYPE)

        try:
            out_msg = cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            robohead.media_driver_pub_stream.publish(out_msg)
        except Exception as e:
            robohead.get_logger().warn(f"[std_make_photo] Failed to publish image: {e}")

        rate.sleep()

    # Переход к следующему числу
    countdown_step(num - 1)


def play_audio_and_show_final_frame():
    if cancel_event.is_set():
        safe_complete()
        return

    # Проигрываем звук
    req = PlayMedia.Request()

    req.path_to_media_file = os.path.join(action_dir, 'camera.png')
    req.path_to_override_audio_file = os.path.join(action_dir, 'make_photo.mp3')
    req.is_block = True
    req.is_cycle = False

    try:
        future = robohead.media_driver_srv_play_media.call_async(req)
        future.add_done_callback(finish)
        # Не ждём завершения — звук не блокирующий
    except Exception as e:
        robohead.get_logger().error(f"[std_make_photo] Failed to play audio: {e}")

def finish(future):
    # Показываем финальный кадр
    img_msg = robohead.usb_cam_image_raw
    if img_msg is not None:
        try:
            cv_image = cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
            cv_image = cv2.resize(cv_image, (1080, 1080))
            out_msg = cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            robohead.media_driver_pub_stream.publish(out_msg)
        except Exception as e:
            robohead.get_logger().warn(f"[std_make_photo] Final frame publish failed: {e}")

    # Ждём ~4 секунды (как в оригинале), но с возможностью отмены
    start = robohead.get_clock().now()
    while (robohead.get_clock().now() - start).nanoseconds / 1e9 < 4.0:
        if cancel_event.is_set():
            robohead.get_logger().info("[std_make_photo] Cancelled during final wait")
            safe_complete()
            return
        time.sleep(0.1)

    safe_complete()


def safe_complete():
    if on_complete:
        try:
            on_complete()
        except Exception as e:
            robohead.get_logger().error(f"[std_make_photo] on_complete failed: {e}")