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


import threading
def run(controller, action_name: str="", cancel_event: threading.Event=None):

    logger = controller.get_logger()
    logger.info(f"[{action_name}] Start")

    # controller.media_driver.play_display(cancel_event,
    #     video_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/actions/std_low_bat/low_bat.mp4",
    #     loop=True, block=False
    # )
    # controller.media_driver.play_audio(cancel_event,
    #     audio_path=controller.media_driver.stop_command,
    #     loop=False, block=False
    # )


    controller.ears_driver.set_angle(cancel_event, left=-30, right=-30, duration=1.0, block=False)
    controller.neck_driver.set_angle(cancel_event, vertical=30, horizontal=0, duration=1.0, block=False)

    countdown_step(controller, cancel_event, 5)


def countdown_step(controller, cancel_event, num: int):
    if cancel_event.is_set():
        controller.get_logger().info(f"[std_make_photo] Cancelled during countdown ({num})")
        return

    if num <= 0:
        # Завершение отсчёта → показать финальный кадр + звук
        finish(controller, cancel_event)
        return

    start_time = controller.get_clock().now()
    duration_sec = 1.0

    while not cancel_event.is_set() and (controller.get_clock().now() - start_time).nanoseconds / 1e9 < duration_sec:

        # Получаем свежий кадр
        img_msg = controller.usb_cam.image_raw
        if img_msg is None:
            controller.rate_.sleep()
            continue


        cv_image = cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
 
        cv_image = cv2.resize(cv_image, (1080, 1080))

        text = str(num)
        text_size = cv2.getTextSize(text, FONT, FONT_SCALE, THICKNESS)[0]
        pos = (
            1080 // 2 - text_size[0] // 2,
            1080 // 2 + text_size[1] // 2
        )
        cv2.putText(cv_image, text, pos, FONT, FONT_SCALE, FONT_COLOR, THICKNESS, LINE_TYPE)

        out_msg = cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        controller.media_driver.pub_stream.publish(out_msg)

        controller.rate_.sleep()

    # Переход к следующему числу
    countdown_step(controller, cancel_event, num - 1)


def finish(controller, cancel_event):
    if cancel_event.is_set():
        return

    controller.media_driver.play_audio(cancel_event,
        audio_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/actions/std_make_photo/make_photo.mp3",
        loop=False, block=False
    )

    img_msg = controller.usb_cam.image_raw

# if img_msg is not None:
    cv_image = cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
    cv_image = cv2.resize(cv_image, (1080, 1080))
    out_msg = cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
    controller.media_driver.pub_stream.publish(out_msg)
    controller.sleep(cancel_event, 5)
