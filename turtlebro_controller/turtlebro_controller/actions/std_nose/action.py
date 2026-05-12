# std_node
# Управление головой через перетаскивание «носа»

from __future__ import annotations
from typing import TYPE_CHECKING, Optional, Tuple
import cv2
import numpy as np
from cv_bridge import CvBridge

if TYPE_CHECKING:
    from turtlebro_controller.controller import TurtlebroController
    import threading
    from robohead_interfaces.msg import TouchEvent

# Настройки графики
SCREEN_W, SCREEN_H = 1080, 1080  # из вашего примера
NOSE_COLOR = (255, 128, 60)  # BGR
NOSE_BORDER_COLOR = (255, 255, 255)
NOSE_BORDER_THICK = 4
BG_COLOR = (0, 0, 0)  # чёрный фон


def _render_nose_frame(x: int, y: int):
    # Чёрный фон
    frame = np.full((SCREEN_H, SCREEN_W, 3), BG_COLOR, dtype=np.uint8)

    # Основной круг
    cv2.circle(img=frame, center=(x, y), radius=60, color=NOSE_COLOR, thickness=-1, lineType=cv2.LINE_AA)
    # Обводка
    cv2.circle(img=frame, center=(x, y), radius=60, color=NOSE_BORDER_COLOR, thickness=NOSE_BORDER_THICK, lineType=cv2.LINE_AA)
    # Маленькая точка в центре для визуального якоря
    cv2.circle(img=frame, center=(x, y), radius=6, color=NOSE_BORDER_COLOR, thickness=-1, lineType=cv2.LINE_AA)
    
    # Конвертируем в ROS-сообщение
    return CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")

cur_xy = (int(1080/2), int(1080/2))

def on_touch(msg: TouchEvent):
    global cur_xy
    
    if msg.state == "up":
        cur_xy = (int(1080/2), int(1080/2))
    else:
        x = int(msg.x/4095*1079)
        y = int(msg.y/4095*1079)

        if ((cur_xy[0]-x)**2 + (cur_xy[1]-y)**2) > 10**2:
            cur_xy = (x,y)

def run(controller: TurtlebroController, action_name: str, cancel_event: threading.Event):
    logger = controller.get_logger()
    logger.info(f"[{action_name}] start nose control")

    bridge = CvBridge()
    
    controller.media_driver.register_touch_callback(on_touch)
    frame = _render_nose_frame(cur_xy[0],cur_xy[1])
    controller.media_driver.stream_publish(frame)
    controller.silero_tts.say(cancel_event=cancel_event, text="Потяни за носик", block=False)

    try:
        start_time = controller.get_clock().now()
        while not cancel_event.is_set():
            elapsed = (controller.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed >= 20:
                break

            x,y = cur_xy
            frame = _render_nose_frame(x,y)
            controller.media_driver.stream_publish(frame)
            horizontal = -(1080/2 - x)/1080*2.0
            vertical = (1080/2 - y)/1080*30

            controller.neck_driver.set_angle(cancel_event, int(vertical), 0, duration=0.0, block=True)
            controller.turtlebro_driver.velocity_publish(linear_x=0.0, angular_z=float(horizontal))

            controller.sleep(cancel_event, 0.05)

    finally:
        controller.neck_driver.set_angle(
            cancel_event=cancel_event,
            horizontal=0, vertical=0,
            duration=0.3, block=True
        )
        controller.turtlebro_driver.velocity_publish(linear_x=0.0, angular_z=0.0)

        controller.media_driver.unregister_touch_callback(on_touch)
        controller.media_driver.clear_touch_state()

        frame = _render_nose_frame(cur_xy[0],cur_xy[1])
        controller.media_driver.stream_publish(frame)
        controller.silero_tts.say(cancel_event=cancel_event, text="Всё, хватит!", block=True)

        
        logger.info(f"[{action_name}] nose control stopped")