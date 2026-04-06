# std_make_photo
# действие, выполняющееся при команде "Сделай фото"

from __future__ import annotations
from typing import TYPE_CHECKING
import os
import cv2
from cv_bridge import CvBridge

if TYPE_CHECKING:
    from robohead_controller.controller import RoboheadController
    import threading

# Константы для отрисовки текста на кадре
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 10
FONT_COLOR = (255, 255, 255)
THICKNESS = 20
LINE_TYPE = cv2.LINE_AA


def run(
    controller: RoboheadController, action_name: str, cancel_event: threading.Event
):
    """
    Args:
        controller: Ссылка на контроллер
        action_name: Команда, по которой было вызвано действие
        cancel_event: threading.Event для проверки отмены
    """
    action_dir = os.path.dirname(os.path.abspath(__file__))
    cv_bridge = CvBridge()

    logger = controller.get_logger()
    logger.info(f"[{action_name}] start")

    # Поворачиваем уши и шею
    controller.ears_driver.set_angle(
        cancel_event=cancel_event, left=-30, right=-30, duration=1.0, block=False
    )
    controller.neck_driver.set_angle(
        cancel_event=cancel_event, horizontal=0, vertical=30, duration=1.0, block=False
    )

    # Цикл обратного отсчета (от 5 до 1)
    for num in range(5, 0, -1):
        if cancel_event.is_set():
            logger.info(f"[{action_name}] Cancelled during countdown ({num})")
            return

        start_time = controller.get_clock().now()
        duration_sec = 1.0

        # Крутим цикл ровно 1 секунду для отображения текущей цифры
        while (
            not cancel_event.is_set()
            and (controller.get_clock().now() - start_time).nanoseconds / 1e9
            < duration_sec
        ):
            # Получаем свежий кадр
            img_msg = controller.usb_cam.image_raw
            if img_msg is None:
                controller.rate_.sleep()
                continue

            cv_image = cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
            cv_image = cv2.resize(cv_image, (1080, 1080))

            text = str(num)
            text_size = cv2.getTextSize(text, FONT, FONT_SCALE, THICKNESS)[0]
            pos = (1080 // 2 - text_size[0] // 2, 1080 // 2 + text_size[1] // 2)

            cv2.putText(
                cv_image, text, pos, FONT, FONT_SCALE, FONT_COLOR, THICKNESS, LINE_TYPE
            )

            out_msg = cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            controller.media_driver.stream_publish(out_msg)

            controller.rate_.sleep()

    # Завершение отсчета: финальное фото и звук
    if cancel_event.is_set():
        return

    # Проигрываем звук затвора
    controller.media_driver.play_audio(
        cancel_event=cancel_event,
        audio_path=os.path.join(action_dir, "make_photo.mp3"),
        loop=False,
        block=False,
    )

    # Захватываем и показываем финальный кадр (без цифр)
    controller.sleep(cancel_event, 0.1)
    img_msg = controller.usb_cam.image_raw
    if img_msg is not None:
        cv_image = cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
        cv_image = cv2.resize(cv_image, (1080, 1080))
        out_msg = cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        controller.media_driver.stream_publish(out_msg)
        controller.media_driver.stream_publish(out_msg)
        controller.media_driver.stream_publish(out_msg)

    # Пауза 5 секунд, чтобы пользователь успел посмотреть на "фотографию"
    controller.sleep(cancel_event, 5.0)

    logger.info(f"[{action_name}] finish")
