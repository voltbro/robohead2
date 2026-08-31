# std_make_photo
# действие, выполняющееся при команде "Сделай фото"

from __future__ import annotations

from typing import TYPE_CHECKING

import os
import threading

import cv2
import numpy as np
from sensor_msgs.msg import Image

if TYPE_CHECKING:
    from robohead_controller.controller import RoboheadController


# === Функции для замены cv_bridge ===
def imgmsg_to_cv2(img_msg: Image) -> np.ndarray:
    """Конвертирует ROS Image (bgr8) в numpy array без cv_bridge"""
    dtype = np.uint8
    n_channels = 3
    # Если в строке нет паддинга (step == width * 3)
    if img_msg.step == img_msg.width * n_channels:
        return np.frombuffer(img_msg.data, dtype=dtype).reshape((img_msg.height, img_msg.width, n_channels)).copy()
    else:
        # Если есть паддинг в строках
        return np.ndarray(
            shape=(img_msg.height, img_msg.width, n_channels),
            dtype=dtype,
            buffer=img_msg.data,
            strides=(img_msg.step, n_channels, 1)
        ).copy()


def cv2_to_imgmsg(cv_image: np.ndarray) -> Image:
    """Конвертирует numpy array (bgr8) в ROS Image без cv_bridge"""
    msg = Image()
    msg.height = cv_image.shape[0]
    msg.width = cv_image.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = 3 * cv_image.shape[1]
    msg.data = cv_image.tobytes()
    return msg


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
    
    # cv_bridge больше не нужен
    
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

            try:
                cv_image = imgmsg_to_cv2(img_msg)
            except Exception as e:
                logger.warn(f"[{action_name}] CV conversion failed: {e}")
                controller.rate_.sleep()
                continue

            cv_image = cv2.resize(cv_image, (1080, 1080))

            text = str(num)
            text_size = cv2.getTextSize(text, FONT, FONT_SCALE, THICKNESS)[0]
            pos = (1080 // 2 - text_size[0] // 2, 1080 // 2 + text_size[1] // 2)

            cv2.putText(
                cv_image, text, pos, FONT, FONT_SCALE, FONT_COLOR, THICKNESS, LINE_TYPE
            )

            try:
                out_msg = cv2_to_imgmsg(cv_image)
                controller.media_driver.stream_publish(out_msg)
            except Exception as e:
                pass

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
        try:
            cv_image = imgmsg_to_cv2(img_msg)
            cv_image = cv2.resize(cv_image, (1080, 1080))
            out_msg = cv2_to_imgmsg(cv_image)
            controller.media_driver.stream_publish(out_msg)
            controller.media_driver.stream_publish(out_msg)
            controller.media_driver.stream_publish(out_msg)
        except Exception as e:
            pass

    # Пауза 5 секунд, чтобы пользователь успел посмотреть на "фотографию"
    controller.sleep(cancel_event, 5.0)

    logger.info(f"[{action_name}] finish")