# std_ball_tracker
# Действие: откалибровать цвет шарика, затем отслеживать его и двигать шеей. По команде "Следи за шариком"

from __future__ import annotations
from typing import TYPE_CHECKING
import os
import time
import threading
import cv2
import numpy as np
from cv_bridge import CvBridge

if TYPE_CHECKING:
    from robohead_controller.controller import RoboheadController

# === Глобальные параметры машинного зрения ===
RESIZED_CAMERA_RESOLUTION = (270, 270)
ORIGINAL_CAMERA_RESOLUTION = (640, 480)
SCREEN_RESOLUTION = (1080, 1080)

NECK_STEP_DURATION = 0.1
NECK_STEP_VALUE = 2
NECK_CONSTRAINT_VERTICAL = (-30, 30)
NECK_CONSTRAINT_HORIZONTAL = (-30, 30)

THRESHOLD_ZONE = 15
RADIUS_CAL = 30
DELTA_CAL = 10

DELTA_K = (
    5
    / (
        (RESIZED_CAMERA_RESOLUTION[0] // 2) ** 2
        + (RESIZED_CAMERA_RESOLUTION[1] // 2) ** 2
    )
    ** 0.5
)


class BallTrackerApp:
    def __init__(
        self,
        controller: RoboheadController,
        action_name: str,
        cancel_event: threading.Event,
    ):
        self.controller = controller
        self.action_name = action_name
        self.cancel_event = cancel_event
        self.logger = controller.get_logger()
        self.action_dir = os.path.dirname(os.path.abspath(__file__))
        self.cv_bridge = CvBridge()

        # Внутреннее состояние, разделяемое между потоками
        self.is_run = False
        self.ball_xy = (0, 0)
        self.hsv_filter = ((16, 180), (40, 255), (120, 255))  # Дефолтный (H, S, V)

    def execute(self):
        self.logger.info(f"[{self.action_name}] start")

        # --- 1. Ушки и шея в нейтральное положение ---
        if self.cancel_event.is_set():
            return
        self.controller.ears_driver.set_angle(
            cancel_event=self.cancel_event,
            left=-30,
            right=-30,
            duration=0.3,
            block=True,
        )
        self.controller.neck_driver.set_angle(
            cancel_event=self.cancel_event,
            horizontal=0,
            vertical=0,
            duration=0.5,
            block=True,
        )

        # --- 2. Проигрываем звук "Калибровка" ---
        if self.cancel_event.is_set():
            return
        self.controller.media_driver.play_audio(
            cancel_event=self.cancel_event,
            audio_path=os.path.join(self.action_dir, "calibrate_voice.mp3"),
            loop=False,
            block=False,
        )

        # --- 3. Калибровка цвета ---
        if self.cancel_event.is_set():
            return
        self.logger.info(f"[{self.action_name}] start calibrating")
        self.hsv_filter = self.calibrate(
            duration_calibrate=10, radius_cal=RADIUS_CAL, delta_cal=DELTA_CAL
        )
        self.logger.info(f"[{self.action_name}] calibrated HSV: {self.hsv_filter}")

        if self.cancel_event.is_set():
            return

        # --- 4. Запуск потоков трекинга и движения ---
        self.is_run = True
        thread_vision = threading.Thread(target=self.vision_thread)
        thread_neck = threading.Thread(target=self.neck_thread)

        thread_vision.start()
        thread_neck.start()

        # --- 5. Ждём 30 секунд или отмены ---
        start_time = time.time()
        while (
            self.is_run
            and not self.cancel_event.is_set()
            and (time.time() - start_time) < 30
        ):
            time.sleep(0.1)

        # --- 6. Завершение работы потоков ---
        self.is_run = False
        thread_vision.join()
        thread_neck.join()

        if self.cancel_event.is_set():
            self.logger.info(f"[{self.action_name}] Cancelled")
            return

        # --- 7. Проигрываем финальный звук ---
        self.controller.media_driver.play_audio(
            cancel_event=self.cancel_event,
            audio_path=os.path.join(self.action_dir, "finish_voice.mp3"),
            loop=False,
            block=True,
        )
        self.logger.info(f"[{self.action_name}] finish")

    def neck_thread(self):
        cur_horizontal_angle = 0
        cur_vertical_angle = 0
        step = NECK_STEP_VALUE
        center_x = RESIZED_CAMERA_RESOLUTION[0] // 2
        center_y = RESIZED_CAMERA_RESOLUTION[1] // 2

        while self.is_run and not self.cancel_event.is_set():
            x, y = self.ball_xy
            delta = (((center_x - x) ** 2 + (center_y - y) ** 2) ** 0.5) * DELTA_K
            delta = min(max(delta, 1), 3)
            step_d = int(step * delta)

            changed = False

            # Вычисление горизонтального отклонения
            if x < (center_x - THRESHOLD_ZONE):
                if (cur_horizontal_angle + step_d) <= NECK_CONSTRAINT_HORIZONTAL[1]:
                    cur_horizontal_angle += step_d
                    changed = True
            elif x > (center_x + THRESHOLD_ZONE):
                if (cur_horizontal_angle - step_d) >= NECK_CONSTRAINT_HORIZONTAL[0]:
                    cur_horizontal_angle -= step_d
                    changed = True

            # Вычисление вертикального отклонения
            if y < (center_y - THRESHOLD_ZONE):
                if (cur_vertical_angle + step_d) <= NECK_CONSTRAINT_VERTICAL[1]:
                    cur_vertical_angle += step_d
                    changed = True
            elif y > (center_y + THRESHOLD_ZONE):
                if (cur_vertical_angle - step_d) >= NECK_CONSTRAINT_VERTICAL[0]:
                    cur_vertical_angle -= step_d
                    changed = True

            # Применение движения (асинхронно, чтобы не тормозить цикл)
            if changed:
                self.controller.neck_driver.set_angle(
                    cancel_event=self.cancel_event,
                    horizontal=cur_horizontal_angle,
                    vertical=cur_vertical_angle,
                    duration=NECK_STEP_DURATION,
                    block=True,
                )

            time.sleep(0.05)

    def vision_thread(self):
        prev_img = None
        center_x = int(RESIZED_CAMERA_RESOLUTION[0] / 2)
        center_y = int(RESIZED_CAMERA_RESOLUTION[1] / 2)

        while self.is_run and not self.cancel_event.is_set():
            current_img = self.controller.usb_cam.image_raw
            if current_img is None or (
                prev_img is not None
                and prev_img.header.stamp == current_img.header.stamp
            ):
                time.sleep(0.1)
                continue

            prev_img = current_img

            try:
                cv_image = self.cv_bridge.imgmsg_to_cv2(current_img, "bgr8")
            except Exception as e:
                self.logger.warn(f"[{self.action_name}] CV conversion failed: {e}")
                continue

            rect = min(ORIGINAL_CAMERA_RESOLUTION)
            cv_image = cv_image[:rect, :rect]
            cv_image = cv2.resize(cv_image, RESIZED_CAMERA_RESOLUTION)
            cv_image = cv2.flip(cv_image, 1)

            blurred = cv2.GaussianBlur(cv_image, (15, 15), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            hlow, hhigh = self.hsv_filter[0]
            slow, shigh = self.hsv_filter[1]
            vlow, vhigh = self.hsv_filter[2]
            mask = cv2.inRange(
                hsv, np.array([hlow, slow, vlow]), np.array([hhigh, shigh, vhigh])
            )

            edged = cv2.Canny(mask, 50, 150)
            cnts = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

            if cnts:
                c = max(cnts, key=cv2.contourArea)
                (x, y), radius = cv2.minEnclosingCircle(c)
                if radius > 10:
                    self.ball_xy = (int(x), int(y))
                    cv2.circle(cv_image, self.ball_xy, int(radius), (0, 255, 0), 2)
                    cv2.circle(cv_image, self.ball_xy, 1, (0, 0, 255), 3)

            # Отрисовка зоны допуска
            lu = (center_x - THRESHOLD_ZONE, center_y - THRESHOLD_ZONE)
            rd = (center_x + THRESHOLD_ZONE, center_y + THRESHOLD_ZONE)
            cv2.rectangle(cv_image, lu, rd, (0, 0, 255), 1)

            display_img = cv2.resize(cv_image, SCREEN_RESOLUTION)
            try:
                msg = self.cv_bridge.cv2_to_imgmsg(display_img, encoding="bgr8")
                self.controller.media_driver.stream_publish(msg)
            except Exception as e:
                pass

            time.sleep(0.1)

    def calibrate(self, duration_calibrate: int, radius_cal: int, delta_cal: int):
        prev_img = None
        center_x = RESIZED_CAMERA_RESOLUTION[0] // 2
        center_y = RESIZED_CAMERA_RESOLUTION[1] // 2

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 10
        color = (255, 0, 0)
        thickness = 20

        start_time = time.time()
        last_img = None

        while (
            time.time() - start_time
        ) < duration_calibrate and not self.cancel_event.is_set():
            current_img = self.controller.usb_cam.image_raw

            if current_img is None or (
                prev_img and prev_img.header.stamp == current_img.header.stamp
            ):
                time.sleep(0.03)
                continue
            prev_img = current_img

            try:
                cv_image = self.cv_bridge.imgmsg_to_cv2(current_img, "bgr8")
            except:
                continue

            rect = min(ORIGINAL_CAMERA_RESOLUTION)
            cv_image = cv_image[:rect, :rect]
            cv_image = cv2.resize(cv_image, RESIZED_CAMERA_RESOLUTION)
            cv_image = cv2.flip(cv_image, 1)
            last_img = cv_image.copy()

            # Рисуем круги калибровки
            cv2.circle(
                cv_image, (center_x, center_y), radius_cal + delta_cal, (0, 255, 0), 2
            )
            cv2.circle(cv_image, (center_x, center_y), radius_cal, (255, 0, 0), 1)

            remaining = int(duration_calibrate - (time.time() - start_time))
            text = str(max(remaining, 0))
            screen_img = cv2.resize(cv_image, SCREEN_RESOLUTION)

            text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
            pos = (
                SCREEN_RESOLUTION[0] // 2 - text_size[0] // 2,
                SCREEN_RESOLUTION[1] // 2 - 2 * text_size[1] // 2 - radius_cal,
            )

            cv2.putText(screen_img, text, pos, font, font_scale, color, thickness)

            try:
                msg = self.cv_bridge.cv2_to_imgmsg(screen_img, encoding="bgr8")
                self.controller.media_driver.stream_publish(msg)
            except:
                pass

            time.sleep(0.03)

        if last_img is None:
            return ((0, 180), (0, 255), (0, 255))

        roi = last_img[
            int(center_x - radius_cal) : int(center_x + radius_cal),
            int(center_y - radius_cal) : int(center_y + radius_cal),
        ]

        if roi.size == 0:
            return ((0, 180), (0, 255), (0, 255))

        mask = np.zeros(roi.shape[:2], dtype="uint8")
        cv2.circle(mask, (roi.shape[1] // 2, roi.shape[0] // 2), radius_cal, 255, -1)
        masked = cv2.bitwise_and(roi, roi, mask=mask)

        return self.get_hsv_range(masked)

    @staticmethod
    def get_hsv_range(img):
        blurred = cv2.GaussianBlur(img, (15, 15), 0)
        hsv_img = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        h_vals, s_vals, v_vals = [], [], []

        for row in hsv_img:
            for pixel in row:
                h, s, v = pixel
                if s > 100 and v > 100:
                    h_vals.append(h)
                    s_vals.append(s)
                    v_vals.append(v)

        if not h_vals:
            return ((0, 180), (0, 255), (0, 255))

        return (
            (min(h_vals), max(h_vals)),
            (min(s_vals), max(s_vals)),
            (min(v_vals), max(v_vals)),
        )


def run(
    controller: RoboheadController, action_name: str, cancel_event: threading.Event
):
    """Точка входа (вызывается из ядра системы)"""
    app = BallTrackerApp(controller, action_name, cancel_event)
    app.execute()
