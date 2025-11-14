import os
import threading
import time
import cv2
import numpy as np
from cv_bridge import CvBridge
from robohead_interfaces.srv import PlayMedia, Move
from robohead_controller.main import RoboheadController
import rclpy

robohead = None
on_complete = None

# === Глобальные параметры трекера (можно вынести в параметры) ===
resized_camera_resolution = (270, 270)
original_camera_resolution = (640, 480)
screen_resolution = (1080, 1080)

neck_mover_step_duration = 0.1
neck_mover_step_value = 2
neck_mover_constraint_vertical = (-30, 30)
neck_mover_constraint_horizontal = (-30, 30)

threshold_zone = 15
radius_cal = 30
delta_cal = 10

delta_k = 5 / ((resized_camera_resolution[0] // 2) ** 2 + (resized_camera_resolution[1] // 2) ** 2) ** 0.5

# === Внутренние переменные ===
is_run = False
ball_xy = (0, 0)
hsv_filter = ((16, 180), (40, 255), (120, 255))  # (H, S, V)

# === Потоки ===
thread_ball_tracker = None
thread_neck_mover = None

cv_bridge = CvBridge()

def run(robohead_: RoboheadController, cmd: str, cancel_event: threading.Event, on_complete_=None):
    """
    Откалибровать цвет шарика, затем отслеживать его и двигать шеей.
    Прерывается по cancel_event.
    """
    global robohead, on_complete, is_run
    robohead = robohead_
    on_complete = on_complete_

    script_path = os.path.dirname(os.path.abspath(__file__)) + '/'

    # --- Ушки в нейтральное положение ---
    if cancel_event.is_set():
        return
    move_ears(-30, -30, 0.3)

    robohead.get_logger().info("[ball_tracker] ears seted")


    # --- Проиграть звук калибровки ---
    if cancel_event.is_set():
        return
    play_audio("/home/pi/robohead_ws/src/robohead2/robohead_controller/actions/std_ball_tracker/calibrate_voice.mp3", block=True)
    robohead.get_logger().info("[ball_tracker]  audio_played")

    # --- Калибровка ---
    if cancel_event.is_set():
        return
    global hsv_filter
    robohead.get_logger().info("[ball_tracker] start calibrating")

    hsv_filter = calibrate(cancel_event, 10, radius_cal, delta_cal)
    robohead.get_logger().info(f"[ball_tracker] calibrated:{hsv_filter}")
    if cancel_event.is_set():
        return

    # --- Запуск трекинга ---
    is_run = True
    global thread_ball_tracker, thread_neck_mover
    thread_ball_tracker = threading.Thread(target=ball_tracker, args=(cancel_event,))
    thread_neck_mover = threading.Thread(target=neck_mover, args=(cancel_event,))

    thread_ball_tracker.start()
    thread_neck_mover.start()

    # --- Ждём 30 секунд или отмены ---
    start_time = time.time()
    while is_run and not cancel_event.is_set() and (time.time() - start_time) < 30:
        time.sleep(0.1)

    is_run = False
    if thread_ball_tracker.is_alive():
        thread_ball_tracker.join()
    if thread_neck_mover.is_alive():
        thread_neck_mover.join()

    if cancel_event.is_set():
        robohead.get_logger().info("[ball_tracker] Cancelled")
        return

    # --- Проиграть финальный звук ---
    time.sleep(0.2)
    play_audio("/home/pi/robohead_ws/src/robohead2/robohead_controller/actions/std_ball_tracker/finish_voice.mp3", block=True)

    if on_complete:
        try:
            on_complete()
        except Exception as e:
            robohead.get_logger().error(f"[ball_tracker] on_complete failed: {e}")


# === Вспомогательные функции ===

def play_audio(path, block=True):
    req = PlayMedia.Request()
    req.path_to_media_file = path  # без изображения
    # req.path_to_override_audio_file = path
    req.is_block = block
    req.is_cycle = False
    future = robohead.media_driver_srv_play_media.call_async(req)
    while not future.done():
        time.sleep(0.1)


def move_ears(left_angle, right_angle, duration):
    req = Move.Request()
    req.angle_a = left_angle
    req.angle_b = right_angle
    req.duration = duration
    req.is_block = True
    future = robohead.ears_driver_srv_ears_set_angle.call_async(req)
    while not future.done():
        time.sleep(0.1)


def neck_mover(cancel_event: threading.Event):
    global ball_xy, is_run

    cur_horizontal_angle = 0
    cur_vertical_angle = 0
    step = neck_mover_step_value
    center_x = resized_camera_resolution[0] // 2
    center_y = resized_camera_resolution[1] // 2

    while is_run and not cancel_event.is_set():
        x, y = ball_xy
        delta = (((center_x - x) ** 2 + (center_y - y) ** 2) ** 0.5) * delta_k
        delta = min(max(delta, 1), 3)
        step_d = int(step * delta)

        changed = False

        if x < (center_x - threshold_zone):
            if (cur_horizontal_angle + step_d) <= neck_mover_constraint_horizontal[1]:
                cur_horizontal_angle += step_d
                changed = True
        elif x > (center_x + threshold_zone):
            if (cur_horizontal_angle - step_d) >= neck_mover_constraint_horizontal[0]:
                cur_horizontal_angle -= step_d
                changed = True

        if y < (center_y - threshold_zone):
            if (cur_vertical_angle + step_d) <= neck_mover_constraint_vertical[1]:
                cur_vertical_angle += step_d
                changed = True
        elif y > (center_y + threshold_zone):
            if (cur_vertical_angle - step_d) >= neck_mover_constraint_vertical[0]:
                cur_vertical_angle -= step_d
                changed = True

        if changed:
            req = Move.Request()
            req.angle_a = cur_vertical_angle   # vertical
            req.angle_b = cur_horizontal_angle # horizontal
            req.duration = neck_mover_step_duration
            req.is_block = False
            future = robohead.ears_driver_srv_ears_set_angle.call_async(req)
            while not future.done():
                time.sleep(0.1)

        time.sleep(0.05)


def ball_tracker(cancel_event: threading.Event):
    global ball_xy, is_run
    prev_img = None
    center_x = int(resized_camera_resolution[0]/2)
    center_y = int(resized_camera_resolution[1]/2)
    while is_run and not cancel_event.is_set():
        current_img = robohead.usb_cam_image_raw
        if current_img is None or (prev_img is not None and prev_img.header.stamp == current_img.header.stamp):
            time.sleep(0.1)
            continue

        prev_img = current_img

        try:
            cv_image = cv_bridge.imgmsg_to_cv2(current_img, "bgr8")
        except Exception as e:
            robohead.get_logger().warn(f"[ball_tracker] CV conversion failed: {e}")
            continue

        rect = min(original_camera_resolution)
        cv_image = cv_image[:rect, :rect]
        cv_image = cv2.resize(cv_image, resized_camera_resolution)
        cv_image = cv2.flip(cv_image, 1)

        blurred = cv2.GaussianBlur(cv_image, (15, 15), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        hlow, hhigh = hsv_filter[0]
        slow, shigh = hsv_filter[1]
        vlow, vhigh = hsv_filter[2]
        mask = cv2.inRange(hsv, np.array([hlow, slow, vlow]), np.array([hhigh, shigh, vhigh]))

        edged = cv2.Canny(mask, 50, 150)
        cnts = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

        if cnts:
            c = max(cnts, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(c)
            if radius > 10:
                ball_xy = (int(x), int(y))
                cv2.circle(cv_image, ball_xy, int(radius), (0, 255, 0), 2)
                cv2.circle(cv_image, ball_xy, 1, (0, 0, 255), 3)

        lu = (center_x - threshold_zone, center_y - threshold_zone)
        rd = (center_x + threshold_zone, center_y + threshold_zone)
        cv2.rectangle(cv_image, lu, rd, (0, 0, 255), 1)

        display_img = cv2.resize(cv_image, screen_resolution)
        try:
            msg = cv_bridge.cv2_to_imgmsg(display_img, encoding="bgr8")
            robohead.media_driver_pub_stream.publish(msg)
        except Exception as e:
            robohead.get_logger().warn(f"[ball_tracker] Display publish failed: {e}")

        time.sleep(0.1)


def calibrate(cancel_event: threading.Event, duration_calibrate: int, radius_cal: int, delta_cal: int):
    global ball_xy
    prev_img = None
    center_x = resized_camera_resolution[0] // 2
    center_y = resized_camera_resolution[1] // 2

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 10
    color = (255, 0, 0)
    thickness = 20

    start_time = time.time()
    last_img = None

    while (time.time() - start_time) < duration_calibrate and not cancel_event.is_set():
        current_img = robohead.usb_cam_image_raw
        robohead.get_logger().warn(f"[ball_tracker] calibrate_1")
        # robohead.get_logger().warn(f"[ball_tracker] calibrate_1")

        if current_img is None or (prev_img and prev_img.header.stamp == current_img.header.stamp):
            time.sleep(0.03)
            continue
        robohead.get_logger().warn(f"[ball_tracker] calibrate_2")
        prev_img = current_img

        try:
            cv_image = cv_bridge.imgmsg_to_cv2(current_img, "bgr8")
        except:
            continue
        robohead.get_logger().warn(f"[ball_tracker] calibrate_3")

        rect = min(original_camera_resolution)
        cv_image = cv_image[:rect, :rect]
        cv_image = cv2.resize(cv_image, resized_camera_resolution)
        cv_image = cv2.flip(cv_image, 1)
        last_img = cv_image.copy()

        cv2.circle(cv_image, (center_x, center_y), radius_cal + delta_cal, (0, 255, 0), 2)
        cv2.circle(cv_image, (center_x, center_y), radius_cal, (255, 0, 0), 1)

        remaining = int(duration_calibrate - (time.time() - start_time))
        text = str(max(remaining, 0))
        screen_img = cv2.resize(cv_image, screen_resolution)
        text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
        pos = (screen_resolution[0] // 2 - text_size[0] // 2,
               screen_resolution[1] // 2 - 2 * text_size[1] // 2 - radius_cal)
        cv2.putText(screen_img, text, pos, font, font_scale, color, thickness)

        try:
            msg = cv_bridge.cv2_to_imgmsg(screen_img, encoding="bgr8")
            robohead.media_driver_pub_stream.publish(msg)
        except:
            pass

        time.sleep(0.03)

    if last_img is None:
        return ((0, 180), (0, 255), (0, 255))

    roi = last_img[
          int(center_x - radius_cal):int(center_x + radius_cal),
          int(center_y - radius_cal):int(center_y + radius_cal)
          ]

    if roi.size == 0:
        return ((0, 180), (0, 255), (0, 255))

    mask = np.zeros(roi.shape[:2], dtype="uint8")
    cv2.circle(mask, (roi.shape[1] // 2, roi.shape[0] // 2), radius_cal, 255, -1)
    masked = cv2.bitwise_and(roi, roi, mask=mask)

    return get_hsv_range(masked)


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
        (min(v_vals), max(v_vals))
    )