# std_show_voltage
# Действие: выводит текущее напряжение и ток аккумулятора на экран в течение 8 секунд

from __future__ import annotations
from typing import TYPE_CHECKING
import cv2
import numpy as np
from cv_bridge import CvBridge

if TYPE_CHECKING:
    from robohead_controller.controller import RoboheadController
    import threading

# === Настройки графики ===
SCREEN_RESOLUTION = (1080, 1080)
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE_TITLE = 4  # Чуть уменьшили для заголовка
FONT_SCALE_VALUE = 5  # Уменьшили, чтобы влезли две строки
THICKNESS_TITLE = 10
THICKNESS_VALUE = 15
LINE_TYPE = cv2.LINE_AA

COLOR_NORMAL = (0, 255, 0)  # Зеленый (BGR: Green)
COLOR_WAIT = (200, 200, 200)  # Серый

# Настройки кругового прогресс-бара
PROGRESS_RADIUS = (
    510  # Радиус кольца (чуть меньше 540, чтобы не обрезалось краями дисплея)
)
PROGRESS_THICKNESS = 30  # Толщина кольца

DURATION_SEC = 8.0  # Время показа

def float_to_volts_text(number: float) -> str:
    # Округляем до 1 знака и разбиваем на целую и дробную части
    num_str = f"{number:.1f}"
    int_part, frac_part = map(int, num_str.split('.'))
    
    # Списки слов (в женском роде, так как согласуются с "долями")
    words_int = ["", "одна", "две", "три", "четыре", "пять", "шесть", "семь", "восемь", "девять"]
    words_frac = ["ноль", "одна", "две", "три", "четыре", "пять", "шесть", "семь", "восемь", "девять"]
    
    # Окончания для целой части (одна целая, остальные — целых)
    int_unit = "целая" if int_part == 1 else "целых"
    
    # Окончания для дробной части (одна десятая, остальные — десятых)
    frac_unit = "десятая" if frac_part == 1 else "десятых"
    
    # Правило для слова "вольт": если дробь .0 — то "вольт", для всех остальных — "вольта"
    volt_unit = "вольт" if frac_part == 0 else "вольта"
    
    # Собираем итоговую строку
    return f"{words_int[int_part]} {int_unit} {words_frac[frac_part]} {frac_unit} {volt_unit}"

def run(
    controller: RoboheadController, action_name: str, cancel_event: threading.Event
):
    """
    Args:
        controller: Ссылка на контроллер
        action_name: Команда, по которой было вызвано действие
        cancel_event: threading.Event для проверки отмены
    """
    logger = controller.get_logger()
    logger.info(f"[{action_name}] start")

    cv_bridge = CvBridge()

    voltage = controller.sensor_driver.battery_voltage if controller.sensor_driver.battery_voltage is not None else 0
    controller.silero_tts.say(cancel_event=cancel_event, text=f"Напряжение батареи {float_to_volts_text(voltage)}", block=False)

    # Засекаем время старта
    start_time = controller.get_clock().now()

    # Крутим цикл, постоянно обновляя экран
    while not cancel_event.is_set():

        # Считаем, сколько секунд прошло
        elapsed_sec = (controller.get_clock().now() - start_time).nanoseconds / 1e9

        # Если время вышло — прерываем цикл
        if elapsed_sec >= DURATION_SEC:
            break

        # 1. Читаем актуальное напряжение и ток
        voltage = controller.sensor_driver.battery_voltage
        current = controller.sensor_driver.battery_current

        # 2. Создаем черный фон
        cv_image = np.zeros(
            (SCREEN_RESOLUTION[1], SCREEN_RESOLUTION[0], 3), dtype=np.uint8
        )

        # === Отрисовка кругового таймера по краю экрана ===
        # Вычисляем долю оставшегося времени (от 1.0 до 0.0)
        fraction_left = max(0.0, 1.0 - (elapsed_sec / DURATION_SEC))
        colors = list()
        for i in range(12):
            colors.append(0)
            if i <= int(fraction_left*12):
                colors.append(255)
            else:
                colors.append(0)
            colors.append(0)
        controller.respeaker_driver.set_led_mode(cancel_event=cancel_event, mode=3)
        controller.respeaker_driver.set_led_color_manual(cancel_event=cancel_event, colors=colors)

        # Вычисляем угол дуги. -90 градусов в OpenCV — это ровно верхняя точка (12 часов)
        start_angle = -90
        end_angle = int(-90 + 360 * fraction_left)

        # Рисуем дугу, которая тает со временем
        cv2.ellipse(
            cv_image,
            (SCREEN_RESOLUTION[0] // 2, SCREEN_RESOLUTION[1] // 2),  # Центр
            (PROGRESS_RADIUS, PROGRESS_RADIUS),  # Оси X и Y
            0,
            start_angle,
            end_angle,
            COLOR_NORMAL,
            PROGRESS_THICKNESS,
            LINE_TYPE,
        )
        # ==================================================

        # 3. Отрисовка слова "BATTERY" (верхняя часть экрана)
        title_text = "BATTERY"
        title_size = cv2.getTextSize(
            title_text, FONT, FONT_SCALE_TITLE, THICKNESS_TITLE
        )[0]
        title_pos = (SCREEN_RESOLUTION[0] // 2 - title_size[0] // 2, 250)

        cv2.putText(
            cv_image,
            title_text,
            title_pos,
            FONT,
            FONT_SCALE_TITLE,
            COLOR_NORMAL,
            THICKNESS_TITLE,
            LINE_TYPE,
        )

        # 4. Отрисовка значения НАПРЯЖЕНИЯ (по центру)
        if voltage is not None:
            text_v = f"{voltage:.3f} V"
            color_v = COLOR_NORMAL
        else:
            text_v = "U: Wait"
            color_v = COLOR_WAIT

        size_v = cv2.getTextSize(text_v, FONT, FONT_SCALE_VALUE, THICKNESS_VALUE)[0]
        pos_v = (SCREEN_RESOLUTION[0] // 2 - size_v[0] // 2, 550)

        cv2.putText(
            cv_image,
            text_v,
            pos_v,
            FONT,
            FONT_SCALE_VALUE,
            color_v,
            THICKNESS_VALUE,
            LINE_TYPE,
        )

        # 5. Отрисовка значения ТОКА (нижняя часть экрана)
        if current is not None:
            text_i = f"{current:.3f} A"
            color_i = COLOR_NORMAL
        else:
            text_i = "I: Wait"
            color_i = COLOR_WAIT

        size_i = cv2.getTextSize(text_i, FONT, FONT_SCALE_VALUE, THICKNESS_VALUE)[0]
        pos_i = (SCREEN_RESOLUTION[0] // 2 - size_i[0] // 2, 850)

        cv2.putText(
            cv_image,
            text_i,
            pos_i,
            FONT,
            FONT_SCALE_VALUE,
            color_i,
            THICKNESS_VALUE,
            LINE_TYPE,
        )

        # 6. Конвертируем и отправляем на экран
        out_msg = cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        controller.media_driver.stream_publish(out_msg)

        # Спим небольшое время (около 20 FPS)
        controller.sleep(cancel_event, 0.05)

    logger.info(f"[{action_name}] finish")
