# std_ya_music
# Действие: воспроизведение Яндекс.Музыки с графическим интерфейсом на OpenCV
# - Круглый дисплей 1080×1080: все элементы в безопасной зоне
# - Кнопки: << (пред.), >> (след.), ((| / |)) (громкость), X (выход)
# - Обложка трека встраивается в кадр и публикуется через stream_publish
# - Аудио воспроизводится через play_audio
# - Ручное преобразование cv2 → sensor_msgs/Image (без CvBridge)
# - Текст через Pillow: полная поддержка кириллицы
# - Громкость управляется через driver.get_volume() / driver.set_volume()
# - FIX: сброс глобального состояния при каждом запуске

from __future__ import annotations
from typing import TYPE_CHECKING
import os
import cv2
import numpy as np
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import Header
from yandex_music import Client
from PIL import ImageFont, ImageDraw, Image
import socket
from yandex_music.exceptions import UnauthorizedError, YandexMusicError

if TYPE_CHECKING:
    from robohead_controller.controller import RoboheadController
    import threading

from pathlib import Path
from dotenv import load_dotenv

# Робот ВСЕГДА ищет секреты в корне пользователя, 
# независимо от того, где установлен ROS 2

# === Настройки графики ===
SCREEN_RESOLUTION = (1080, 1080)
TOUCH_RESOLUTION = 4096
SCREEN_CENTER = (540, 540)
SAFE_RADIUS = 490

# Цвета (BGR)
COLOR_BG = (0, 0, 0)
COLOR_BTN_NORMAL = (40, 40, 40)
COLOR_BTN_PRESSED = (80, 80, 120)
COLOR_TEXT = (255, 255, 255)
COLOR_VOLUME = (0, 255, 0)
COLOR_TITLE = (255, 200, 0)
COLOR_BORDER = (180, 180, 180)

# === Зоны кнопок (ASCII-символы) ===
BTN_PREV = {'x1': 240, 'y1': 820, 'x2': 360, 'y2': 940, 'label': '<<'}
BTN_VOL_DOWN = {'x1': 390, 'y1': 820, 'x2': 510, 'y2': 940, 'label': '((|'}
BTN_VOL_UP = {'x1': 570, 'y1': 820, 'x2': 690, 'y2': 940, 'label': '|))'}
BTN_NEXT = {'x1': 720, 'y1': 820, 'x2': 840, 'y2': 940, 'label': '>>'}
BTN_EXIT = {'x1': 820, 'y1': 140, 'x2': 940, 'y2': 260, 'label': 'X'}

# Область обложки
COVER_CENTER = (540, 540)
COVER_RADIUS = 220

# Настройки громкости
VOLUME_MIN = 0
VOLUME_MAX = 100
VOLUME_STEP = 10

# === Глобальные переменные состояния ===
cur_ind = 0
exit_requested = False
touch_command = None
pressed_button = None


# === Шрифты для Pillow (кириллица) ===
def find_cyrillic_font():
    """Возвращает путь к первому найденному шрифту с поддержкой кириллицы"""
    font_candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
        "/usr/share/fonts/noto/NotoSans-Regular.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/msttcorefonts/Arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed.ttf",
    ]
    for font_path in font_candidates:
        if os.path.exists(font_path):
            return font_path
    return None

CYRILLIC_FONT_PATH = find_cyrillic_font()


def draw_text_pillow(cv_image: np.ndarray, text: str, position: tuple, 
                     font_size: int, color_bgr: tuple, 
                     font_path: str = None, anchor: str = "left-top") -> np.ndarray:
    """Рисует текст с поддержкой кириллицы через Pillow."""
    cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(cv_image_rgb)
    draw = ImageDraw.Draw(pil_image)
    
    font_path = font_path or CYRILLIC_FONT_PATH
    if font_path and os.path.exists(font_path):
        try:
            font = ImageFont.truetype(font_path, font_size)
        except Exception:
            font = ImageFont.load_default()
    else:
        font = ImageFont.load_default()
    
    color_rgb = color_bgr[::-1]
    
    if anchor == "center":
        try:
            bbox = draw.textbbox((0, 0), text, font=font)
            text_w = bbox[2] - bbox[0]
            text_h = bbox[3] - bbox[1]
            x, y = position[0] - text_w // 2, position[1] - text_h // 2
        except:
            x, y = position[0] - len(text) * font_size // 3, position[1] - font_size // 2
    elif anchor == "right-top":
        try:
            bbox = draw.textbbox((0, 0), text, font=font)
            text_w = bbox[2] - bbox[0]
            x, y = position[0] - text_w, position[1]
        except:
            x, y = position[0] - len(text) * font_size * 0.6, position[1]
    else:
        x, y = position
    
    draw.text((x, y), text, font=font, fill=color_rgb)
    result = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
    return result


def scale_touch_coord(coord: int) -> int:
    return int(coord * SCREEN_RESOLUTION[0] / TOUCH_RESOLUTION)


def is_point_in_safe_zone(x: int, y: int) -> bool:
    dx = x - SCREEN_CENTER[0]
    dy = y - SCREEN_CENTER[1]
    return (dx * dx + dy * dy) <= (SAFE_RADIUS * SAFE_RADIUS)


def point_in_rect(x: int, y: int, btn: dict) -> bool:
    in_rect = btn['x1'] <= x < btn['x2'] and btn['y1'] <= y < btn['y2']
    btn_center_x = (btn['x1'] + btn['x2']) // 2
    btn_center_y = (btn['y1'] + btn['y2']) // 2
    in_safe = is_point_in_safe_zone(btn_center_x, btn_center_y)
    return in_rect and in_safe


def on_touch(msg: TouchEvent):
    """Обработчик касаний"""
    global exit_requested, touch_command, pressed_button
    
    if msg.state != "up":
        return
    
    x = scale_touch_coord(msg.x)
    y = scale_touch_coord(msg.y)
    
    if point_in_rect(x, y, BTN_EXIT):
        exit_requested = True
        touch_command = 'exit'
    elif point_in_rect(x, y, BTN_PREV):
        touch_command = 'prev'
    elif point_in_rect(x, y, BTN_NEXT):
        touch_command = 'next'
    elif point_in_rect(x, y, BTN_VOL_UP):
        touch_command = 'vol_up'
    elif point_in_rect(x, y, BTN_VOL_DOWN):
        touch_command = 'vol_down'
    elif is_point_in_safe_zone(x, y):
        dx = x - COVER_CENTER[0]
        dy = y - COVER_CENTER[1]
        if dx * dx + dy * dy <= COVER_RADIUS * COVER_RADIUS:
            touch_command = 'prev' if x < COVER_CENTER[0] else 'next'


def draw_round_button(cv_image: np.ndarray, btn: dict, is_pressed: bool = False):
    cx = (btn['x1'] + btn['x2']) // 2
    cy = (btn['y1'] + btn['y2']) // 2
    radius = (btn['x2'] - btn['x1']) // 2
    
    color = COLOR_BTN_PRESSED if is_pressed else COLOR_BTN_NORMAL
    cv2.circle(cv_image, (cx, cy), radius, color, -1)
    cv2.circle(cv_image, (cx, cy), radius, COLOR_BORDER, 2)
    
    cv_image = draw_text_pillow(
        cv_image, 
        text=btn['label'], 
        position=(cx, cy), 
        font_size=36, 
        color_bgr=COLOR_TEXT,
        font_path=CYRILLIC_FONT_PATH,
        anchor="center"
    )
    return cv_image


def draw_volume_control(cv_image: np.ndarray, vol: int):
    bar_y = 760
    bar_x_start = 350
    bar_width = 380
    bar_height = 20
    
    vol = max(VOLUME_MIN, min(VOLUME_MAX, int(vol)))
    
    cv2.rectangle(cv_image, (bar_x_start, bar_y), (bar_x_start + bar_width, bar_y + bar_height), 
                  COLOR_BTN_NORMAL, -1)
    cv2.rectangle(cv_image, (bar_x_start, bar_y), (bar_x_start + bar_width, bar_y + bar_height), 
                  COLOR_BORDER, 1)
    
    fill_width = int(bar_width * vol / VOLUME_MAX)
    if fill_width > 0:
        cv2.rectangle(cv_image, (bar_x_start, bar_y), (bar_x_start + fill_width, bar_y + bar_height), 
                      COLOR_VOLUME, -1)
    
    icon = "|))" if vol > 50 else "|)" if vol > 0 else "|"
    text = f"{icon} {vol}%"

    cv_image = draw_text_pillow(
        cv_image,
        text=text,
        position=(SCREEN_CENTER[0], 750),
        font_size=26,
        color_bgr=COLOR_TEXT,
        font_path=CYRILLIC_FONT_PATH,
        anchor="center"
    )
    return cv_image


def draw_cover_with_mask(cv_image: np.ndarray, cover_img: np.ndarray | None):
    if cover_img is None:
        cv2.circle(cv_image, COVER_CENTER, COVER_RADIUS, COLOR_BTN_NORMAL, -1)
        cv2.circle(cv_image, COVER_CENTER, COVER_RADIUS, COLOR_BORDER, 3)
        # ASCII-заглушка вместо эмодзи
        cv_image = draw_text_pillow(
            cv_image,
            text="♪",  # ASCII-нота вместо 🎵
            position=(COVER_CENTER[0], COVER_CENTER[1] + 15),
            font_size=60,
            color_bgr=COLOR_TEXT,
            font_path=CYRILLIC_FONT_PATH,
            anchor="center"
        )
        return cv_image

    cover_resized = cv2.resize(cover_img, (COVER_RADIUS * 2, COVER_RADIUS * 2))
    mask = np.zeros((COVER_RADIUS * 2, COVER_RADIUS * 2), dtype=np.uint8)
    cv2.circle(mask, (COVER_RADIUS, COVER_RADIUS), COVER_RADIUS, 255, -1)
    cover_masked = cv2.bitwise_and(cover_resized, cover_resized, mask=mask)
    
    y1, y2 = COVER_CENTER[1] - COVER_RADIUS, COVER_CENTER[1] + COVER_RADIUS
    x1, x2 = COVER_CENTER[0] - COVER_RADIUS, COVER_CENTER[0] + COVER_RADIUS
    
    roi = cv_image[y1:y2, x1:x2]
    roi_masked = cv2.bitwise_and(roi, roi, mask=cv2.bitwise_not(mask))
    cv_image[y1:y2, x1:x2] = cv2.add(roi_masked, cover_masked)
    
    cv2.circle(cv_image, COVER_CENTER, COVER_RADIUS, COLOR_BORDER, 3)
    return cv_image


def load_cover_image(url: str) -> np.ndarray | None:
    try:
        import requests
        response = requests.get(url, timeout=5)
        if response.status_code == 200:
            img_array = np.frombuffer(response.content, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            return img if img is not None else None
    except Exception:
        pass
    return None


def create_base_frame() -> np.ndarray:
    cv_image = np.zeros((SCREEN_RESOLUTION[1], SCREEN_RESOLUTION[0], 3), dtype=np.uint8)
    for r in range(SAFE_RADIUS, 540):
        alpha = (r - SAFE_RADIUS) / (540 - SAFE_RADIUS)
        color_val = int(30 * alpha)
        cv2.circle(cv_image, SCREEN_CENTER, r, (color_val, color_val, color_val), 1)
    return cv_image


def cv2_to_ros_image(cv_image: np.ndarray, frame_id: str = "screen") -> ROSImage:
    height, width = cv_image.shape[:2]
    ros_img = ROSImage()
    ros_img.header = Header()
    ros_img.header.frame_id = frame_id
    ros_img.height = height
    ros_img.width = width
    ros_img.encoding = "bgr8"
    ros_img.is_bigendian = 0
    ros_img.step = width * 3
    ros_img.data = cv_image.tobytes()
    return ros_img

def is_internet_available():
    """Быстрая проверка наличия интернета через DNS Яндекса."""
    try:
        # Пытаемся подключиться к DNS Яндекса на порт 53 за 2 секунды
        socket.create_connection(("77.88.8.8", 53), timeout=2.0)
        return True
    except OSError:
        return False

def run(
    controller: RoboheadController, action_name: str, cancel_event: threading.Event
):
    """Основной цикл действия"""
    logger = controller.get_logger()
    logger.info(f"[{action_name}] start")

    env_path = Path.home() / '.env'
    load_dotenv(dotenv_path=env_path, override=True)
    TOKEN = os.getenv('YANDEX_MUSIC_TOKEN')
    
    # === FIX: СБРОС ГЛОБАЛЬНОГО СОСТОЯНИЯ ПРИ КАЖДОМ ЗАПУСКЕ ===
    global cur_ind, exit_requested, touch_command, pressed_button
    cur_ind = 0
    exit_requested = False
    touch_command = None
    pressed_button = None
    # ===========================================================

    if not is_internet_available():
        controller.get_logger().error("Яндекс.Музыка: Отсутствует подключение к интернету!")
        controller.silero_tts.say(cancel_event=cancel_event, text="Отсутствует подключение к интернету", block=True)
        return

    if not TOKEN:
        controller.get_logger().error(f"Яндекс.Музыка: Токен не найден в файле {env_path}!")
        controller.silero_tts.say(cancel_event=cancel_event, text="Токен не найден в файле окружения", block=True)
        return

    try:
        controller.get_logger().info("Авторизация в Яндекс.Музыке...")
        client = Client(TOKEN).init()
        controller.get_logger().info(f"Успешный вход! Аккаунт: {client.me.account.login}")
        
    except UnauthorizedError:
        controller.get_logger().error("Яндекс.Музыка: Ошибка авторизации! Неверный или просроченный токен.")
        controller.silero_tts.say(cancel_event=cancel_event, text="Ошибка авторизации! Неверный или просроченный токен.", block=True)
        return
        
    except YandexMusicError as e:
        controller.get_logger().error(f"Яндекс.Музыка: Сбой API Яндекса: {str(e)}")
        controller.silero_tts.say(cancel_event=cancel_event, text="Сбой сервисов Яндекса", block=True)

        return
        
    except Exception as e:
        controller.get_logger().error(f"Яндекс.Музыка: Непредвиденная ошибка: {str(e)}")
        controller.silero_tts.say(cancel_event=cancel_event, text="Непредвиденная ошибка", block=True)
        return

    my_ind = 0
    tracks = client.users_likes_tracks()
    
    # Регистрируем колбэк
    controller.media_driver.register_touch_callback(on_touch)
    
    # Получаем начальную громкость
    try:
        current_volume = controller.media_driver.get_volume(cancel_event=cancel_event)
    except Exception:
        current_volume = 50

    try:  # === FIX: try/finally для гарантированной очистки ===
        while my_ind < len(tracks) and not cancel_event.is_set() and not exit_requested:
            track_data = tracks[my_ind].fetch_track()
            track_id = track_data['id']
            
            download_info = client.tracks_download_info(track_id)
            best_stream = max(download_info, key=lambda x: x.bitrate_in_kbps)
            audio_url = best_stream.get_direct_link()
            
            base_uri = track_data['cover_uri']
            image_url = f"https://{base_uri.replace('%%', '400x400')}"
            cover_img = load_cover_image(image_url)
            
            # Воспроизведение аудио
            controller.media_driver.play_audio(
                cancel_event=cancel_event,
                audio_path=audio_url,
                loop=False,
                block=False
            )
            
            # Цикл отрисовки интерфейса
            while (not controller.media_driver.is_idle_audio(cancel_event=cancel_event) 
                   and not cancel_event.is_set() 
                   and not exit_requested):
                
                # Синхронизация громкости
                try:
                    current_volume = controller.media_driver.get_volume(cancel_event=cancel_event)
                except Exception:
                    pass
                    
                cv_image = create_base_frame()
                cv_image = draw_cover_with_mask(cv_image, cover_img)
                
                artist_name = track_data.artists[0].name if track_data.artists else 'Unknown'
                title = f"{track_data.title} • {artist_name}"
                
                cv_image = draw_text_pillow(
                    cv_image,
                    text=title,
                    position=(SCREEN_CENTER[0], COVER_CENTER[1] - COVER_RADIUS - 30),
                    font_size=32,
                    color_bgr=COLOR_TITLE,
                    font_path=CYRILLIC_FONT_PATH,
                    anchor="center"
                )
                
                cv_image = draw_round_button(cv_image, BTN_PREV, pressed_button == 'prev')
                cv_image = draw_round_button(cv_image, BTN_VOL_DOWN, pressed_button == 'vol_down')
                cv_image = draw_round_button(cv_image, BTN_VOL_UP, pressed_button == 'vol_up')
                cv_image = draw_round_button(cv_image, BTN_NEXT, pressed_button == 'next')
                cv_image = draw_round_button(cv_image, BTN_EXIT, pressed_button == 'exit')
                cv_image = draw_volume_control(cv_image, current_volume)
                
                out_msg = cv2_to_ros_image(cv_image, frame_id="screen")
                controller.media_driver.stream_publish(out_msg)
                
                # Обработка команд
                if touch_command == 'exit':
                    # Явно останавливаем аудио
                    try:
                        controller.media_driver.stop_audio(cancel_event=cancel_event)
                    except Exception:
                        pass
                    break
                elif touch_command == 'prev':
                    try:
                        controller.media_driver.stop_audio(cancel_event=cancel_event)
                    except Exception:
                        pass
                    my_ind = max(0, my_ind - 1)
                    cur_ind = my_ind
                    break
                elif touch_command == 'next':
                    try:
                        controller.media_driver.stop_audio(cancel_event=cancel_event)
                    except Exception:
                        pass
                    my_ind = min(len(tracks) - 1, my_ind + 1)
                    cur_ind = my_ind
                    break
                elif touch_command == 'vol_up':
                    vol = controller.media_driver.get_volume(cancel_event=cancel_event)
                    controller.media_driver.set_volume(
                        cancel_event=cancel_event, 
                        volume=min(VOLUME_MAX, vol + VOLUME_STEP)
                    )
                elif touch_command == 'vol_down':
                    vol = controller.media_driver.get_volume(cancel_event=cancel_event)
                    controller.media_driver.set_volume(
                        cancel_event=cancel_event, 
                        volume=max(VOLUME_MIN, vol - VOLUME_STEP)
                    )
                
                touch_command = None
                pressed_button = None
                controller.sleep(cancel_event=cancel_event, duration=0.05)
            
            if exit_requested or cancel_event.is_set() or touch_command == 'exit':
                break
            
            if touch_command not in ['prev', 'next']:
                my_ind += 1
                cur_ind = my_ind
            touch_command = None

    finally:  # === FIX: гарантированная очистка ===
        logger.info(f"[{action_name}] cleanup")
        # Останавливаем аудио (на всякий случай)
        try:
            controller.media_driver.stop_audio(cancel_event=cancel_event)
        except Exception:
            pass
        # Отписываем колбэк
        try:
            controller.media_driver.unregister_touch_callback(on_touch)
        except Exception:
            pass
        # Очищаем состояние тача
        try:
            controller.media_driver.clear_touch_state()
        except Exception:
            pass
        # Сбрасываем глобальные переменные (дублируем для надёжности)
        exit_requested = False
        touch_command = None
        pressed_button = None

    # Финальный экран
    cv_image = create_base_frame()
    cv_image = draw_text_pillow(
        cv_image,
        text="Yandex Music",
        position=(SCREEN_CENTER[0], SCREEN_CENTER[1]),
        font_size=48,
        color_bgr=COLOR_TEXT,
        font_path=CYRILLIC_FONT_PATH,
        anchor="center"
    )
    out_msg = cv2_to_ros_image(cv_image, frame_id="screen")
    controller.media_driver.stream_publish(out_msg)
    
    logger.info(f"[{action_name}] finish")