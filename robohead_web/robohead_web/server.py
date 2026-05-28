from __future__ import annotations

import asyncio
import os
import signal
import threading
from pathlib import Path
from typing import Any

from aiohttp import WSMsgType, web
from aiohttp.client_exceptions import ClientConnectionResetError
import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robohead_interfaces.msg import AudioData, Color as ColorMsg
from robohead_interfaces.msg import ColorArray, TouchEvent
from robohead_interfaces.srv import Color as ColorSrv
from robohead_interfaces.srv import ColorPalette, Move, PlayMedia, SimpleCommand, Speak
from sensor_msgs.msg import BatteryState, Image
from std_msgs.msg import Int32

import json  # Добавьте, если ещё нет

import logging
logger = logging.getLogger(__name__)


class RoboheadWebNode(Node):
    def __init__(self) -> None:
        super().__init__('robohead_web_node')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)

        self.pub_display = self.create_publisher(Image, '/robohead/media_driver/stream', 1)
        self.pub_led_manual = self.create_publisher(ColorArray, '/robohead/respeaker_driver/set_color_manual', 10)

        self.neck_client = self.create_client(Move, '/robohead/neck_driver/neck_set_angle')
        self.ears_client = self.create_client(Move, '/robohead/ears_driver/ears_set_angle')
        self.speak_client = self.create_client(Speak, '/robohead/silero_tts/speak')
        self.play_media_client = self.create_client(PlayMedia, '/robohead/media_driver/play_media')
        self.set_volume_client = self.create_client(SimpleCommand, '/robohead/media_driver/set_volume')
        self.led_mode_client = self.create_client(SimpleCommand, '/robohead/respeaker_driver/set_mode')
        self.led_brightness_client = self.create_client(SimpleCommand, '/robohead/respeaker_driver/set_brightness')
        self.led_color_client = self.create_client(ColorSrv, '/robohead/respeaker_driver/set_color_all')
        self.led_palette_client = self.create_client(ColorPalette, '/robohead/respeaker_driver/set_color_palette')

        self.latest_camera: Image | None = None
        self.latest_battery: BatteryState | None = None
        self.latest_doa: Int32 | None = None
        self.latest_audio: bytes = b''
        self.audio_seq = 0
        self.latest_touches: dict[int, TouchEvent] = {}
        self.lock = threading.Lock()

        self.create_subscription(Image, '/robohead/usb_cam/image_raw', self.on_camera, 1)
        self.create_subscription(BatteryState, '/robohead/sensor_driver/battery', self.on_battery, 10)
        self.create_subscription(Int32, '/robohead/respeaker_driver/doa', self.on_doa, 10)
        self.create_subscription(AudioData, '/robohead/respeaker_driver/audio/main', self.on_audio, 20)
        self.create_subscription(TouchEvent, '/robohead/media_driver/touchscreen', self.on_touch, 10)

        self.canvas_width = 1080
        self.canvas_height = 1080
        self.canvas = np.full((self.canvas_height, self.canvas_width, 3), 255, dtype=np.uint8)
        self.canvas_lock = threading.Lock()

        self.canvas_history: list[dict] = []
        self.max_history_len = 3000  # Лимит команд, чтобы не забить память
        self.history_lock = threading.Lock()
        self.ws_clients: set[web.WebSocketResponse] = set()  # Список активных соединений

    def broadcast(self, message: dict, exclude: web.WebSocketResponse = None) -> None:
        """
        Рассылает сообщение всем клиентам, кроме 'exclude'
        """
        dead_clients = set()
        msg_str = json.dumps(message) # Сериализуем один раз для скорости
        
        for ws in self.ws_clients:
            if ws == exclude:
                continue
            if ws.closed:
                dead_clients.add(ws)
                continue
            
            try:
                ws.send_str(msg_str)
            except RuntimeError:
                dead_clients.add(ws)
        
        # Чистим список от отключившихся клиентов
        if dead_clients:
            self.ws_clients -= dead_clients

    def on_camera(self, msg: Image) -> None:
        with self.lock:
            self.latest_camera = msg

    def on_battery(self, msg: BatteryState) -> None:
        with self.lock:
            self.latest_battery = msg

    def on_doa(self, msg: Int32) -> None:
        with self.lock:
            self.latest_doa = msg

    def on_audio(self, msg: AudioData) -> None:
        data = np.asarray(msg.data, dtype=np.int16).tobytes()
        with self.lock:
            self.latest_audio = data
            self.audio_seq += 1

    def on_touch(self, msg: TouchEvent) -> None:
        with self.lock:
            if msg.state == 'up':
                self.latest_touches.pop(int(msg.tracking_id), None)
            else:
                self.latest_touches[int(msg.tracking_id)] = msg

    def status(self) -> dict[str, Any]:
        with self.lock:
            battery = self.latest_battery
            doa = self.latest_doa
            touches = list(self.latest_touches.values())
        return {
            'battery': None if battery is None else {
                'voltage': round(float(battery.voltage), 3),
                'current': round(float(battery.current), 3),
                'status': int(battery.power_supply_status),
                'present': bool(battery.present),
            },
            'doa': None if doa is None else int(doa.data),
            'touches': [
                {'slot': int(t.slot), 'tracking_id': int(t.tracking_id), 'x': int(t.x), 'y': int(t.y), 'state': str(t.state)}
                for t in touches
            ],
        }

    def handle_video_frame(self, data: bytes) -> None:
        """Обрабатывает бинарный пакет с видео-фреймом от клиента"""
        try:
            import base64
            import numpy as np
            import cv2
            
            # Парсим заголовок: [4 байта magic][4 байта width][4 байта height]
            if len(data) < 12:
                return
                
            magic = data[0:4]
            if magic != b'VID\x00':  # Проверка сигнатуры
                return
            
            width = int.from_bytes(data[4:8], 'little')
            height = int.from_bytes(data[8:12], 'little')
            jpeg_data = data[12:]
            
            if not jpeg_data:
                return
            
            # Декодируем JPEG в numpy array (BGR)
            frame = cv2.imdecode(np.frombuffer(jpeg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                return
            
            # Ресайзим под размер дисплея робота, если нужно
            if frame.shape[1] != self.canvas_width or frame.shape[0] != self.canvas_height:
                frame = cv2.resize(frame, (self.canvas_width, self.canvas_height), interpolation=cv2.INTER_LINEAR)
            
            # Конвертируем BGR → RGB для ROS encoding 'rgb8'
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Создаём ROS Image сообщение
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'client_camera_stream'
            msg.height = self.canvas_height
            msg.width = self.canvas_width
            msg.encoding = 'rgb8'
            msg.is_bigendian = 0
            msg.step = self.canvas_width * 3
            msg.data = frame_rgb.tobytes()
            
            # Публикуем в тот же топик, что и медиа-плеер
            self.pub_display.publish(msg)
            
        except Exception as e:
            self.get_logger().warning(f'video_frame decode error: {e}')

    def audio_chunk(self, last_seq: int) -> tuple[int, bytes]:
        with self.lock:
            seq = self.audio_seq
            data = self.latest_audio
        return seq, data if seq != last_seq else b''

    def camera_jpeg(self, max_width: int = 480, quality: int = 58) -> bytes | None:
        with self.lock:
            msg = self.latest_camera
        if msg is None:
            return None
        try:
            frame = image_msg_to_bgr(msg)
            if frame is None:
                return None
            height, width = frame.shape[:2]
            if width > max_width:
                scale = max_width / float(width)
                frame = cv2.resize(frame, (max_width, int(height * scale)), interpolation=cv2.INTER_AREA)
            ok, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
            return encoded.tobytes() if ok else None
        except Exception as exc:
            self.get_logger().warning(f'camera encode error: {exc}')
            return None

    def publish_canvas(self) -> None:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = self.canvas_height
        msg.width = self.canvas_width
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = self.canvas_width * 3
        with self.canvas_lock:
            msg.data = self.canvas.tobytes()
        self.pub_display.publish(msg)

    def draw_line(self, x0: int, y0: int, x1: int, y1: int, color: tuple[int, int, int], size: int) -> None:
        x0 = clamp(x0, 0, self.canvas_width - 1)
        x1 = clamp(x1, 0, self.canvas_width - 1)
        y0 = clamp(y0, 0, self.canvas_height - 1)
        y1 = clamp(y1, 0, self.canvas_height - 1)
        thickness = max(1, int(size) * 2)
        
        with self.canvas_lock:
            cv2.line(self.canvas, (x0, y0), (x1, y1), color, thickness=thickness, lineType=cv2.LINE_AA)
        self.publish_canvas()
        
        # 📝 Запись команды в историю
        cmd = {
            "type": "draw",
            "x0": x0, "y0": y0, "x1": x1, "y1": y1,
            "color": f"#{color[0]:02x}{color[1]:02x}{color[2]:02x}",
            "size": size
        }
        with self.history_lock:
            self.canvas_history.append(cmd)
            if len(self.canvas_history) > self.max_history_len:
                del self.canvas_history[:len(self.canvas_history) - self.max_history_len]

    def clear_canvas(self, color: tuple[int, int, int] = (255, 255, 255)) -> None:
        with self.canvas_lock:
            self.canvas[:, :] = color
        self.publish_canvas()
        
        #  Очистка истории и запись команды сброса
        hex_color = f"#{color[0]:02x}{color[1]:02x}{color[2]:02x}"
        with self.history_lock:
            self.canvas_history.clear()
            self.canvas_history.append({"type": "clear", "color": hex_color})


class WebApp:
    def __init__(self, node: RoboheadWebNode) -> None:
        self.node = node
        self.app = web.Application(client_max_size=8 * 1024 * 1024)
        self.app.add_routes([
            web.get('/', self.index),
            web.get('/app.js', self.static_file('app.js', 'application/javascript')),
            web.get('/styles.css', self.static_file('styles.css', 'text/css')),
            web.get('/camera.mjpg', self.camera_mjpg),
            web.get('/audio_ws', self.audio_ws),
            web.get('/ws', self.ws),
            web.get('/api/status', self.status),
            web.post('/api/neck', self.neck),
            web.post('/api/ears', self.ears),
            web.post('/api/tts', self.tts),
            web.post('/api/volume', self.volume),
            web.post('/api/media', self.media),
            web.post('/api/led/mode', self.led_mode),
            web.post('/api/led/brightness', self.led_brightness),
            web.post('/api/led/color', self.led_color),
            web.post('/api/led/palette', self.led_palette),
            web.post('/api/led/manual', self.led_manual),
            web.post('/api/draw/clear', self.draw_clear),
            web.post('/api/media/upload', self.media_upload),
        ])
        self.app.on_startup.append(self._start_sync)
        self.app.on_cleanup.append(self._stop_sync)


    async def media_upload(self, request: web.Request) -> web.Response:
        """Загрузка медиафайла и немедленное воспроизведение (исправленная версия)"""
        try:
            reader = await request.multipart()
            
            file_data = None
            filename = None
            loop = False  # По умолчанию зацикливание выключено
            
            # 🔄 Перебираем ВСЕ поля формы (порядок не важен)
            async for field in reader:
                if field.name == 'file':
                    file_data = await field.read(decode=False)
                    filename = field.filename or 'uploaded_media'
                    
                elif field.name == 'loop':
                    loop_value = await field.text()
                    loop = loop_value.lower() in ('true', '1', 'yes')
            
            # Валидация: файл обязателен
            if not file_data or not filename:
                return web.json_response({'ok': False, 'error': 'no_file_received'}, status=400)
            
            # Определяем расширение из имени файла
            ext = Path(filename).suffix.lower()
            if not ext or ext == '.bin':
                # Fallback: пробуем определить по Content-Type из заголовков
                content_type = field.headers.get('Content-Type', '') if 'field' in locals() else ''
                if 'video' in content_type: ext = '.mp4'
                elif 'image' in content_type: ext = '.jpg'
                elif 'audio' in content_type: ext = '.mp3'
                else: ext = '.bin'
            
            # Сохранение файла
            upload_dir = Path('/tmp/robohead_media')
            upload_dir.mkdir(parents=True, exist_ok=True)
            
            saved_path = upload_dir / f'media{ext}'
            
            with open(saved_path, 'wb') as f:
                f.write(file_data)
            
            # ✅ Логируем через правильный логгер
            logger.info(f'✅ Uploaded: {saved_path} ({len(file_data)} bytes), loop={loop}')
            
            # 🎬 Запуск воспроизведения через ROS сервис
            req = PlayMedia.Request()
            req.path_to_video_file = str(saved_path)
            req.path_to_audio_file = str(saved_path)  # Для видео со звуком
            req.loop = loop
            
            result = await self.call(self.node.play_media_client, req, timeout=10.0)
            
            return web.json_response({
                'ok': True,
                'path': str(saved_path),
                'loop': loop,
                'size': len(file_data)
            })
            
        except Exception as e:
            import traceback
            traceback.print_exc()
            # ✅ Логируем ошибку через правильный логгер
            logger.error(f'❌ Media upload error: {e}')
            
            # Возвращаем валидный JSON даже при ошибке
            return web.json_response({'ok': False, 'error': str(e)}, status=500)

    async def _start_sync(self, app):
        self._sync_task = asyncio.create_task(self._canvas_sync_loop())

    async def _stop_sync(self, app):
        if self._sync_task:
            self._sync_task.cancel()
            try: await self._sync_task
            except asyncio.CancelledError: pass

    async def _canvas_sync_loop(self):
        while True:
            await asyncio.sleep(2.0)
            if not self.node.ws_clients:
                continue
            
            # Берём копию под блокировкой, чтобы не конфликтовать с ROS-потоком
            with self.node.history_lock:
                if not self.node.canvas_history:
                    continue
                payload = json.dumps({"type": "canvas_history", "commands": self.node.canvas_history})
            
            # Рассылаем всем подключённым клиентам
            disconnected = set()
            for client in self.node.ws_clients:
                try:
                    if not client.closed:
                        await client.send_str(payload)
                except Exception:
                    disconnected.add(client)
            for dead in disconnected:
                self.node.ws_clients.discard(dead)

    def static_dir(self) -> Path:
        here = Path(__file__).resolve().parent / 'static'
        if here.exists():
            return here
        for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(':'):
            path = Path(prefix) / 'share' / 'robohead_web' / 'static'
            if path.exists():
                return path
        return here

    async def index(self, _request: web.Request) -> web.Response:
        return web.FileResponse(self.static_dir() / 'index.html')

    def static_file(self, name: str, content_type: str):
        async def handler(_request: web.Request) -> web.Response:
            return web.FileResponse(self.static_dir() / name, headers={'Content-Type': content_type})
        return handler

    async def camera_mjpg(self, request: web.Request) -> web.StreamResponse:
        response = web.StreamResponse(
            status=200,
            reason='OK',
            headers={
                'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
                'Cache-Control': 'no-store, no-cache, must-revalidate, max-age=0',
                'Pragma': 'no-cache',
            },
        )
        await response.prepare(request)
        try:
            while True:
                jpeg = self.node.camera_jpeg()
                if jpeg is not None:
                    chunk = b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n'
                    await response.write(chunk)
                await asyncio.sleep(0.05)
        except (asyncio.CancelledError, ConnectionResetError, ClientConnectionResetError, RuntimeError):
            pass
        return response

    async def audio_ws(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        await ws.send_json({'type': 'config', 'sampleRate': 16000})
        last_seq = -1
        try:
            while not ws.closed:
                seq, chunk = self.node.audio_chunk(last_seq)
                if chunk:
                    last_seq = seq
                    await ws.send_bytes(chunk)
                await asyncio.sleep(0.01)
        except (asyncio.CancelledError, ConnectionResetError, ClientConnectionResetError, RuntimeError):
            pass
        return ws

    async def status(self, _request: web.Request) -> web.Response:
        return web.json_response(self.node.status())

    async def ws(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        
        # 1. Регистрируем клиента
        self.node.ws_clients.add(ws)
        
        # 2. Отправляем полную историю новому клиенту
        with self.node.history_lock:
            if self.node.canvas_history:
                # Отправляем одним пакетом
                await ws.send_json({
                    'type': 'canvas_history', 
                    'commands': self.node.canvas_history
                })

        sender = asyncio.create_task(self.ws_sender(ws))
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    # Передаем ws в обработчик, чтобы знать, кого исключить при рассылке
                    await self.handle_ws_message(msg.json(), sender_ws=ws)
                elif msg.type == WSMsgType.BINARY:
                    # Передаём сырые байты в обработчик
                    self.node.handle_video_frame(msg.data)
        finally:
            sender.cancel()
            self.node.ws_clients.discard(ws)  # Удаляем при отключении
        return ws

    async def ws_sender(self, ws: web.WebSocketResponse) -> None:
        while not ws.closed:
            await ws.send_json({'type': 'status', 'data': self.node.status()})
            await asyncio.sleep(0.5)

    async def handle_ws_message(self, data: dict[str, Any], sender_ws: web.WebSocketResponse) -> None:
        kind = data.get('type')
        
        if kind == 'draw':
            # 1. Рисуем на сервере (ROS) и сохраняем в историю
            self.node.draw_line(
                data.get('x0', 0), data.get('y0', 0), 
                data.get('x1', 0), data.get('y1', 0), 
                parse_color(data.get('color', '#111111')), 
                int(data.get('size', 12))
            )
            
            # 2. Рассылаем всем, КРОМЕ того, кто прислал (sender_ws)
            self.node.broadcast(data, exclude=sender_ws)

        elif kind == 'clear':
            # 1. Очищаем на сервере
            self.node.clear_canvas(parse_color(data.get('color', '#ffffff')))
            
            # 2. Рассылаем команду очистки всем остальным
            self.node.broadcast(data, exclude=sender_ws)

        elif kind == 'neck':
            # Шею не рассылываем, это локальное управление
            self.send_neck(data.get('vertical', 0), data.get('horizontal', 0), data.get('duration', 0.08))

    async def body(self, request: web.Request) -> dict[str, Any]:
        try:
            return await request.json()
        except Exception:
            return {}

    async def call(self, client, request_msg, timeout: float = 4.0) -> web.Response:
        if not client.service_is_ready():
            await asyncio.to_thread(client.wait_for_service, timeout_sec=0.2)
        if not client.service_is_ready():
            return web.json_response({'ok': False, 'error': 'service_unavailable'}, status=503)
        future = client.call_async(request_msg)
        start = asyncio.get_running_loop().time()
        while not future.done():
            if asyncio.get_running_loop().time() - start > timeout:
                return web.json_response({'ok': False, 'error': 'timeout'}, status=504)
            await asyncio.sleep(0.02)
        result = future.result()
        return web.json_response({'ok': True, 'data': None if result is None else int(getattr(result, 'data', 0))})

    def send_neck(self, vertical: Any, horizontal: Any, duration: Any = 0.08) -> bool:
        if not self.node.neck_client.service_is_ready():
            return False
        req = Move.Request()
        req.angle_a = clamp(vertical, -30, 30)
        req.angle_b = clamp(horizontal, -30, 30)
        req.duration = float(duration)
        self.node.neck_client.call_async(req)
        return True

    async def neck(self, request: web.Request) -> web.Response:
        data = await self.body(request)
        ok = self.send_neck(data.get('vertical', 0), data.get('horizontal', 0), data.get('duration', 0.08))
        return web.json_response({'ok': ok})

    async def ears(self, request: web.Request) -> web.Response:
        data = await self.body(request)
        req = Move.Request()
        req.angle_a = clamp(data.get('left', 0), -90, 90)
        req.angle_b = clamp(data.get('right', 0), -90, 90)
        req.duration = float(data.get('duration', 0.25))
        return await self.call(self.node.ears_client, req)

    async def tts(self, request: web.Request) -> web.Response:
        data = await self.body(request)
        voices = {'aidar': 0, 'baya': 1, 'kseniya': 2, 'eugene': 3, 'xenia': 4}
        req = Speak.Request()
        req.text = str(data.get('text', ''))
        voice = data.get('voice', 'eugene')
        req.voice = int(voices.get(str(voice), voice if isinstance(voice, int) else 3))
        req.path_to_save = str(data.get('path_to_save', ''))
        req.put_accent = bool(data.get('put_accent', True))
        req.put_yo = bool(data.get('put_yo', True))
        req.play = bool(data.get('play', True))
        return await self.call(self.node.speak_client, req, timeout=20.0)

    async def volume(self, request: web.Request) -> web.Response:
        data = await self.body(request)
        req = SimpleCommand.Request()
        req.data = clamp(data.get('value', 60), 0, 100)
        return await self.call(self.node.set_volume_client, req)

    async def media(self, request: web.Request) -> web.Response:
        data = await self.body(request)
        req = PlayMedia.Request()
        req.path_to_video_file = str(data.get('video_path', ''))
        req.path_to_audio_file = str(data.get('audio_path', ''))
        req.loop = bool(data.get('loop', False))
        return await self.call(self.node.play_media_client, req)

    async def led_mode(self, request: web.Request) -> web.Response:
        data = await self.body(request)
        req = SimpleCommand.Request()
        req.data = int(data.get('mode', 0))
        return await self.call(self.node.led_mode_client, req)

    async def led_brightness(self, request: web.Request) -> web.Response:
        data = await self.body(request)
        req = SimpleCommand.Request()
        value = clamp(data.get('value', 80), 0, 100)
        req.data = int(255 * value / 100)
        return await self.call(self.node.led_brightness_client, req)

    async def led_color(self, request: web.Request) -> web.Response:
        data = await self.body(request)
        r, g, b = parse_color(data.get('color', '#24d18b'))
        req = ColorSrv.Request()
        req.red, req.green, req.blue = r, g, b
        return await self.call(self.node.led_color_client, req)

    async def led_palette(self, request: web.Request) -> web.Response:
        data = await self.body(request)
        a = parse_color(data.get('color_a', '#24d18b'))
        b = parse_color(data.get('color_b', '#ff4d6d'))
        req = ColorPalette.Request()
        req.color_a.red, req.color_a.green, req.color_a.blue = a
        req.color_b.red, req.color_b.green, req.color_b.blue = b
        return await self.call(self.node.led_palette_client, req)

    async def led_manual(self, request: web.Request) -> web.Response:
        data = await self.body(request)
        msg = ColorArray()
        msg.colors = []
        for raw in data.get('colors', [])[:12]:
            r, g, b = parse_color(raw)
            color = ColorMsg()
            color.red, color.green, color.blue = r, g, b
            msg.colors.append(color)
        while len(msg.colors) < 12:
            msg.colors.append(ColorMsg())
        self.node.pub_led_manual.publish(msg)
        return web.json_response({'ok': True})

    async def draw_clear(self, request: web.Request) -> web.Response:
        data = await self.body(request)
        self.node.clear_canvas(parse_color(data.get('color', '#ffffff')))
        return web.json_response({'ok': True})


def image_msg_to_bgr(msg: Image) -> np.ndarray | None:
    enc = (msg.encoding or '').lower()
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if height <= 0 or width <= 0:
        return None
    if enc in ('bgr8', 'rgb8'):
        rows = data[:height * step].reshape((height, step))[:, :width * 3]
        frame = rows.reshape((height, width, 3))
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if enc == 'rgb8' else frame
    if enc in ('bgra8', 'rgba8'):
        rows = data[:height * step].reshape((height, step))[:, :width * 4]
        frame = rows.reshape((height, width, 4))
        return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR) if enc == 'rgba8' else cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    if enc in ('mono8', '8uc1'):
        rows = data[:height * step].reshape((height, step))[:, :width]
        return cv2.cvtColor(rows, cv2.COLOR_GRAY2BGR)
    return None


def parse_color(value: Any) -> tuple[int, int, int]:
    if isinstance(value, str):
        value = value.strip()
        if value.startswith('#') and len(value) == 7:
            return int(value[1:3], 16), int(value[3:5], 16), int(value[5:7], 16)
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return clamp(value[0], 0, 255), clamp(value[1], 0, 255), clamp(value[2], 0, 255)
    return 255, 255, 255


def clamp(value: Any, low: int, high: int) -> int:
    return max(low, min(high, int(value)))


def main() -> None:
    rclpy.init()
    node = RoboheadWebNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    host = str(node.get_parameter('host').value)
    port = int(node.get_parameter('port').value)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    async def start() -> None:
        runner = web.AppRunner(WebApp(node).app)
        await runner.setup()
        runner_redirect = None

        import ssl
        ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        use_ssl = False
        
        try:
            ssl_context.load_cert_chain('ssl/cert.pem', 'ssl/key.pem')
            use_ssl = True
            node.get_logger().info(' SSL certificates loaded. HTTPS enabled.')
        except FileNotFoundError:
            node.get_logger().warning('⚠️ cert.pem or key.pem not found. Running HTTP only.')

        https_port = port
        site_https = web.TCPSite(runner, host, https_port, ssl_context=ssl_context if use_ssl else None)
        await site_https.start()
        
        protocol = 'https' if use_ssl else 'http'
        node.get_logger().info(f' Main server running at {protocol}://{host}:{https_port}')

        # 🔄 Если HTTPS активен, поднимаем HTTP-редирект
        if use_ssl:
            redirect_app = web.Application()
            
            # 🟢 Используем явную регистрацию вместо декоратора
            async def redirect_to_https(request):
                client_host = request.headers.get('Host', f'{host}:{https_port}').split(':')[0]
                target = f"https://{client_host}:{https_port}{request.path_qs}"
                return web.HTTPFound(target)
            
            # Регистрируем основные методы
            redirect_app.router.add_get('/{path:.*}', redirect_to_https)
            redirect_app.router.add_post('/{path:.*}', redirect_to_https)
            redirect_app.router.add_put('/{path:.*}', redirect_to_https)
            redirect_app.router.add_delete('/{path:.*}', redirect_to_https)
            # redirect_app.router.add_head('/{path:.*}', redirect_to_https)
            redirect_app.router.add_options('/{path:.*}', redirect_to_https)
            redirect_app.router.add_patch('/{path:.*}', redirect_to_https)

            runner_redirect = web.AppRunner(redirect_app)
            await runner_redirect.setup()
            
            http_port = port + 1
            site_http = web.TCPSite(runner_redirect, host, http_port)
            await site_http.start()
            node.get_logger().info(f'🔀 HTTP redirect server running at http://{host}:{http_port} -> {protocol}://{host}:{https_port}')

        stop = asyncio.Event()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(sig, stop.set)
            except NotImplementedError:
                pass
        await stop.wait()

        await runner.cleanup()
        if runner_redirect:
            await runner_redirect.cleanup()

    try:
        loop.run_until_complete(start())
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
