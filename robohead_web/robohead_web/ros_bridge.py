from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from robohead_interfaces.msg import AudioData, Color as ColorMsg, ColorArray, TouchEvent
from robohead_interfaces.srv import Color as ColorSrv
from robohead_interfaces.srv import ColorPalette, Move, PlayMedia, SimpleCommand, Speak
from sensor_msgs.msg import BatteryState, Image
from std_msgs.msg import Int32

from .image_utils import bgr_to_jpeg, clamp, image_msg_to_bgr, parse_color


@dataclass(frozen=True)
class DisplayConfig:
    """Rendering limits for the robot display stream."""

    width: int = 1080
    height: int = 1080
    min_publish_interval: float = 0.055
    flush_delay: float = 0.10
    second_flush_delay: float = 0.24


class RoboheadBridge(Node):
    """ROS side of the web interface.

    This node owns all ROS publishers, subscriptions, and service clients. FastAPI
    handlers call its thread-safe methods, while ROS callbacks update compact
    buffers for camera, audio, status, and canvas state.
    """

    def __init__(self) -> None:
        """Initialize the ROS bridge node and set up publishers, subscribers, service clients, and the display canvas state."""
        super().__init__('robohead_web_node')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('camera_jpeg_width', 480)
        self.declare_parameter('camera_jpeg_quality', 58)

        fast_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        default_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pub_display = self.create_publisher(Image, '/robohead/media_driver/stream', fast_qos)
        self.pub_led_manual = self.create_publisher(ColorArray, '/robohead/respeaker_driver/set_color_manual', default_qos)
        self.pub_audio_stream = self.create_publisher(AudioData, '/robohead/media_driver/audio_stream', fast_qos)

        self.neck_client = self.create_client(Move, '/robohead/neck_driver/neck_set_angle')
        self.ears_client = self.create_client(Move, '/robohead/ears_driver/ears_set_angle')
        self.speak_client = self.create_client(Speak, '/robohead/silero_tts/speak')
        self.play_media_client = self.create_client(PlayMedia, '/robohead/media_driver/play_media')
        self.set_volume_client = self.create_client(SimpleCommand, '/robohead/media_driver/set_volume')
        self.led_mode_client = self.create_client(SimpleCommand, '/robohead/respeaker_driver/set_mode')
        self.led_brightness_client = self.create_client(SimpleCommand, '/robohead/respeaker_driver/set_brightness')
        self.led_color_client = self.create_client(ColorSrv, '/robohead/respeaker_driver/set_color_all')
        self.led_palette_client = self.create_client(ColorPalette, '/robohead/respeaker_driver/set_color_palette')

        self.lock = threading.RLock()
        self.canvas_lock = threading.RLock()
        self.latest_battery: BatteryState | None = None
        self.latest_doa: Int32 | None = None
        self.latest_touches: dict[int, TouchEvent] = {}
        self.latest_jpeg: bytes | None = None
        self.camera_seq = 0
        self.latest_audio = b''
        self.audio_seq = 0

        self.display = DisplayConfig()
        self.canvas = np.full((self.display.height, self.display.width, 3), 255, dtype=np.uint8)
        self.canvas_history: deque[dict[str, Any]] = deque(maxlen=3000)
        self.canvas_history.append({'type': 'clear', 'color': '#ffffff'})
        self._last_canvas_publish = 0.0
        self._flush_timer: threading.Timer | None = None
        self._second_flush_timer: threading.Timer | None = None

        self.create_subscription(Image, '/robohead/usb_cam/image_raw', self._on_camera, fast_qos)
        self.create_subscription(BatteryState, '/robohead/sensor_driver/battery', self._on_battery, default_qos)
        self.create_subscription(Int32, '/robohead/respeaker_driver/doa', self._on_doa, default_qos)
        self.create_subscription(AudioData, '/robohead/respeaker_driver/audio/main', self._on_audio, default_qos)
        self.create_subscription(TouchEvent, '/robohead/media_driver/touchscreen', self._on_touch, default_qos)

    @property
    def host(self) -> str:
        """Return the configured host address for the web server."""
        return str(self.get_parameter('host').value)

    @property
    def port(self) -> int:
        """Return the configured port number for the web server."""
        return int(self.get_parameter('port').value)

    def _on_camera(self, msg: Image) -> None:
        """Receive a ROS camera image, convert it to JPEG, and cache it for browser streaming."""
        frame = image_msg_to_bgr(msg)
        if frame is None:
            return
        jpeg = bgr_to_jpeg(
            frame,
            max_width=int(self.get_parameter('camera_jpeg_width').value),
            quality=int(self.get_parameter('camera_jpeg_quality').value),
        )
        if jpeg is None:
            return
        with self.lock:
            self.latest_jpeg = jpeg
            self.camera_seq += 1

    def _on_battery(self, msg: BatteryState) -> None:
        """Store the latest battery state from the robot."""
        with self.lock:
            self.latest_battery = msg

    def _on_doa(self, msg: Int32) -> None:
        """Store the latest direction-of-arrival sensor value."""
        with self.lock:
            self.latest_doa = msg

    def _on_audio(self, msg: AudioData) -> None:
        """Store the latest streamed audio packet from the robot."""
        data = np.asarray(msg.data, dtype=np.int16).tobytes()
        with self.lock:
            self.latest_audio = data
            self.audio_seq += 1

    def _on_touch(self, msg: TouchEvent) -> None:
        """Track the latest touchscreen touch events from the robot."""
        with self.lock:
            if msg.state == 'up':
                self.latest_touches.pop(int(msg.tracking_id), None)
            else:
                self.latest_touches[int(msg.tracking_id)] = msg

    def status(self) -> dict[str, Any]:
        """Return a compact status snapshot for the browser."""
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

    def camera_frame(self, last_seq: int) -> tuple[int, bytes | None]:
        """Return the latest encoded JPEG if it is newer than *last_seq*."""
        with self.lock:
            if self.camera_seq == last_seq:
                return last_seq, None
            return self.camera_seq, self.latest_jpeg

    def audio_chunk(self, last_seq: int) -> tuple[int, bytes]:
        """Return the latest microphone PCM chunk if it is newer than *last_seq*."""
        with self.lock:
            if self.audio_seq == last_seq:
                return last_seq, b''
            return self.audio_seq, self.latest_audio

    def canvas_history_snapshot(self) -> list[dict[str, Any]]:
        """Return a copy of the current canvas command history."""
        with self.canvas_lock:
            return list(self.canvas_history)

    def clear_canvas(self, color_value: Any = '#ffffff', *, flush: bool = True) -> dict[str, Any]:
        """Clear the square robot display canvas and publish the new background."""
        color = parse_color(color_value)
        cmd = {'type': 'clear', 'color': f'#{color[0]:02x}{color[1]:02x}{color[2]:02x}'}
        with self.canvas_lock:
            self.canvas[:, :] = color
            self.canvas_history.clear()
            self.canvas_history.append(cmd)
        self.publish_canvas(force=True, repeat=flush)
        return cmd

    def draw_line(self, payload: dict[str, Any], *, flush: bool = False) -> dict[str, Any]:
        """Draw a line segment on the server canvas and publish it to the robot."""
        color = parse_color(payload.get('color', '#111111'), default=(17, 17, 17))
        size = clamp(payload.get('size', 12), 1, 80)
        x0 = clamp(payload.get('x0', 0), 0, self.display.width - 1)
        y0 = clamp(payload.get('y0', 0), 0, self.display.height - 1)
        x1 = clamp(payload.get('x1', 0), 0, self.display.width - 1)
        y1 = clamp(payload.get('y1', 0), 0, self.display.height - 1)
        thickness = max(1, size * 2)
        cmd = {
            'type': 'draw',
            'x0': x0,
            'y0': y0,
            'x1': x1,
            'y1': y1,
            'color': f'#{color[0]:02x}{color[1]:02x}{color[2]:02x}',
            'size': size,
        }
        with self.canvas_lock:
            cv2.line(self.canvas, (x0, y0), (x1, y1), color, thickness=thickness, lineType=cv2.LINE_AA)
            self.canvas_history.append(cmd)
        self.publish_canvas(force=flush, repeat=flush)
        if not flush:
            self.schedule_canvas_flush()
        return cmd

    def publish_canvas(self, *, force: bool = False, repeat: bool = False) -> None:
        """Publish the current canvas with rate limiting and optional repeats.

        media_driver currently reloads images through mpv from the same temporary
        filename. Repeating a final frame makes the display robust against a lost
        or stale final load without flooding the stream while the user draws.
        """
        now = time.monotonic()
        if not force and now - self._last_canvas_publish < self.display.min_publish_interval:
            return
        self._last_canvas_publish = now
        self._publish_canvas_once()
        if repeat:
            self._schedule_republish(self.display.flush_delay, second=False)
            self._schedule_republish(self.display.second_flush_delay, second=True)

    def schedule_canvas_flush(self) -> None:
        """Schedule a delayed canvas republish to ensure the final frame is shown."""
        self._schedule_republish(self.display.flush_delay, second=False)

    def _schedule_republish(self, delay: float, *, second: bool) -> None:
        """Start a timer to republish the canvas after a short delay."""
        attr = '_second_flush_timer' if second else '_flush_timer'
        old = getattr(self, attr)
        if old is not None:
            old.cancel()
        timer = threading.Timer(delay, self._publish_canvas_once)
        timer.daemon = True
        setattr(self, attr, timer)
        timer.start()

    def _publish_canvas_once(self) -> None:
        """Convert the current canvas into a ROS Image message and publish it."""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = self.display.height
        msg.width = self.display.width
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = self.display.width * 3
        with self.canvas_lock:
            msg.data = self.canvas.tobytes()
        self.pub_display.publish(msg)

    def send_neck(self, vertical: Any, horizontal: Any, duration: Any = 0.06) -> bool:
        """Send a neck movement request to the robot if the service is available."""
        if not self.neck_client.service_is_ready():
            return False
        req = Move.Request()
        req.angle_a = clamp(vertical, -30, 30)
        req.angle_b = clamp(horizontal, -30, 30)
        req.duration = float(duration)
        self.neck_client.call_async(req)
        return True

    def publish_client_audio(self, pcm_bytes: bytes) -> None:
        """Publish browser microphone audio as a ROS AudioData message."""
        if not pcm_bytes:
            return
        msg = AudioData()
        msg.data = np.frombuffer(pcm_bytes, dtype=np.int16).astype(np.int16).tolist()
        self.pub_audio_stream.publish(msg)

    def publish_client_video_frame(self, packet: bytes) -> bool:
        """Decode a browser JPEG frame packet and publish it to the display."""
        if len(packet) < 12 or packet[:4] != b'VID\0':
            return False
        width = int.from_bytes(packet[4:8], 'little')
        height = int.from_bytes(packet[8:12], 'little')
        if width <= 0 or height <= 0:
            return False
        frame = cv2.imdecode(np.frombuffer(packet[12:], dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return False
        if frame.shape[1] != self.display.width or frame.shape[0] != self.display.height:
            frame = cv2.resize(frame, (self.display.width, self.display.height), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = self.display.height
        msg.width = self.display.width
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = self.display.width * 3
        msg.data = rgb.tobytes()
        self.pub_display.publish(msg)
        return True
