from __future__ import annotations

from typing import Any

import cv2
import numpy as np
from sensor_msgs.msg import Image


def clamp(value: Any, low: int, high: int) -> int:
    """Return *value* as an int constrained to the inclusive range."""
    return max(low, min(high, int(value)))


def parse_color(value: Any, default: tuple[int, int, int] = (255, 255, 255)) -> tuple[int, int, int]:
    """Parse a CSS hex color or RGB-like sequence into an RGB tuple."""
    if isinstance(value, str):
        value = value.strip()
        if value.startswith('#') and len(value) == 7:
            return int(value[1:3], 16), int(value[3:5], 16), int(value[5:7], 16)
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return clamp(value[0], 0, 255), clamp(value[1], 0, 255), clamp(value[2], 0, 255)
    return default


def image_msg_to_bgr(msg: Image) -> np.ndarray | None:
    """Convert common ROS Image encodings into a BGR OpenCV frame.

    The function avoids cv_bridge so the web package stays lightweight and can run
    on the robot with only numpy/opencv installed. Unsupported encodings return
    None instead of raising in the hot camera path.
    """
    enc = (msg.encoding or '').lower()
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    if height <= 0 or width <= 0 or step <= 0:
        return None

    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if data.size < height * step:
        return None

    if enc in ('bgr8', 'rgb8'):
        rows = data[:height * step].reshape((height, step))[:, :width * 3]
        frame = rows.reshape((height, width, 3))
        if enc == 'rgb8':
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame.copy()

    if enc in ('bgra8', 'rgba8'):
        rows = data[:height * step].reshape((height, step))[:, :width * 4]
        frame = rows.reshape((height, width, 4))
        code = cv2.COLOR_RGBA2BGR if enc == 'rgba8' else cv2.COLOR_BGRA2BGR
        return cv2.cvtColor(frame, code)

    if enc in ('mono8', '8uc1'):
        rows = data[:height * step].reshape((height, step))[:, :width]
        return cv2.cvtColor(rows, cv2.COLOR_GRAY2BGR)

    return None


def bgr_to_jpeg(frame: np.ndarray, max_width: int, quality: int) -> bytes | None:
    """Resize and encode a BGR frame as JPEG bytes for MJPEG streaming."""
    height, width = frame.shape[:2]
    if width > max_width:
        scale = max_width / float(width)
        frame = cv2.resize(frame, (max_width, max(1, int(height * scale))), interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    return encoded.tobytes() if ok else None
