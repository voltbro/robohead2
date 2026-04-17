#!/usr/bin/env python3
import time
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from robohead_interfaces.msg import TouchEvent


Color = Tuple[int, int, int]  # BGR


@dataclass
class Finger:
    slot: int
    tracking_id: int = -1
    x: int = 0   # pixel coords in 0..img_size-1
    y: int = 0
    state: str = "up"  # "down"|"move"|"up"
    color: Color = (255, 0, 0)  # blue by default (BGR)
    last_update: float = 0.0


class TouchscreenVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("touchscreen_visualizer")

        # Topics
        self.touch_topic = self.declare_parameter(
            "touch_topic", "/media_driver/touchscreen"
        ).value
        self.stream_topic = self.declare_parameter(
            "stream_topic", "/media_driver/stream"
        ).value

        # Image
        self.img_size = int(self.declare_parameter("img_size", 1080).value)
        self.circle_radius = int(self.declare_parameter("circle_radius", 60).value)
        self.publish_hz = float(self.declare_parameter("publish_hz", 30.0).value)

        # Touch coordinate range (raw values in TouchEvent)
        # По твоим логам это примерно 0..4096
        self.in_min = int(self.declare_parameter("input_min", 0).value)
        self.in_max = int(self.declare_parameter("input_max", 4096).value)

        self.fingers: Dict[int, Finger] = {}  # slot -> Finger

        self.sub = self.create_subscription(
            TouchEvent, self.touch_topic, self.on_touch, 10
        )
        self.pub = self.create_publisher(Image, self.stream_topic, 1)

        period = 1.0 / max(1e-6, self.publish_hz)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(f"Sub: {self.touch_topic}")
        self.get_logger().info(f"Pub: {self.stream_topic}")
        self.get_logger().info(
            f"img_size={self.img_size}, input_range=[{self.in_min}..{self.in_max}], hz={self.publish_hz}"
        )

    def scale_to_img(self, v: int) -> int:
        # map [in_min..in_max] -> [0..img_size-1]

        t = (v - self.in_min) / (self.in_max - self.in_min)
        px = int(round(t * (self.img_size - 1)))
        return int(np.clip(px, 0, self.img_size - 1))

    def on_touch(self, msg: TouchEvent) -> None:
        slot = int(msg.slot)

        f = self.fingers.get(slot)
        if f is None:
            f = Finger(slot=slot)
            self.fingers[slot] = f

        f.tracking_id = int(msg.tracking_id)
        f.x = self.scale_to_img(int(msg.x))
        f.y = self.scale_to_img(int(msg.y))
        f.state = str(msg.state)
        f.last_update = time.monotonic()

        # Цвета (BGR)
        if f.state == "down":
            f.color = (0, 0, 255)      # red
        elif f.state == "move":
            f.color = (0, 255, 0)      # green
        elif f.state == "up":
            f.color = (255, 0, 0)      # blue
        else:
            # на случай нестандартных состояний
            f.color = (255, 255, 255)  # white

    def on_timer(self) -> None:
        img = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)

        # рисуем кружочки по слотам
        for slot in sorted(self.fingers.keys()):
            f = self.fingers[slot]

            cv2.circle(img, (f.x, f.y), self.circle_radius, f.color, thickness=-1, lineType=cv2.LINE_AA)
            cv2.circle(img, (f.x, f.y), self.circle_radius, (40, 40, 40), thickness=2, lineType=cv2.LINE_AA)

            label = f"s{f.slot} id{f.tracking_id} {f.state}"
            cv2.putText(
                img,
                label,
                (min(f.x + self.circle_radius + 6, self.img_size - 1), min(f.y + 5, self.img_size - 1)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        self.publish_image(img)

    def publish_image(self, img_bgr: np.ndarray) -> None:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "touchscreen"
        msg.height = img_bgr.shape[0]
        msg.width = img_bgr.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = img_bgr.tobytes()
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TouchscreenVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()