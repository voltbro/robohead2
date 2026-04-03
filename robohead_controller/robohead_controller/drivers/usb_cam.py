from __future__ import annotations
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from ..controller import RoboheadController
    from rclpy.subscription import Subscription

from sensor_msgs.msg import Image


class UsbCamConnector:

    def __init__(self, controller: RoboheadController):
        self.controller: RoboheadController = controller

        self._sub_image_raw: Optional[Subscription] = None
        self._image_raw_msg: Optional[Image] = None

    def connect(self) -> bool:
        """
        Подключение к топикам.

        Returns:
            out (bool): True if successfull, False - else
        """

        sub_image_raw_name: str = str(
            self.controller.declare_parameter(
                "usb_cam.topic_name_image_raw", "dflt"
            ).value
        )

        self._sub_image_raw = self.controller.create_subscription(
            Image, sub_image_raw_name, self._sub_image_raw_callback, 1
        )

        self.controller.get_logger().info("usb_cam connected")
        return True

    def _sub_image_raw_callback(self, msg: Image) -> None:
        self._image_raw_msg = msg

    @property
    def image_raw(self) -> Optional[Image]:
        """
        Возвращает последнее полученное сырое изображение с USB-камеры.
        Доступно только для чтения.
        """
        return self._image_raw_msg
