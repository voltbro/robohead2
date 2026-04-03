from __future__ import annotations
from typing import TYPE_CHECKING, cast, Optional
import threading

if TYPE_CHECKING:
    from ..controller import RoboheadController
    from rclpy.client import Client
    from rclpy.subscription import Subscription
    from rclpy.publisher import Publisher

from robohead_interfaces.srv import SimpleCommand, ColorPalette
from robohead_interfaces.srv import Color as ColorSrv
from robohead_interfaces.msg import AudioData, ColorArray
from robohead_interfaces.msg import Color as ColorMsg
from std_msgs.msg import Int32


class RespeakerDriverConnector:
    def __init__(self, controller: RoboheadController):
        self.controller: RoboheadController = controller

        self._sub_audio_main: Optional[Subscription] = None
        self._audio_main_msg: Optional[AudioData] = None
        # self._sub_audio_channel_0 = None
        # self._sub_audio_channel_1 = None
        # self._sub_audio_channel_2 = None
        # self._sub_audio_channel_3 = None
        # self._sub_audio_channel_4 = None
        # self._sub_audio_channel_5 = None
        self._sub_doa: Optional[Subscription] = None
        self._doa_msg: Optional[Int32] = None

        self.pub_set_color_manual: Optional[Publisher] = None

        self._srv_set_brightness: Optional[Client] = None
        self._srv_set_color_all: Optional[Client] = None
        self._srv_set_color_palette: Optional[Client] = None
        self._srv_set_mode: Optional[Client] = None
        self._respeaker_type: str = "XVF3800"  # TODO autoselect

    def connect(self) -> bool:
        """
        Подключение к сервисам и топикам.

        Returns:

            out (bool): True if successfull, False - else
        """

        sub_audio_main_name = str(
            self.controller.declare_parameter(
                "respeaker_driver.topic_name.audio_main", "dflt"
            ).value
        )

        sub_doa_name = str(
            self.controller.declare_parameter(
                "respeaker_driver.topic_name.doa", "dflt"
            ).value
        )
        pub_set_color_manual_name = str(
            self.controller.declare_parameter(
                "respeaker_driver.topic_name.set_color_manual", "dflt"
            ).value
        )

        srv_set_brightness_name = str(
            self.controller.declare_parameter(
                "respeaker_driver.service_name.set_brightness", "dflt"
            ).value
        )
        srv_set_color_all_name = str(
            self.controller.declare_parameter(
                "respeaker_driver.service_name.set_color_all", "dflt"
            ).value
        )
        srv_set_color_palette_name = str(
            self.controller.declare_parameter(
                "respeaker_driver.service_name.set_color_palette", "dflt"
            ).value
        )
        srv_set_mode_name = str(
            self.controller.declare_parameter(
                "respeaker_driver.service_name.set_mode", "dflt"
            ).value
        )

        self.pub_set_color_manual = self.controller.create_publisher(
            msg_type=ColorArray, topic=pub_set_color_manual_name, qos_profile=10
        )

        self._srv_set_brightness = self.controller.create_client(
            SimpleCommand, srv_set_brightness_name
        )
        self._srv_set_color_all = self.controller.create_client(
            ColorSrv, srv_set_color_all_name
        )
        self._srv_set_color_palette = self.controller.create_client(
            ColorPalette, srv_set_color_palette_name
        )
        self._srv_set_mode = self.controller.create_client(
            SimpleCommand, srv_set_mode_name
        )

        self._sub_audio_main = self.controller.create_subscription(
            AudioData, sub_audio_main_name, self._sub_audio_main_callback, 1
        )
        self._sub_doa = self.controller.create_subscription(
            Int32, sub_doa_name, self.sub_doa_callback, 1
        )

        while not self._srv_set_brightness.wait_for_service(
            timeout_sec=self.controller.wait_timeout
        ):
            self.controller.get_logger().info(
                f"Service {srv_set_brightness_name} not available, waiting..."
            )
        while not self._srv_set_color_all.wait_for_service(
            timeout_sec=self.controller.wait_timeout
        ):
            self.controller.get_logger().info(
                f"Service {srv_set_color_all_name} not available, waiting..."
            )
        while not self._srv_set_color_palette.wait_for_service(
            timeout_sec=self.controller.wait_timeout
        ):
            self.controller.get_logger().info(
                f"Service {srv_set_color_palette_name} not available, waiting..."
            )
        while not self._srv_set_mode.wait_for_service(
            timeout_sec=self.controller.wait_timeout
        ):
            self.controller.get_logger().info(
                f"Service {srv_set_mode_name} not available, waiting..."
            )

        self.controller.get_logger().info("respeaker_driver connected")
        return True

    def _sub_audio_main_callback(self, msg: AudioData) -> None:
        self._audio_main_msg = msg

    @property
    def audio_main(self) -> list:
        """
        Возвращает текущий полученный звук с канала audio_main микрофона ReSpeaker.
        Доступно только для чтения.
        """
        if self._audio_main_msg is not None:
            return list(self._audio_main_msg.data)
        return list()

    def sub_doa_callback(self, msg: Int32):
        self._doa_msg = msg

    @property
    def doa(self) -> int:
        """
        Возвращает текущее направление источника звука (Direction of Arrival).
        Доступно только для чтения.
        """
        if self._doa_msg is not None:
            return int(self._doa_msg.data)
        return 0

    def set_led_mode(self, cancel_event: threading.Event, mode: int) -> int | None:
        """
        Устанавливает режим работы LED-кольца у микрофона ReSpeaker

        Args:
            cancel_event (threading.Event): event преждевременного завершения работы
            mode (int): режим работы от 0 до 5

        Returns:
            int | None: 0 если успешно, -1 при ошибке. None - если выполнение было прервано через cancel_event.
        """
        if self._srv_set_mode is not None:
            req = SimpleCommand.Request()
            req.data = mode

            future = self._srv_set_mode.call_async(request=req)

            while not future.done() and not cancel_event.is_set():
                self.controller.rate_.sleep()

            raw_result = future.result()
            if raw_result is not None:
                result = cast(SimpleCommand.Response, raw_result)
                return result.data
            return None
        else:
            self.controller.get_logger().error(
                "Try to call respeaker_driver.set_led_mode() but respeaker_driver._srv_set_mode is None. Skip call..."
            )

    def set_led_brightness(
        self, cancel_event: threading.Event, value: int
    ) -> int | None:
        """
        Устанавливает яркость LED-кольца у микрофона ReSpeaker

        Args:
            cancel_event (threading.Event): event преждевременного завершения работы
            value (int): Значение яркости от 0 до 100

        Returns:
            int | None: 0 если успешно, -1 при ошибке. None - если выполнение было прервано через cancel_event.
        """
        if self._srv_set_brightness is not None:
            req = SimpleCommand.Request()

            if self._respeaker_type == "XVF3800":
                req.data = int(255 * value / 100)
            elif self._respeaker_type == "XVF3000":
                req.data = int(32 * value / 100)
            else:
                return None

            future = self._srv_set_brightness.call_async(request=req)

            while not future.done() and not cancel_event.is_set():
                self.controller.rate_.sleep()

            raw_result = future.result()
            if raw_result is not None:
                result = cast(SimpleCommand.Response, raw_result)
                return result.data
            return None
        else:
            self.controller.get_logger().error(
                "Try to call respeaker_driver.set_led_brightness() but respeaker_driver._srv_set_brightness is None. Skip call..."
            )

    def set_led_color_all(
        self, cancel_event: threading.Event, red: int, green: int, blue: int
    ) -> int | None:
        """
        Устанавливает одинаковый цвет всем светодиодам LED-кольца у микрофона ReSpeaker

        Args:
            cancel_event (threading.Event): event преждевременного завершения работы
            red (int): Канал "Красный", значение от 0 до 255
            green (int): Канал "Зеленый", значение от 0 до 255
            blue (int): Канал "Синий", значение от 0 до 255

        Returns:
            int | None: 0 если успешно, -1 при ошибке. None - если выполнение было прервано через cancel_event.
        """
        if self._srv_set_color_all is not None:
            req = ColorSrv.Request()
            req.red = red
            req.green = green
            req.blue = blue

            future = self._srv_set_color_all.call_async(request=req)

            while not future.done() and not cancel_event.is_set():
                self.controller.rate_.sleep()

            raw_result = future.result()
            if raw_result is not None:
                result = cast(ColorSrv.Response, raw_result)
                return result.data
            return None
        else:
            self.controller.get_logger().error(
                "Try to call respeaker_driver.set_led_color_all() but respeaker_driver._srv_set_color_all is None. Skip call..."
            )

    def set_led_color_palette(
        self,
        cancel_event: threading.Event,
        red_a: int,
        green_a: int,
        blue_a: int,
        red_b: int,
        green_b: int,
        blue_b: int,
    ) -> int | None:
        """
        Устанавливает двухцветную палитру для анимационных режимов LED (DOA, breathe и т.д.)

        Args:
            cancel_event (threading.Event): event преждевременного завершения работы
            red_a (int): Канал "Красный" основного цвета, значение от 0 до 255
            green_a (int): Канал "Зеленый" основного цвета, значение от 0 до 255
            blue_a (int): Канал "Синий" основного цвета, значение от 0 до 255

            red_b (int): Канал "Красный" дополнительного цвета, значение от 0 до 255
            green_b (int): Канал "Зеленый" дополнительного цвета, значение от 0 до 255
            blue_b (int): Канал "Синий" дополнительного цвета, значение от 0 до 255
        Returns:
            int | None: 0 если успешно, -1 при ошибке. None - если выполнение было прервано через cancel_event.
        """
        if self._srv_set_color_palette is not None:
            req = ColorPalette.Request()
            req.color_a.red = red_a
            req.color_a.green = green_a
            req.color_a.blue = blue_a

            req.color_b.red = red_b
            req.color_b.green = green_b
            req.color_b.blue = blue_b

            future = self._srv_set_color_palette.call_async(request=req)

            while not future.done() and not cancel_event.is_set():
                self.controller.rate_.sleep()

            raw_result = future.result()
            if raw_result is not None:
                result = cast(ColorPalette.Response, raw_result)
                return result.data
            return None
        else:
            self.controller.get_logger().error(
                "Try to call respeaker_driver.set_led_color_palette() but respeaker_driver._srv_set_color_palette is None. Skip call..."
            )

    def set_led_color_manual(self, cancel_event: threading.Event, colors: list) -> None:
        """
        Устанавливает каждому (из 12) светодиоду LED-кольца отдельный цвет

        Args:
            cancel_event (threading.Event): event преждевременного завершения работы
            colors (list): массив из 36=12*3 элементов (каждые три элемента - три канала на один светодиод). Каждый элемент: (int) от 0 до 255
        Returns:
            None
        """
        if self.pub_set_color_manual is None:
            self.controller.get_logger().error(
                "Try to call respeaker_driver.set_led_color_manual() but respeaker_driver.pub_set_color_manual is None. Skip call..."
            )
            return None

        if len(colors) != 36:
            self.controller.get_logger().error(
                "Try to call respeaker_driver.set_led_color_manual() but len(colors)!=36. Skip call..."
            )
            return None
        msg = ColorArray()
        msg.colors = list()

        for i in range(12):
            color = ColorMsg()
            color.red = colors[i * 3]
            color.green = colors[i * 3 + 1]
            color.blue = colors[i * 3 + 2]
            msg.colors.append(color)

        self.pub_set_color_manual.publish(msg)
        return None
