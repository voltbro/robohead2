from __future__ import annotations
from typing import TYPE_CHECKING, cast, Optional
import threading

if TYPE_CHECKING:
    from ..controller import RoboheadController
    from rclpy.client import Client
    from rclpy.subscription import Subscription

from std_msgs.msg import String
from robohead_interfaces.srv import SimpleCommand


class SpeechRecognizerKwsConnector:

    def __init__(self, controller: RoboheadController):
        self.controller: RoboheadController = controller

        self._srv_set_mode: Optional[Client] = None
        self._sub_fast_commands: Optional[Subscription] = None
        self._sub_wake_phrases: Optional[Subscription] = None

    def connect(self) -> bool:
        """
        Подключение к сервисам и топикам.

        Returns:
            out (bool): True if successfull, False - else
        """

        srv_set_mode_name: str = str(
            self.controller.declare_parameter(
                "speech_recognizer_kws.service_name.set_mode", "dflt"
            ).value
        )
        sub_fast_commands_name: str = str(
            self.controller.declare_parameter(
                "speech_recognizer_kws.topic_name.fast_commands", "dflt"
            ).value
        )
        sub_wake_phrases_name: str = str(
            self.controller.declare_parameter(
                "speech_recognizer_kws.topic_name.wake_phrases", "dflt"
            ).value
        )

        self._sub_fast_commands = self.controller.create_subscription(
            String, sub_fast_commands_name, self._sub_fast_commands_callback, 1
        )
        self._sub_wake_phrases = self.controller.create_subscription(
            String, sub_wake_phrases_name, self._sub_wake_phrases_callback, 1
        )

        self._srv_set_mode = self.controller.create_client(
            srv_type=SimpleCommand,
            srv_name=srv_set_mode_name,
            callback_group=self.controller.reentrant_callback_group,
        )

        while not self._srv_set_mode.wait_for_service(
            timeout_sec=self.controller.wait_timeout
        ):
            self.controller.get_logger().info(
                f"Service {srv_set_mode_name} not available, waiting..."
            )

        self.controller.get_logger().info("speech_recognizer_kws connected")
        return True

    def _sub_fast_commands_callback(self, msg: String) -> None:
        fast_command: str = msg.data
        self.controller.queue_fast_commands.append(fast_command)
        self.controller.get_logger().info(f"fast_command: |{fast_command}| ")

    def _sub_wake_phrases_callback(self, msg: String) -> None:
        wake_phrase: str = msg.data
        self.controller.queue_wake_phrases.append(wake_phrase)
        self.controller.get_logger().info(f"wake_phrase: |{wake_phrase}| ")

    def set_mode(self, cancel_event: threading.Event, mode: int = 0) -> int | None:
        """
        Устанавливает режим работы распознавателя ключевых слов.

        Args:
            cancel_event (threading.Event): event преждевременного завершения работы
            mode (int): режим работы:
                0 — распознавание выключено,
                1 — распознавание включено (wake_phrases + fast_commands)

        Returns:
            int | None: результат выполнения команды, None — если выполнение было прервано через cancel_event.
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
                "Try to call speech_recognizer_kws.set_mode() but speech_recognizer_kws._srv_set_mode is None. Skip call..."
            )
            return None
