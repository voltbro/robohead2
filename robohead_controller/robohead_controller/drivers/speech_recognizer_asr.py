from __future__ import annotations
from typing import TYPE_CHECKING, cast, Optional
import threading

if TYPE_CHECKING:
    from ..controller import RoboheadController
    from rclpy.client import Client
    from rclpy.subscription import Subscription

from std_msgs.msg import String
from robohead_interfaces.srv import SimpleCommand


class SpeechRecognizerAsrConnector:

    def __init__(self, controller: RoboheadController):
        self.controller: RoboheadController = controller

        self._srv_set_mode: Optional[Client] = None
        self._sub_commands: Optional[Subscription] = None
        self._sub_free: Optional[Subscription] = None
        self._timeout_text: str = "dflt"

    def connect(self) -> bool:
        """
        Подключение к сервисам и топикам.

        Returns:
            out (bool): True if successfull, False - else
        """

        srv_set_mode_name: str = str(
            self.controller.declare_parameter(
                "speech_recognizer_asr.service_name.set_mode", "dflt"
            ).value
        )
        sub_commands_name: str = str(
            self.controller.declare_parameter(
                "speech_recognizer_asr.topic_name.commands", "dflt"
            ).value
        )
        sub_free_name: str = str(
            self.controller.declare_parameter(
                "speech_recognizer_asr.topic_name.free", "dflt"
            ).value
        )

        self._timeout_text = str(
            self.controller.declare_parameter(
                "speech_recognizer_asr.timeout_text", "dflt"
            ).value
        )

        self._sub_free = self.controller.create_subscription(
            String, sub_free_name, self._sub_free_callback, 1
        )
        self._sub_commands = self.controller.create_subscription(
            String, sub_commands_name, self._sub_commands_callback, 1
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

        self.controller.get_logger().info("speech_recognizer_asr connected")
        return True

    def _sub_free_callback(self, msg: String) -> None:
        free: str = msg.data
        self.controller.get_logger().info(f"Free: |{free}| ")
        self.controller.queue_frees.append(free)

    def _sub_commands_callback(self, msg: String) -> None:
        command: str = msg.data
        self.controller.get_logger().info(f"Command: |{command}| ")
        self.controller.queue_commands.append(command)

    @property
    def timeout_text(self) -> str:
        """
        Возвращает текст таймаута распознавания.
        Доступно только для чтения.
        """
        return self._timeout_text

    def set_mode(self, cancel_event: threading.Event, mode: int = 0) -> int | None:
        """
        Устанавливает режим работы распознавателя речи.

        Args:
            cancel_event (threading.Event): event преждевременного завершения работы
            mode (int): режим работы:
                0 — распознавание выключено,
                1 — распознавание команд,
                2 — свободное распознавание

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
                "Try to call speech_recognizer_asr.set_mode() but speech_recognizer_asr._srv_set_mode is None. Skip call..."
            )
            return None
