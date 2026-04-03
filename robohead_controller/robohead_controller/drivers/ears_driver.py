from __future__ import annotations
from typing import TYPE_CHECKING, cast, Optional
import threading

if TYPE_CHECKING:
    from ..controller import RoboheadController
    from rclpy.client import Client

from robohead_interfaces.srv import Move, SimpleCommand


class EarsDriverConnector:

    def __init__(self, controller: RoboheadController):
        self.controller: RoboheadController = controller

        self._srv_ears_set_angle: Optional[Client] = None
        self._srv_is_idle: Optional[Client] = None

    def connect(self) -> bool:
        """
        Подключение к сервисам.

        Returns:
            out (bool): True if successfull, False - else
        """

        srv_ears_set_angle_name: str = str(
            self.controller.declare_parameter(
                "ears_driver.srv_ears_set_angle_name", "dflt"
            ).value
        )
        srv_is_idle_name: str = str(
            self.controller.declare_parameter(
                "ears_driver.srv_is_idle_name", "dflt"
            ).value
        )

        self._srv_ears_set_angle = self.controller.create_client(
            Move, srv_ears_set_angle_name
        )
        self._srv_is_idle = self.controller.create_client(
            SimpleCommand, srv_is_idle_name
        )

        while not self._srv_ears_set_angle.wait_for_service(
            timeout_sec=self.controller.wait_timeout
        ):
            self.controller.get_logger().info(
                f"Service {srv_ears_set_angle_name} not available, waiting..."
            )

        while not self._srv_is_idle.wait_for_service(
            timeout_sec=self.controller.wait_timeout
        ):
            self.controller.get_logger().info(
                f"Service {srv_is_idle_name} not available, waiting..."
            )

        self.controller.get_logger().info("ears_driver connected")
        return True

    def is_idle(self, cancel_event: threading.Event) -> int | None:
        """
        Проверяет, находятся ли уши в состоянии покоя (idle).

        Args:
            cancel_event (threading.Event): event преждевременного завершения работы

        Returns:
            int | None: результат проверки (1 - ожидание команды, 0 - идёт движение), None - если выполнение было прервано через cancel_event.
        """
        if self._srv_is_idle is not None:
            req = SimpleCommand.Request()

            future = self._srv_is_idle.call_async(request=req)

            while not future.done() and not cancel_event.is_set():
                self.controller.rate_.sleep()

            raw_result = future.result()
            if raw_result is not None:
                result = cast(SimpleCommand.Response, raw_result)
                return result.data
            return None
        else:
            self.controller.get_logger().error(
                "Try to call ears_driver.is_idle() but ears_driver._srv_is_idle is None. Skip call..."
            )
            return None

    def set_angle(
        self,
        cancel_event: threading.Event,
        left: int = 0,
        right: int = 0,
        duration: float = 1.0,
        block: bool = True,
    ) -> int | None:
        """
        Устанавливает угол поворота ушей.

        Args:
            cancel_event (threading.Event): event преждевременного завершения работы
            left (int): угол поворота левого уха
            right (int): угол поворота правого уха
            duration (float): длительность перемещения в секундах >=0
            block (bool): если True - ожидает завершения движения (вызов завершится, когда будет достигнуто заданное положение или вызвано прерывание по cancel_event)

        Returns:
            int | None: результат выполнения команды, None — если выполнение было прервано через cancel_event.
        """
        if self._srv_ears_set_angle is not None:
            req = Move.Request()
            req.angle_a = int(left)
            req.angle_b = int(right)
            req.duration = float(duration)

            future = self._srv_ears_set_angle.call_async(request=req)

            while not future.done() and not cancel_event.is_set():
                self.controller.rate_.sleep()

            while (
                block and not self.is_idle(cancel_event) and not cancel_event.is_set()
            ):
                self.controller.rate_.sleep()

            raw_result = future.result()
            if raw_result is not None:
                result = cast(Move.Response, raw_result)
                return result.data
            return None
        else:
            self.controller.get_logger().error(
                "Try to call ears_driver.set_angle() but ears_driver._srv_ears_set_angle is None. Skip call..."
            )
            return None
