from __future__ import annotations
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from ..controller import RoboheadController
    from rclpy.subscription import Subscription

from sensor_msgs.msg import BatteryState


class SensorDriverConnector:

    def __init__(self, controller: RoboheadController):
        self.controller: RoboheadController = controller

        self._sub_battery_state: Optional[Subscription] = None
        self._battery_state_msg: Optional[BatteryState] = None

    def connect(self) -> bool:
        """
        Подключение к топикам.

        Returns:
            out (bool): True if successfull, False - else
        """

        sub_battery_state_name: str = str(
            self.controller.declare_parameter("sensor_driver.topic_name", "dflt").value
        )

        self._sub_battery_state = self.controller.create_subscription(
            BatteryState,
            sub_battery_state_name,
            self._sub_battery_state_callback,
            1,
        )

        self.controller.get_logger().info("sensor_driver connected")
        return True

    def _sub_battery_state_callback(self, msg: BatteryState) -> None:
        self._battery_state_msg = msg

    @property
    def battery_state(self) -> Optional[BatteryState]:
        """
        Возвращает последнее полученное сообщение о состоянии батареи.
        Доступно только для чтения.
        """
        return self._battery_state_msg

    @property
    def battery_voltage(self) -> Optional[float]:
        """
        Возвращает текущее напряжение батареи.
        Доступно только для чтения.
        """
        if self._battery_state_msg is not None:
            return float(self._battery_state_msg.voltage)
        return None

    @property
    def battery_current(self) -> Optional[float]:
        """
        Возвращает текущий ток батареи.
        Доступно только для чтения.
        """
        if self._battery_state_msg is not None:
            return float(self._battery_state_msg.current)
        return None

    @property
    def battery_power_supply_status(self) -> Optional[int]:
        """
        Возвращает текущий статус источника питания.
        Доступно только для чтения.
        """
        if self._battery_state_msg is not None:
            return int(self._battery_state_msg.power_supply_status)
        return None

    @property
    def battery_power_supply_technology(self) -> Optional[int]:
        """
        Возвращает тип технологии источника питания.
        Доступно только для чтения.
        """
        if self._battery_state_msg is not None:
            return int(self._battery_state_msg.power_supply_technology)
        return None
