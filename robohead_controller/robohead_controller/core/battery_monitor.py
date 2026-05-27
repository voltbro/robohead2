from __future__ import annotations
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from ..controller import RoboheadController
    from rclpy.timer import Timer

from sensor_msgs.msg import BatteryState



class BatteryMonitor:
    """Мониторинг состояния батареи и реакция на низкое напряжение."""

    def __init__(self, controller: RoboheadController):
        self.controller: RoboheadController = controller
        self.controller.is_allow_work = False

        self._current_status = None

        self._timer: Optional[Timer] = self.controller.create_timer(
            0.5, self._battery_monitor_tick
        )  # 2 раза в секунду

    def _battery_monitor_tick(self) -> None:
        """Обработчик обновления батареи (вызывается таймером)."""

        voltage: Optional[float] = self.controller.sensor_driver.battery_voltage

        # Ждём первого сообщения от сенсора
        if voltage is None:
            return
        
        if self._current_status is None:
            self._current_status = self.controller.sensor_driver.battery_power_supply_status
        elif self._current_status != self.controller.sensor_driver.battery_power_supply_status:
            self._current_status = self.controller.sensor_driver.battery_power_supply_status
            if self.controller.sensor_driver.battery_power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
                self.controller.action_manager.execute_action("system_charging", None, False)
            elif self.controller.sensor_driver.battery_power_supply_status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING:
                self.controller.action_manager.execute_action("system_discharging", None, False)

        low_threshold: float = self.controller.low_voltage_threshold
        recover_threshold: float = (
            self.controller.low_voltage_threshold
            + self.controller.low_voltage_hysteresis
        )
        is_allow_work: bool = self.controller.is_allow_work
        current_action: Optional[str] = (
            self.controller.action_manager._current_action_name
        )

        # Напряжение упало ниже порога — разряд батареи
        if voltage < low_threshold and is_allow_work:
            self.controller.get_logger().error(
                f"LOW BATTERY! voltage={voltage:.2f}V < threshold={low_threshold:.2f}V. "
                "Triggering std_low_bat action..."
            )
            self.controller.is_allow_work = False
            self.controller.action_manager.execute_action("std_low_bat", None, True)

        # Напряжение восстановилось выше порога с гистерезисом
        elif not is_allow_work and voltage >= recover_threshold:
            self.controller.get_logger().info(
                f"Battery recovered! voltage={voltage:.2f}V >= threshold={recover_threshold:.2f}V. "
                "Starting std_wait..."
            )
            self.controller.is_allow_work = True
            if current_action not in ("std_wait", "std_startup"):
                self.controller.action_manager.execute_action("std_wait", None, True)
