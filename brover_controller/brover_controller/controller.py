from robohead_controller.controller import RoboheadController

from .drivers.brover_driver import BRoverConnector


class BRoverController(RoboheadController):
    def __init__(self, package_dir:str):
        super().__init__(package_dir=package_dir)

        self._brover_driver: BRoverConnector = BRoverConnector(self)

    def connect_all_drivers(self) -> None:
        super().connect_all_drivers()

        self._brover_driver.connect()

    @property
    def brover_driver(self) -> BRoverConnector:
        """Драйвер BRover."""
        return self._brover_driver
