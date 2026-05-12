from robohead_controller.controller import RoboheadController

from .drivers.turtlebro_driver import TurtlebroConnector


class TurtlebroController(RoboheadController):
    def __init__(self, package_dir:str):
        super().__init__(package_dir=package_dir)

        self._turtlebro_driver: TurtlebroConnector = TurtlebroConnector(self)

    def connect_all_drivers(self) -> None:
        super().connect_all_drivers()

        self._turtlebro_driver.connect()

    @property
    def turtlebro_driver(self) -> TurtlebroConnector:
        """Драйвер BRover."""
        return self._turtlebro_driver
