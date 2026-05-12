from __future__ import annotations
from typing import TYPE_CHECKING, cast, Optional
import threading

if TYPE_CHECKING:
    from ..controller import TurtlebroController
    from rclpy.client import Client
    from rclpy.publisher import Publisher

from geometry_msgs.msg import Twist


class TurtlebroConnector:

    def __init__(self, controller: TurtlebroController):
        self.controller: TurtlebroController = controller

        self._pub_cmd_vel: Optional[Publisher] = None

    def connect(self) -> bool:
        """
        Подключение к сервисам и топикам.

        Returns:
            out (bool): True if successfull, False - else
        """
        topic_cmd_vel_name: str = str(
            self.controller.declare_parameter(
                "turtlebro_controller.topic_cmd_vel_name", "/cmd_vel"
            ).value)

        self._pub_cmd_vel = self.controller.create_publisher(
            msg_type=Twist, topic=topic_cmd_vel_name, qos_profile=1
        )

        self.controller.get_logger().info("turtlebro_driver connected")
        return True

    def velocity_publish(self, linear_x: float=0.0, angular_z: float=0.0) -> None:
        """
        Публикация скорости.

        Args:
            linear_x (float): скорость линейного перемещения
            angular_z (float): скорость углового вращения
        """
        if self._pub_cmd_vel is not None:
            msg = Twist()
            msg.linear.x = float(linear_x)
            msg.angular.z = float(angular_z)
            self._pub_cmd_vel.publish(msg)
            # self.controller.get_logger().info(
            #     f"Published linear_x: {linear_x}, angular_z: {angular_z}"
            # )
        else:
            self.controller.get_logger().error(
                "Try to call turtlebro_driver.velocity_publish() but turtlebro_driver._pub_cmd_vel is None. Skip call..."
            )