#!/usr/bin/env python3
from __future__ import annotations
import sys
import traceback

import rclpy
import os
from rclpy.executors import MultiThreadedExecutor

from brover_controller.controller import BRoverController


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    controller: BRoverController | None = None

    try:
        controller = BRoverController(package_dir=os.path.dirname(os.path.abspath(__file__)))

        # Подключение всех драйверов
        controller.connect_all_drivers()

        # Отложенный запуск контроллера (после первого spin)
        controller._startup_timer = controller.create_timer(0.1, controller.start)

        # Запуск многопоточного исполнителя
        executor = MultiThreadedExecutor()
        executor.add_node(controller)
        executor.spin()

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f"Fatal error: {e}", file=sys.stderr)
        traceback.print_exc()

    finally:
        if controller is not None:
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
