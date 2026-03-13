#!/usr/bin/env python3
import rclpy
from gyrobro_controller.controller import RoboheadController
import time

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = RoboheadController()
        # time.sleep(1)  # Небольшая пауза для инициализации
        
        # Подключение всех драйверов
        controller.connect_all_drivers()
        controller.startup_timer = controller.create_timer(0.1, controller.start)
        
        # Запуск многопоточного исполнителя
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(controller)
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Fatal error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()