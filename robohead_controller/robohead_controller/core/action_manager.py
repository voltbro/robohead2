import threading
import importlib.util
import json

class ActionManager:
    # Управление пользовательскими действиями
    
    def __init__(self, controller):
        self.controller = controller
        self._current_action_cancel = None
        self._action_lock = threading.Lock()
    
    def execute_action(self, name: str, on_complete=None):
        """Запуск действия в отдельном потоке с поддержкой отмены"""
        with self._action_lock:
            if self._current_action_cancel is not None:
                self._current_action_cancel.set()
            self._current_action_cancel = threading.Event()
            cancel_event = self._current_action_cancel
        
        try:
            action_path = self.controller.actions_match.get(name)
            if not action_path:
                self.controller.get_logger().error(f"Action '{name}' not found in mapping")
                return

            spec = importlib.util.spec_from_file_location(name, action_path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            
            if not hasattr(module, 'run'):
                self.controller.get_logger().error(f"Action {name} has no 'run' function")
                return
            
            thread = threading.Thread(
                target=module.run,
                args=(self.controller, name, cancel_event, on_complete),
                daemon=True,
                name=f"Action-{name}"
            )
            thread.start()
            self.controller.get_logger().info(f"Started action: {name}")
            
        except Exception as e:
            self.controller.get_logger().error(f"Failed to start action {name}: {e}", exc_info=True)
    
    def on_action_complete(self):
        """Колбэк завершения действия (вызывается из действий)"""
        self.controller.get_logger().info("Action completed → starting std_wait")
        self.execute_action("std_wait", on_complete=None)