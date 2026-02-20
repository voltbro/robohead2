import threading
import importlib.util
import json
import traceback

class ActionManager:
    """Управление действиями с полной изоляцией ошибок"""
    
    def __init__(self, controller):
        self.controller = controller
        self._current_action_cancel = None
        self._current_action_thread = None
        self._action_lock = threading.Lock()
        self._current_action_name = None
    
    def execute_action(self, name: str, on_complete=None):
        """Запуск действия с защитой от ошибок и зависаний"""
        # 1. Отменяем текущее действие с таймаутом
        self._cancel_current_action(timeout_sec=1.0)
        
        # 2. Создаём новое событие отмены
        cancel_event = threading.Event()
        
        with self._action_lock:
            self._current_action_cancel = cancel_event
            self._current_action_name = name
        
        # 3. Запускаем действие в изолированном потоке
        thread = threading.Thread(
            target=self._action_wrapper,
            args=(name, cancel_event, on_complete),
            daemon=True,
            name=f"Action-{name}"
        )
        thread.start()
        
        with self._action_lock:
            self._current_action_thread = thread
        
        self.controller.get_logger().info(f"▶ Started action: '{name}'")
    
    def _action_wrapper(self, name: str, cancel_event: threading.Event, on_complete):
        """Обёртка с полной изоляцией ошибок"""
        logger = self.controller.get_logger()
        logger.debug(f"[Action:{name}] Wrapper started")
        
        try:
            # Загрузка модуля действия
            action_path = self.controller.actions_match.get(name)
            if not action_path:
                raise FileNotFoundError(f"Action '{name}' not found in mapping")
            
            spec = importlib.util.spec_from_file_location(name, action_path)
            if spec is None or spec.loader is None:
                raise ImportError(f"Cannot load spec for action '{name}' from {action_path}")
            
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            
            if not hasattr(module, 'run'):
                raise AttributeError(f"Action '{name}' has no 'run' function")
            
            # Выполнение действия
            logger.debug(f"[Action:{name}] Executing run()...")
            module.run(self.controller, name, cancel_event)
            
            logger.info(f"✓ Action '{name}' completed successfully")
            
            # Вызов колбэка при успешном завершении
            if not cancel_event.is_set() and on_complete is not None:
                logger.debug(f"[Action:{name}] Calling on_complete callback")
                try:
                    on_complete()
                except Exception as e:
                    logger.error(f"[Action:{name}] on_complete callback failed: {e}", exc_info=True)
        
        except Exception as e:
            # ЛОГИРУЕМ ОШИБКУ, НО НЕ ПАДАЕМ!
            logger.error(f"✗ Action '{name}' FAILED: {e}", exc_info=True)
            logger.error(f"Traceback:\n{''.join(traceback.format_exception(type(e), e, e.__traceback__))}")
        
        finally:
            # Гарантированная очистка состояния
            self._cleanup_action(name)
            logger.debug(f"[Action:{name}] Wrapper finished")
    
    def _cancel_current_action(self, timeout_sec: float = 1.0):
        """Безопасная отмена текущего действия"""
        with self._action_lock:
            if self._current_action_cancel is not None:
                self.controller.get_logger().info(
                    f"⊘ Cancelling current action '{self._current_action_name}'"
                )
                self._current_action_cancel.set()
            
            # if self._current_action_thread is not None and self._current_action_thread.is_alive():
            #     # Даём потоку время на корректное завершение
            #     self._current_action_thread.join(timeout=timeout_sec)
            #     if self._current_action_thread.is_alive():
            #         self.controller.get_logger().warn(
            #             f"⚠ Action thread did not terminate in {timeout_sec}s (will be abandoned)"
            #         )
    
    def _cleanup_action(self, name: str):
        """Очистка состояния после завершения действия"""
        with self._action_lock:
            # Очищаем ТОЛЬКО если это всё ещё актуальное действие
            if self._current_action_name == name:
                self._current_action_cancel = None
                self._current_action_thread = None
                self._current_action_name = None
                self.controller.get_logger().debug(f"[Action:{name}] State cleaned up")
    
    def execute_action_std_wait(self):
        """Запуск действия ожидания с защитой от рекурсии"""
        # Предотвращаем бесконечную рекурсию при вызове из on_complete
        if self._current_action_name == "std_wait":
            self.controller.get_logger().debug("std_wait already running, skipping")
            return
        
        self.execute_action("std_wait", on_complete=None)