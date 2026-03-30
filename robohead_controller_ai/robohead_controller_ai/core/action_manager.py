import threading
import importlib.util
import json
import traceback
import requests

class ActionManager:
    """Управление действиями с полной изоляцией ошибок"""
    
    def __init__(self, controller):
        self.controller = controller
        self._current_action_cancel = None
        self._current_action_thread = None
        self._action_lock = threading.Lock()
        self._current_action_name = None

    def get_nested_attr(self, obj, name):
        for part in name.split('.'):
            obj = getattr(obj, part)
        return obj
    def execute_action_ai(self, command:str, on_complete=None, cancelling:bool=True):
        if cancelling:
            self._cancel_current_action(timeout_sec=1.0)

        # 2. Создаём новое событие отмены
        cancel_event = threading.Event()

        with self._action_lock:
            self._current_action_cancel = cancel_event
            self._current_action_name = "__ACTION_AI__"

        # 3. Запускаем действие в изолированном потоке
        thread = threading.Thread(
            target=self._action_wrapper_ai,
            args=(command, cancel_event,on_complete),
            daemon=True,
            name=f"__ACTION_AI__"
        )
        thread.start()
        
        with self._action_lock:
            self._current_action_thread = thread
        
        self.controller.get_logger().info(f"-> Started action: AI")

    def execute_action(self, name: str, on_complete=None, cancelling:bool=True):
        """Запуск действия с защитой от ошибок и зависаний"""
        # 1. Отменяем текущее действие с таймаутом
        if cancelling:
            self._cancel_current_action(timeout_sec=1.0)
        
        # 2. Создаём новое событие отмены
        cancel_event = threading.Event()
        
        with self._action_lock:
            self._current_action_cancel = cancel_event
            self._current_action_name = name
        
        # 3. Запускаем действие в изолированном потоке
        thread = threading.Thread(
            target=self._action_wrapper,
            args=(name, cancel_event,on_complete),
            daemon=True,
            name=f"Action-{name}"
        )
        thread.start()
        
        with self._action_lock:
            self._current_action_thread = thread
        
        self.controller.get_logger().info(f"-> Started action: '{name}'")

    def process_llm(self, prompt:str) -> dict:

        payload = {
            'model': self.controller.llm_model,
            'prompt': prompt,
            'system': self.controller.llm_system_prompt,
            'stream': False,
            'format': self.controller.llm_format_schema,         
            'think': False,             
            'options': {
                'temperature': self.controller.llm_temperature,
                'num_predict': self.controller.llm_num_predict,
            }
        }
        self.controller.get_logger().info(f"raw llm payload: {payload}")
        response = requests.post(self.controller.llm_url, json=payload, timeout=self.controller.llm_timeout)
        response.raise_for_status()

        result = response.json().get('response', '')
        self.controller.get_logger().info(f"raw llm_answer {result}")
        return json.loads(result)

    def _action_wrapper_ai(self, command:str, cancel_event: threading.Event, on_complete:str=None):
        """Обёртка с полной изоляцией ошибок"""
        logger = self.controller.get_logger()
        logger.debug(f"[Action ai] Wrapper started")

        try:
            llm_answer = self.process_llm(command)
            actions = llm_answer["actions"]
            for action in actions:
                if cancel_event.is_set():
                    return
                func_name = action["function"]
                args = action["args"]
                print(f"Try: {func_name}({args})")

                try:
                    method = self.get_nested_attr(
                        self.controller, func_name)
                except AttributeError:
                    logger.error(
                        f"Method not found: {func_name}")
                    continue

                if callable(method):
                    method(cancel_event, **args)
                else:
                    print("Метод не найден")
                
            logger.info(f"✓ Action AI completed successfully")
            if not cancel_event.is_set() and on_complete != None:
                self._action_wrapper(on_complete, cancel_event, None)
        
        except Exception as e:
            # ЛОГИРУЕМ ОШИБКУ, НО НЕ ПАДАЕМ!
            logger.error(f"✗ Action AI' FAILED: {e}", exc_info=True)
            logger.error(f"Traceback:\n{''.join(traceback.format_exception(type(e), e, e.__traceback__))}")
        
        finally:
            # Гарантированная очистка состояния
            self._cleanup_action("__ACTION_AI__")
            logger.debug(f"[Action:{"__ACTION_AI__"}] Wrapper finished")

    def _action_wrapper(self, name: str, cancel_event: threading.Event, on_complete:str=None):
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
            if not cancel_event.is_set() and on_complete != None:
                self._action_wrapper(on_complete, cancel_event, None)
        
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
            
            if self._current_action_thread is not None and self._current_action_thread.is_alive():
                # Даём потоку время на корректное завершение
                self._current_action_thread.join(timeout=timeout_sec)
                if self._current_action_thread.is_alive():
                    self.controller.get_logger().warn(
                        f"⚠ Action thread did not terminate in {timeout_sec}s (will be abandoned)"
                    )
    
    def _cleanup_action(self, name: str):
        """Очистка состояния после завершения действия"""
        with self._action_lock:
            # Очищаем ТОЛЬКО если это всё ещё актуальное действие
            if self._current_action_name == name:
                self._current_action_cancel = None
                self._current_action_thread = None
                self._current_action_name = None
                self.controller.get_logger().debug(f"[Action:{name}] State cleaned up")