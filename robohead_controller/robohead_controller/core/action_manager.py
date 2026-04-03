from __future__ import annotations
from typing import TYPE_CHECKING, Optional
import threading
import importlib.util
import traceback
import os

if TYPE_CHECKING:
    from ..controller import RoboheadController
    from types import ModuleType


class CachedModule:
    """Кэшированный модуль с отслеживанием времени модификации."""

    __slots__ = ("module", "mtime", "path")

    def __init__(self, module: ModuleType, mtime: float, path: str):
        self.module: ModuleType = module
        self.mtime: float = mtime
        self.path: str = path

    def is_stale(self) -> bool:
        """Проверяет, изменился ли файл с момента загрузки."""
        try:
            current_mtime = os.path.getmtime(self.path)
            return current_mtime > self.mtime
        except OSError:
            # Файл удалён или недоступен — считаем устаревшим
            return True


class ActionManager:
    """Управление действиями с полной изоляцией ошибок и умным кэшированием."""

    def __init__(self, controller: RoboheadController):
        self.controller: RoboheadController = controller

        self._current_action_cancel: Optional[threading.Event] = None
        self._current_action_thread: Optional[threading.Thread] = None
        self._current_action_name: Optional[str] = None
        self._action_lock: threading.Lock = threading.Lock()

        # Кэш с отслеживанием изменений файлов
        self._module_cache: dict[str, CachedModule] = {}

    def preload_all_actions(self) -> None:
        """
        Предварительная загрузка всех действий в кэш при инициализации.
        Ускоряет первый вызов каждого действия.
        """
        logger = self.controller.get_logger()
        logger.info("Preloading all actions into cache...")

        total = len(self.controller.actions_match)
        loaded = 0
        failed = 0

        for name in self.controller.actions_match.keys():
            try:
                self._load_action_module(name)
                loaded += 1
                logger.debug(f"[Preload] OK: '{name}'")
            except Exception as e:
                failed += 1
                logger.warning(f"[Preload] Error: '{name}': {e}")

        logger.info(
            f"Preload complete: {loaded}/{total} actions loaded, {failed} failed"
        )

    def execute_action(
        self,
        name: str,
        on_complete: Optional[str] = None,
        cancelling: bool = True,
    ) -> None:
        """
        Запуск действия с защитой от ошибок и зависаний.

        Args:
            name (str): имя действия для выполнения
            on_complete (Optional[str]): имя действия, которое будет запущено после успешного завершения
            cancelling (bool): если True — отменяет текущее выполняющееся действие перед запуском нового
        """
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
            args=(name, cancel_event, on_complete),
            daemon=True,
            name=f"Action-{name}",
        )
        thread.start()

        with self._action_lock:
            self._current_action_thread = thread

        self.controller.get_logger().info(f"[action_manager] Started action: '{name}'")

    def _load_action_module(self, name: str) -> ModuleType:
        """
        Загружает модуль действия с умным кэшированием.
        Автоматически перезагружает при изменении файла.

        Args:
            name (str): имя действия

        Returns:
            ModuleType: загруженный модуль

        Raises:
            FileNotFoundError: если действие не найдено в маппинге
            ImportError: если не удалось загрузить модуль
            AttributeError: если в модуле нет функции 'run'
        """
        action_path: Optional[str] = self.controller.actions_match.get(name)
        if not action_path:
            raise FileNotFoundError(f"Action '{name}' not found in mapping")

        # Проверяем кэш
        cached = self._module_cache.get(name)
        if cached is not None and cached.path == action_path and not cached.is_stale():
            self.controller.get_logger().debug(
                f"[action_manager] Using cached module Action:{name}"
            )
            return cached.module

        # Перезагрузка (файл изменился или не в кэше)
        if cached is not None and cached.is_stale():
            self.controller.get_logger().info(
                f"[action_manager] File (Action:{name}) changed, reloading..."
            )

        # Загрузка модуля
        spec = importlib.util.spec_from_file_location(name, action_path)
        if spec is None or spec.loader is None:
            raise ImportError(
                f"Cannot load spec for action '{name}' from {action_path}"
            )

        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        if not hasattr(module, "run"):
            raise AttributeError(f"Action '{name}' has no 'run' function")

        # Сохраняем в кэш с временем модификации
        mtime = os.path.getmtime(action_path)
        self._module_cache[name] = CachedModule(module, mtime, action_path)

        return module

    def _action_wrapper(
        self,
        name: str,
        cancel_event: threading.Event,
        on_complete: Optional[str] = None,
    ) -> None:
        """
        Обёртка с полной изоляцией ошибок.

        Args:
            name (str): имя действия
            cancel_event (threading.Event): событие для отмены действия
            on_complete (Optional[str]): имя действия для выполнения после завершения
        """
        logger = self.controller.get_logger()
        logger.debug(f"[action_manager] Wrapper (Action:{name}) started")

        try:
            # Загрузка модуля (с умным кэшированием)
            module = self._load_action_module(name)

            # Выполнение действия
            logger.debug("[action_manager] Executing run()...")
            module.run(self.controller, name, cancel_event)

            logger.info(f"[action_manager] Action '{name}' completed successfully")

            # Запуск on_complete действия (рекурсивно, в том же потоке)
            if not cancel_event.is_set() and on_complete is not None:
                self._action_wrapper(on_complete, cancel_event, None)

        except FileNotFoundError as e:
            logger.error(f"Err: Action '{name}' NOT FOUND: {e}")

        except ImportError as e:
            logger.error(f"Err: Action '{name}' IMPORT ERROR: {e}")

        except AttributeError as e:
            logger.error(f"Err: Action '{name}' INVALID MODULE: {e}")

        except Exception as e:
            # ЛОГИРУЕМ ОШИБКУ, НО НЕ ПАДАЕМ!
            logger.error(f"Err: Action '{name}' FAILED: {e}")
            logger.error(
                f"Traceback:\n{''.join(traceback.format_exception(type(e), e, e.__traceback__))}"
            )

        finally:
            # Гарантированная очистка состояния
            self._cleanup_action(name)
            logger.debug(f"[Action:{name}] Wrapper finished")

    def _cancel_current_action(self, timeout_sec: float = 1.0) -> None:
        """
        Безопасная отмена текущего действия.

        Args:
            timeout_sec (float): максимальное время ожидания завершения потока
        """
        with self._action_lock:
            cancel_event = self._current_action_cancel
            thread = self._current_action_thread
            action_name = self._current_action_name

        if cancel_event is not None:
            self.controller.get_logger().info(
                f"Cancelling current action '{action_name}'"
            )
            cancel_event.set()

        if thread is not None and thread.is_alive():
            # Даём потоку время на корректное завершение
            thread.join(timeout=timeout_sec)
            if thread.is_alive():
                self.controller.get_logger().warning(
                    f"Err: Action thread did not terminate in {timeout_sec}s (will be abandoned)"
                )

    def _cleanup_action(self, name: str) -> None:
        """
        Очистка состояния после завершения действия.

        Args:
            name (str): имя завершённого действия
        """
        with self._action_lock:
            # Очищаем ТОЛЬКО если это всё ещё актуальное действие
            if self._current_action_name == name:
                self._current_action_cancel = None
                self._current_action_thread = None
                self._current_action_name = None
                self.controller.get_logger().debug(
                    f"[action_manager] State cleaned up for action {name}"
                )

    def clear_cache(self, name: Optional[str] = None) -> None:
        """
        Очищает кэш модулей.

        Args:
            name (Optional[str]): имя конкретного действия для удаления из кэша.
                Если None — очищается весь кэш.
        """
        if name is None:
            self._module_cache.clear()
            self.controller.get_logger().info("Module cache cleared")
        elif name in self._module_cache:
            del self._module_cache[name]
            self.controller.get_logger().info(f"Module '{name}' removed from cache")
