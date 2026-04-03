from __future__ import annotations
from typing import TYPE_CHECKING, Optional
from enum import Enum, auto
import threading

if TYPE_CHECKING:
    from ..controller import RoboheadController
    from rclpy.timer import Timer


class CommanderState(Enum):
    """Состояния командера."""

    WAIT_WAKE_PHRASE = auto()
    WAIT_COMMAND = auto()


class Commander:
    def __init__(self, controller: RoboheadController):
        self.controller: RoboheadController = controller

        self._state: CommanderState = CommanderState.WAIT_WAKE_PHRASE
        self._dummy_cancel_event: threading.Event = threading.Event()

        self._timer: Optional[Timer] = self.controller.create_timer(
            0.01, self._queue_tick
        )  # 100 раз в секунду

    @property
    def state(self) -> CommanderState:
        """
        Возвращает текущее состояние командера.
        Доступно только для чтения.
        """
        return self._state

    def _queue_tick(self) -> None:
        """Обработчик команд (вызывается таймером)."""

        if len(self.controller.queue_fast_commands) > 0:
            fast_command: str = self.controller.queue_fast_commands.pop(0)
            self.controller.action_manager.execute_action(fast_command, None, False)

        if self._state == CommanderState.WAIT_WAKE_PHRASE:
            if len(self.controller.queue_wake_phrases) > 0:
                self._state = CommanderState.WAIT_COMMAND
                self.controller.queue_wake_phrases.clear()

                self.controller.speech_recognizer_asr.set_mode(
                    cancel_event=self._dummy_cancel_event, mode=1
                )
                self.controller.speech_recognizer_kws.set_mode(
                    cancel_event=self._dummy_cancel_event, mode=0
                )
                self.controller.action_manager.execute_action(
                    "std_attention", None, True
                )

        elif self._state == CommanderState.WAIT_COMMAND:
            if len(self.controller.queue_commands) > 0:
                command: str = self.controller.queue_commands.pop(0)
                self.controller.queue_commands.clear()
                self._state = CommanderState.WAIT_WAKE_PHRASE

                self.controller.speech_recognizer_asr.set_mode(
                    cancel_event=self._dummy_cancel_event, mode=0
                )
                self.controller.speech_recognizer_kws.set_mode(
                    cancel_event=self._dummy_cancel_event, mode=1
                )

                if command != self.controller.speech_recognizer_asr.timeout_text:
                    self.controller.action_manager.execute_action(
                        command, "std_wait", True
                    )
                else:
                    self.controller.action_manager.execute_action(
                        "std_wait", None, True
                    )
