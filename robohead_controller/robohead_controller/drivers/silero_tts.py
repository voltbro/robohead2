from __future__ import annotations
from typing import TYPE_CHECKING, cast, Optional
import threading

if TYPE_CHECKING:
    from ..controller import RoboheadController
    from rclpy.client import Client

from robohead_interfaces.srv import Speak


class SileroTTSConnector:

    VOICES: dict[str, int] = {
        "aidar": 0,
        "baya": 1,
        "kseniya": 2,
        "eugene": 3,
        "xenia": 4,
    }

    def __init__(self, controller: RoboheadController):
        self.controller: RoboheadController = controller

        self._srv_speak: Optional[Client] = None

    def connect(self) -> bool:
        """
        Подключение к сервисам.

        Returns:
            out (bool): True if successfull, False - else
        """

        srv_speak_name: str = str(
            self.controller.declare_parameter(
                "silero_tts.srv_speak_name",
                "/robohead_controller/silero_tts/speak",
            ).value
        )

        self._srv_speak = self.controller.create_client(Speak, srv_speak_name)

        while not self._srv_speak.wait_for_service(
            timeout_sec=self.controller.wait_timeout
        ):
            self.controller.get_logger().info(
                f"Service {srv_speak_name} not available, waiting..."
            )

        self.controller.get_logger().info("silero_tts connected")
        return True

    def say(
        self,
        cancel_event: threading.Event,
        text: str,
        voice: str | int = "eugene",
        path_to_save: str = "",
        put_accent: bool = True,
        put_yo: bool = True,
        play: bool = True,
        block: bool = True,
    ) -> int | None:
        """
        Синтез речи.

        Args:
            cancel_event (threading.Event): event преждевременного завершения работы
            text (str): Текст или SSML (начинающийся с \<speak\>). # type: ignore
                Для ручного ударения - '+' перед гласной: "зам+ок"
            voice (str | int): Имя голоса (str) или индекс (int).
                'aidar'(0), 'baya'(1), 'kseniya'(2), 'eugene'(3), 'xenia'(4)
            path_to_save (str): Путь для сохранения WAV.
                '' - сохранение в RAM (/dev/shm/tmp_silero_tts.wav)
            put_accent (bool): Авторасстановка ударений (игнорируется для SSML)
            put_yo (bool): Авторасстановка буквы ё (игнорируется для SSML)
            play (bool): Воспроизвести через media_driver после синтеза
            block (bool): При вызове функции ждать завершения воспроизведения

        Returns:
            int | None:
                0 — успех,
                -1 — пустой текст,
                -2 — недопустимый индекс голоса,
                -3 — ошибка синтеза,
                -4 — ошибка записи файла,
                -5 — play=true, но media_driver не настроен,
                -6 — ошибка воспроизведения,
                None — отменено через cancel_event.
        """
        if self._srv_speak is not None:
            req = Speak.Request()
            req.text = text
            req.path_to_save = path_to_save
            req.put_accent = put_accent
            req.put_yo = put_yo
            req.play = play

            if isinstance(voice, str):
                req.voice = self.VOICES.get(voice, 4)
            else:
                req.voice = int(voice)

            future = self._srv_speak.call_async(request=req)

            while not future.done() and not cancel_event.is_set():
                self.controller.rate_.sleep()

            while block and not cancel_event.is_set():
                idle = self.controller.media_driver.is_idle_audio(cancel_event)
                if idle or cancel_event.is_set():
                    break
                self.controller.rate_.sleep()

            raw_result = future.result()
            if raw_result is not None:
                result = cast(Speak.Response, raw_result)
                return result.data
            return None
        else:
            self.controller.get_logger().error(
                "Try to call silero_tts.say() but silero_tts._srv_speak is None. Skip call..."
            )
            return None
