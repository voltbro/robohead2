from __future__ import annotations
from typing import TYPE_CHECKING, cast, Optional
import threading

if TYPE_CHECKING:
    from ..controller import RoboheadController
    from rclpy.client import Client
    from rclpy.publisher import Publisher

from robohead_interfaces.srv import PlayMedia, SimpleCommand
from sensor_msgs.msg import Image


class MediaDriverConnector:

    def __init__(self, controller: RoboheadController):
        self.controller: RoboheadController = controller

        self._srv_play_media: Optional[Client] = None
        self._srv_get_volume: Optional[Client] = None
        self._srv_set_volume: Optional[Client] = None
        self._srv_is_idle_audio: Optional[Client] = None
        self._srv_is_idle_display: Optional[Client] = None
        self._pub_stream: Optional[Publisher] = None
        self._stop_command: str = "__STOP__"

    @property
    def stop_command(self) -> str:
        """
        Возвращает систменое слово для принудительной остановки воспроизведения.
        Неизменяемое значение. Для изменения редактируйте конфиг-файлы.
        """
        return self._stop_command

    def connect(self) -> bool:
        """
        Подключение к сервисам и топикам.

        Returns:
            out (bool): True if successfull, False - else
        """

        srv_set_volume_name: str = str(
            self.controller.declare_parameter(
                "media_driver.srv_set_volume_name", "set_volume"
            ).value
        )
        srv_get_volume_name: str = str(
            self.controller.declare_parameter(
                "media_driver.srv_get_volume_name", "get_volume"
            ).value
        )
        srv_play_media_name: str = str(
            self.controller.declare_parameter(
                "media_driver.srv_play_media_name", "play_media"
            ).value
        )
        srv_is_idle_audio_name: str = str(
            self.controller.declare_parameter(
                "media_driver.srv_is_idle_audio_name", "is_idle/audio"
            ).value
        )
        srv_is_idle_display_name: str = str(
            self.controller.declare_parameter(
                "media_driver.srv_is_idle_display_name", "is_idle/display"
            ).value
        )
        topic_stream_name: str = str(
            self.controller.declare_parameter(
                "media_driver.topic_stream_name", "stream"
            ).value
        )
        self._stop_command = str(
            self.controller.declare_parameter(
                "media_driver.stop_command", "__STOP__"
            ).value
        )

        self._srv_play_media = self.controller.create_client(
            PlayMedia, srv_play_media_name
        )
        self._srv_get_volume = self.controller.create_client(
            SimpleCommand, srv_get_volume_name
        )
        self._srv_set_volume = self.controller.create_client(
            SimpleCommand, srv_set_volume_name
        )
        self._srv_is_idle_audio = self.controller.create_client(
            SimpleCommand, srv_is_idle_audio_name
        )
        self._srv_is_idle_display = self.controller.create_client(
            SimpleCommand, srv_is_idle_display_name
        )

        self._pub_stream = self.controller.create_publisher(
            msg_type=Image, topic=topic_stream_name, qos_profile=1
        )

        while not self._srv_play_media.wait_for_service(
            timeout_sec=self.controller.wait_timeout
        ):
            self.controller.get_logger().info(
                f"Service {srv_play_media_name} not available, waiting..."
            )
        while not self._srv_get_volume.wait_for_service(
            timeout_sec=self.controller.wait_timeout
        ):
            self.controller.get_logger().info(
                f"Service {srv_get_volume_name} not available, waiting..."
            )
        while not self._srv_set_volume.wait_for_service(
            timeout_sec=self.controller.wait_timeout
        ):
            self.controller.get_logger().info(
                f"Service {srv_set_volume_name} not available, waiting..."
            )
        while not self._srv_is_idle_audio.wait_for_service(
            timeout_sec=self.controller.wait_timeout
        ):
            self.controller.get_logger().info(
                f"Service {srv_is_idle_audio_name} not available, waiting..."
            )
        while not self._srv_is_idle_display.wait_for_service(
            timeout_sec=self.controller.wait_timeout
        ):
            self.controller.get_logger().info(
                f"Service {srv_is_idle_display_name} not available, waiting..."
            )

        self.controller.get_logger().info("media_driver connected")
        return True

    def is_idle_audio(self, cancel_event: threading.Event) -> int | None:
        """
        Проверяет, находится ли аудио-канал в состоянии покоя (idle).

        Args:
            cancel_event (threading.Event): event преждевременного завершения работы

        Returns:
            int | None: результат проверки (1 - ожидание команды, 0 - идёт воспроизведение), None - если выполнение было прервано через cancel_event.
        """
        if self._srv_is_idle_audio is not None:
            req = SimpleCommand.Request()

            future = self._srv_is_idle_audio.call_async(request=req)

            while not future.done() and not cancel_event.is_set():
                self.controller.rate_.sleep()

            raw_result = future.result()
            if raw_result is not None:
                result = cast(SimpleCommand.Response, raw_result)
                return result.data
            return None
        else:
            self.controller.get_logger().error(
                "Try to call media_driver.is_idle_audio() but media_driver._srv_is_idle_audio is None. Skip call..."
            )
            return None

    def is_idle_display(self, cancel_event: threading.Event) -> int | None:
        """
        Проверяет, находится ли дисплей в состоянии покоя (idle).

        Args:
            cancel_event (threading.Event): event преждевременного завершения работы

        Returns:
            int | None: результат проверки (1 - ожидание команды, 0 - идёт воспроизведение), None - если выполнение было прервано через cancel_event.
        """
        if self._srv_is_idle_display is not None:
            req = SimpleCommand.Request()

            future = self._srv_is_idle_display.call_async(request=req)

            while not future.done() and not cancel_event.is_set():
                self.controller.rate_.sleep()

            raw_result = future.result()
            if raw_result is not None:
                result = cast(SimpleCommand.Response, raw_result)
                return result.data
            return None
        else:
            self.controller.get_logger().error(
                "Try to call media_driver.is_idle_display() but media_driver._srv_is_idle_display is None. Skip call..."
            )
            return None

    def play_media(
        self,
        cancel_event: threading.Event,
        video_path: str = "",
        audio_path: str = "",
        loop: bool = False,
        block: bool = True,
    ) -> int | None:
        """
        Воспроизводит медиа (видео и/или аудио).

        Args:
            cancel_event (threading.Event): event преждевременного завершения работы
            video_path (str): путь к видеофайлу (пустая строка - без видео)
            audio_path (str): путь к аудиофайлу (пустая строка - без аудио)
            loop (bool): если True - зацикленное воспроизведение
            block (bool): если True - ожидает завершения воспроизведения (вызов завершится, когда оба канала перейдут в idle или вызвано прерывание по cancel_event)

        Returns:
            int | None: результат выполнения команды, None - если выполнение было прервано через cancel_event.
        """
        if self._srv_play_media is not None:
            req = PlayMedia.Request()
            req.path_to_video_file = video_path
            req.path_to_audio_file = audio_path
            req.loop = loop

            future = self._srv_play_media.call_async(request=req)

            while not future.done() and not cancel_event.is_set():
                self.controller.rate_.sleep()

            while (
                block
                and not (
                    self.is_idle_audio(cancel_event)
                    and self.is_idle_display(cancel_event)
                )
                and not cancel_event.is_set()
            ):
                self.controller.rate_.sleep()

            raw_result = future.result()
            if raw_result is not None:
                result = cast(PlayMedia.Response, raw_result)
                return result.data
            return None
        else:
            self.controller.get_logger().error(
                "Try to call media_driver.play_media() but media_driver._srv_play_media is None. Skip call..."
            )
            return None

    def play_audio(
        self,
        cancel_event: threading.Event,
        audio_path: str = "",
        loop: bool = False,
        block: bool = True,
    ) -> int | None:
        """
        Воспроизводит аудио.

        Args:
            cancel_event (threading.Event): event преждевременного завершения работы
            audio_path (str): путь к аудиофайлу
            loop (bool): если True - зацикленное воспроизведение
            block (bool): если True - ожидает завершения воспроизведения аудио (вызов завершится, когда аудио-канал перейдёт в idle или вызвано прерывание по cancel_event)

        Returns:
            int | None: результат выполнения команды, None - если выполнение было прервано через cancel_event.
        """
        if self._srv_play_media is not None:
            req = PlayMedia.Request()
            req.path_to_video_file = ""
            req.path_to_audio_file = audio_path
            req.loop = loop

            future = self._srv_play_media.call_async(request=req)

            while not future.done() and not cancel_event.is_set():
                self.controller.rate_.sleep()

            while (
                block
                and not self.is_idle_audio(cancel_event)
                and not cancel_event.is_set()
            ):
                self.controller.rate_.sleep()

            raw_result = future.result()
            if raw_result is not None:
                result = cast(PlayMedia.Response, raw_result)
                return result.data
            return None
        else:
            self.controller.get_logger().error(
                "Try to call media_driver.play_audio() but media_driver._srv_play_media is None. Skip call..."
            )
            return None

    def play_display(
        self,
        cancel_event: threading.Event,
        video_path: str = "",
        loop: bool = False,
        block: bool = True,
    ) -> int | None:
        """
        Воспроизводит видео на дисплее.

        Args:
            cancel_event (threading.Event): event преждевременного завершения работы
            video_path (str): путь к видеофайлу
            loop (bool): если True - зацикленное воспроизведение
            block (bool): если True - ожидает завершения воспроизведения на дисплее (вызов завершится, когда дисплей перейдёт в idle или вызвано прерывание по cancel_event)

        Returns:
            int | None: результат выполнения команды, None - если выполнение было прервано через cancel_event.
        """
        if self._srv_play_media is not None:
            req = PlayMedia.Request()
            req.path_to_video_file = video_path
            req.path_to_audio_file = ""
            req.loop = loop

            future = self._srv_play_media.call_async(request=req)

            while not future.done() and not cancel_event.is_set():
                self.controller.rate_.sleep()

            while (
                block
                and not self.is_idle_display(cancel_event)
                and not cancel_event.is_set()
            ):
                self.controller.rate_.sleep()

            raw_result = future.result()
            if raw_result is not None:
                result = cast(PlayMedia.Response, raw_result)
                return result.data
            return None
        else:
            self.controller.get_logger().error(
                "Try to call media_driver.play_display() but media_driver._srv_play_media is None. Skip call..."
            )
            return None

    def set_volume(self, cancel_event: threading.Event, volume: int = 50) -> int | None:
        """
        Устанавливает громкость воспроизведения.

        Args:
            cancel_event (threading.Event): event преждевременного завершения работы
            volume (int): значение громкости от 0 до 100

        Returns:
            int | None: 0 если успешно, -1 при ошибке. None - если выполнение было прервано через cancel_event.
        """
        if self._srv_set_volume is not None:
            req = SimpleCommand.Request()
            req.data = int(volume)

            future = self._srv_set_volume.call_async(request=req)

            while not future.done() and not cancel_event.is_set():
                self.controller.rate_.sleep()

            raw_result = future.result()
            if raw_result is not None:
                result = cast(SimpleCommand.Response, raw_result)
                return result.data
            return None
        else:
            self.controller.get_logger().error(
                "Try to call media_driver.set_volume() but media_driver._srv_set_volume is None. Skip call..."
            )
            return None

    def get_volume(self, cancel_event: threading.Event) -> int | None:
        """
        Получает текущую громкость воспроизведения.

        Args:
            cancel_event (threading.Event): event преждевременного завершения работы

        Returns:
            int | None: текущее значение громкости, None - если выполнение было прервано через cancel_event.
        """
        if self._srv_get_volume is not None:
            req = SimpleCommand.Request()

            future = self._srv_get_volume.call_async(request=req)

            while not future.done() and not cancel_event.is_set():
                self.controller.rate_.sleep()

            raw_result = future.result()
            if raw_result is not None:
                result = cast(SimpleCommand.Response, raw_result)
                return result.data
            return None
        else:
            self.controller.get_logger().error(
                "Try to call media_driver.get_volume() but media_driver._srv_get_volume is None. Skip call..."
            )
            return None

    def stream_publish(self, image_msg: Image) -> None:
        """
        Публикация кадра в видеопоток.

        Args:
            image_msg (Image): сообщение с изображением для публикации
        """
        if self._pub_stream is not None:
            self._pub_stream.publish(image_msg)
        else:
            self.controller.get_logger().error(
                "Try to call media_driver.stream_publish() but media_driver._pub_stream is None. Skip call..."
            )
