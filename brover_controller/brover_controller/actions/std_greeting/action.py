# std_greeting
# действие, выполняющееся при команде "Поздоровайся"

from __future__ import annotations
from typing import TYPE_CHECKING
import os

if TYPE_CHECKING:
    from brover_controller.controller import BRoverController
    import threading


def run(
    controller: BRoverController, action_name: str, cancel_event: threading.Event
):
    """
    Args:
        controller: Ссылка на контроллер
        action_name: Команда, по которой было вызвано действие
        cancel_event: threading.Event для проверки отмены
    """
    action_dir = os.path.dirname(os.path.abspath(__file__))

    logger = controller.get_logger()
    logger.info(f"[{action_name}] brover start")

    # Выводим анимацию greeting.mp4
    controller.media_driver.play_display(
        cancel_event=cancel_event,
        video_path=os.path.join(action_dir, "greeting.mp4"),
        loop=True,
        block=False,
    )

    # Проигрываем звук greeting.mp3
    controller.media_driver.play_audio(
        cancel_event=cancel_event,
        audio_path=os.path.join(action_dir, "greeting.mp3"),
        loop=False,
        block=False,
    )

    # Поворачиваем уши
    controller.ears_driver.set_angle(
        cancel_event=cancel_event,
        left=-90,
        right=90,
        duration=0.5,
        block=False,
    )

    for k in range(5):
        controller.brover_driver.velocity_publish(linear_x=0, angular_z=0.5 * (-1)**k)
        # Трясём головой
        controller.neck_driver.set_angle(
            cancel_event=cancel_event,
            horizontal=0,
            vertical=15 + 10 * (-1) ** k,
            duration=1.0,
            block=True,
        )
    controller.brover_driver.velocity_publish(linear_x=0.0, angular_z=0.0)

    logger.info(f"[{action_name}] finish")
