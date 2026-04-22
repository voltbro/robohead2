# std_ears
# действие, выполняющееся при команде "Покажи уши"

# Импорты нужны для автоподстановок кода при работе через VSCode
from __future__ import annotations
from typing import TYPE_CHECKING
import os

if TYPE_CHECKING:
    from robohead_controller.controller import RoboheadController
    import threading


def run(
    controller: RoboheadController, action_name: str, cancel_event: threading.Event
):
    """
    Args:
        controller: Ссылка на контроллер
        action_name: Команда, по которой было вызвано действие
        cancel_event: threading.Event для проверки отмены
    """
    action_dir = os.path.dirname(os.path.abspath(__file__)) # Путь к папке со скриптом, обычно это:
    # /home/pi/robohead_ws/build/robohead_controller/robohead_controller/actions/std_ears

    logger = controller.get_logger()        # logger - объект логирования, через него можно печатать в консоль
    logger.info(f"[{action_name}] start")   # выводим в терминал "[std_ears] start"

    # Выводим картинку ears.png без зацикливания воспроизведения (это же картинка) и блокирования вызова
    controller.media_driver.play_display(
        cancel_event=cancel_event,
        video_path=os.path.join(action_dir, "ears.png"),
        loop=False,
        block=False,
    )

    # Проигрываем звук ears.mp3 без зацикливания воспроизведения и блокирования вызова
    controller.media_driver.play_audio(
        cancel_event=cancel_event,
        audio_path=os.path.join(action_dir, "ears.mp3"),
        loop=False,
        block=False,
    ) 

    for k in range(5): # Цикл 5 раз
        # Поворачиваем голову
        controller.neck_driver.set_angle(
            cancel_event=cancel_event,
            horizontal=15 * (-1) ** k,  # значения будут: 15, -15, 15, -15, 15
            vertical=15,    # Вертикальный подьем головы 15 градусов
            duration=0.5,   # Длительность достижения заданной позиции 0.5 секунд
            block=False,    # Вызов без блокирования
        )
        # Поворачиваем уши
        controller.ears_driver.set_angle(
            cancel_event=cancel_event,
            left=90 * (-1) ** k,    # Значения поворота левого уха: 90, -90, 90, -90, 90
            right=-90 * (-1) ** k,  # Значения поворота правого уха: -90, 90, -90, 90, -90
            duration=0.5,   # Длительность достижения заданной позиции 0.5 секунд
            block=True,     # Блокирующий вызов: программа здесь "зависнет" на 0.5 секунд (duration)
        )

    logger.info(f"[{action_name}] finish")  # выводим в терминал "[std_ears] finish"
