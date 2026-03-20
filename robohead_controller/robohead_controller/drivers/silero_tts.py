from robohead_interfaces.srv import Speak
import threading

class SileroTTSConnector:
    
    def __init__(self, controller):
        self.controller = controller
        self.srv_speak = None
    
    def connect(self):
        self.VOICES = {
            'aidar': 0,
            'baya': 1,
            'kseniya': 2,
            'eugene': 3,
            'xenia': 4,
        }

        srv_speak_name = self.controller.declare_parameter('silero_tts.srv_speak_name', '/robohead_controller/silero_tts/speak').value

        self.srv_speak = self.controller.create_client(Speak, srv_speak_name)

        while not self.srv_speak.wait_for_service(timeout_sec=self.controller.wait_timeout):
            self.controller.get_logger().info(
                f'Service {srv_speak_name} not available, waiting...')

        self.controller.get_logger().info('silero_tts connected')    

        return True
    


    def say(self, cancel_event: threading.Event,
            text: str,
            voice='eugene',
            path_to_save: str = '',
            put_accent: bool = True,
            put_yo: bool = True,
            play: bool = True,
            block: bool = True) -> int:
        """
        Синтез речи.

        Args:
            cancel_event: Событие отмены действия
            text: Текст или SSML (начинающийся с <speak>).
                  Для ручного ударения — '+' перед гласной: "зам+ок"
            voice: Имя голоса (str) или индекс (int)
                   'aidar'(0), 'baya'(1), 'kseniya'(2), 'eugene'(3), 'xenia'(4)
            path_to_save: Путь для сохранения WAV.
                          '' — сохранение в RAM (/dev/shm/tmp_silero_tts.wav)
            put_accent: Авторасстановка ударений (игнорируется для SSML)
            put_yo: Авторасстановка буквы ё (игнорируется для SSML)
            play: Воспроизвести через media_driver после синтеза
            block: Ждать завершения воспроизведения

        Returns:
            0  — успех
            -1 — пустой текст
            -2 — недопустимый индекс голоса
            -3 — ошибка синтеза
            -4 — ошибка записи файла
            -5 — play=true, но media_driver не настроен
            -6 — ошибка воспроизведения
            None — отменено через cancel_event
        """
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

        future = self.srv_speak.call_async(req)

        while not future.done() and not cancel_event.is_set():
            self.controller.rate_.sleep()

        if future.done() and future.result() is not None:
            return future.result().data
        
        while block and not cancel_event.is_set():
            idle = self.controller.media_driver.is_idle_audio(cancel_event)
            if idle or cancel_event.is_set():
                break
            self.controller.rate_.sleep()

        return None