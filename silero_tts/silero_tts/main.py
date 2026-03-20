#!/usr/bin/env python3
"""
Silero TTS ROS2 Node — локальная модель, без интернета.

Speak.srv:
  string text          — текст для озвучки
  int16 voice          — индекс голоса (0-4)
  string path_to_save  — путь WAV ("" = /dev/shm)
  bool put_accent      — авторасстановка ударений
  bool put_yo          — авторасстановка ё
  bool play            — воспроизвести через media_driver
  ---
  int16 data           — 0=ok, <0=ошибка
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from robohead_interfaces.srv import Speak, PlayMedia

import torch
import wave
import os
import threading
import numpy as np


class SileroTtsNode(Node):

    VOICES = ['aidar', 'baya', 'kseniya', 'eugene', 'xenia']

    def __init__(self):
        super().__init__('silero_tts')

        self.rate_ = self.create_rate(100)

        # ── Параметры ──
        self.sample_rate = self.declare_parameter('sample_rate', 48000).value
        self.default_dir = self.declare_parameter('default_dir', '/dev/shm').value
        self.device = self.declare_parameter('device', 'cpu').value

        srv_speak_name = self.declare_parameter('srv_speak_name', 'speak').value
        self.srv_play_media_name = self.declare_parameter('srv_play_media', '').value

        # ── Путь к модели ──
        model_path = self.declare_parameter('model_path', 'v5_3_ru.pt').value

        package_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(package_dir, 'model', model_path)

        if not os.path.exists(model_path):
            self.get_logger().fatal(f'Model not found: {model_path}\n Run: ./model/download_model.sh')
            raise FileNotFoundError(f'Silero model not found: {model_path}')

        # ── Загрузка модели ──
        self.get_logger().info(f'Loading model: {model_path}')

        self.model = torch.package.PackageImporter(model_path).load_pickle("tts_models", "model")
        self.model.to(torch.device(self.device))

        self.get_logger().info(f'Model loaded')

        # ── ROS2 интерфейсы ──
        cb_group = ReentrantCallbackGroup()

        self.srv_speak = self.create_service(Speak, srv_speak_name, self.handle_speak, callback_group=cb_group)

        # Клиент media_driver
        self.cli_play_media = None
        if self.srv_play_media_name:
            self.cli_play_media = self.create_client(PlayMedia, self.srv_play_media_name, callback_group=cb_group)
            self.get_logger().info(f'Play media via: {self.srv_play_media_name}')

        self.get_logger().info(f'INITED')


    def handle_speak(self, request, response):
        text = request.text.strip()
        voice_idx = request.voice
        path_to_save = request.path_to_save.strip()
        put_accent = request.put_accent
        put_yo = request.put_yo
        play = request.play

        # Валидация текста
        if not text:
            self.get_logger().warn('Empty text received')
            response.data = -1
            return response

        # Валидация голоса
        if voice_idx < 0 or voice_idx >= len(self.VOICES):
            self.get_logger().warn('Invalid voice id')
            response.data = -2
            return response

        speaker = self.VOICES[voice_idx]

        # Путь: явный или RAM
        if not path_to_save:
            path_to_save = os.path.join(self.default_dir, f'tmp_silero_tts.wav')

        # Синтез
        try:
            wav_path = self._synthesize(text, speaker, path_to_save, put_accent, put_yo)
        except Exception as e:
            self.get_logger().error(f'Synthesis failed: {e}')
            response.data = -3
            return response

        if wav_path is None:
            response.data = -4
            return response

        # Воспроизведение
        if play:
            if self.cli_play_media is None:
                self.get_logger().warn(
                    'Play requested but srv_play_media not configured')
                response.data = -5
                return response

            play_result = self._play_via_media_driver(wav_path)
            if play_result < 0:
                self.get_logger().warn('Playback failed')
                response.data = -6
                return response

        response.data = 0
        return response

    # ════════════════════════════════════════════
    #  Синтез
    # ════════════════════════════════════════════

    def _synthesize(self, text: str, speaker: str, output_path: str,
                    put_accent: bool, put_yo: bool) -> str:

        out_dir = os.path.dirname(output_path)
        if out_dir and not os.path.exists(out_dir):
            try:
                os.makedirs(out_dir, exist_ok=True)
            except OSError as e:
                self.get_logger().error(
                    f'Cannot create directory {out_dir}: {e}')
                return None

        audio = self.model.apply_tts(
            text=text,
            speaker=speaker,
            sample_rate=self.sample_rate,
            put_accent=put_accent,
            put_yo=put_yo
        )

        audio_np = audio.cpu().numpy()
        audio_int16 = np.clip(
            audio_np * 32767, -32768, 32767
        ).astype(np.int16)

        try:
            with wave.open(output_path, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(self.sample_rate)
                wf.writeframes(audio_int16.tobytes())
        except Exception as e:
            self.get_logger().error(f'WAV write failed: {e}')
            return None

        return output_path

    # ════════════════════════════════════════════
    #  Воспроизведение
    # ════════════════════════════════════════════

    def _play_via_media_driver(self, wav_path: str) -> int:
        if not self.cli_play_media.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('media_driver not available')
            return -1

        req = PlayMedia.Request()
        req.path_to_video_file = ''
        req.path_to_audio_file = wav_path
        req.loop = False

        future = self.cli_play_media.call_async(req)

        while not future.done():
            self.rate_.sleep()


        if future.done() and future.result() is not None:
            return future.result().data
        return -1



# ════════════════════════════════════════════
#  main
# ════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = SileroTtsNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()