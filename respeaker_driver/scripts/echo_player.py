#!/usr/bin/env python3
"""
Потоково воспроизводит аудио из ROS2 топика через системный аудиовыход.
Работает с Bluetooth-наушниками через PipeWire/PulseAudio.

Использование:
  ros2 run respeaker_driver audio_streamer.py
  ros2 run respeaker_driver audio_streamer.py --ros-args -p topic:=/respeaker_driver/audio/main -p sample_rate:=16000
"""

import rclpy
from rclpy.node import Node
from robohead_interfaces.msg import AudioData
import threading
import queue
import numpy as np
import sounddevice as sd


class AudioStreamer(Node):
    def __init__(self):
        super().__init__("audio_streamer")

        self.topic_ = self.declare_parameter("topic", "/respeaker_driver/audio/main").value
        self.rate_ = self.declare_parameter("sample_rate", 16000).value

        # Очередь для обмена данными между ROS-колбэком и аудио-потоком
        self.audio_queue = queue.Queue(maxsize=30)
        self.is_running = True
        self.stream = None

        self.sub_ = self.create_subscription(AudioData, self.topic_, self.on_audio, 10)

        self.get_logger().info(
            f"Streaming audio from '{self.topic_}' to default output device at {self.rate_} Hz"
        )

        # Запуск отдельного потока для воспроизведения
        self.play_thread = threading.Thread(target=self._playback_loop, daemon=True)
        self.play_thread.start()

    def on_audio(self, msg):
        # Безопасное преобразование входящих данных в int16 numpy array
        if isinstance(msg.data, bytes):
            chunk = np.frombuffer(msg.data, dtype=np.int16)
        else:
            chunk = np.array(msg.data, dtype=np.int16)

        try:
            self.audio_queue.put_nowait(chunk)
        except queue.Full:
            self.get_logger().warn("Audio buffer full, dropping oldest chunk")
            # Удаляем самый старый чанк, чтобы не блокировать ROS-поток
            try:
                self.audio_queue.get_nowait()
                self.audio_queue.put_nowait(chunk)
            except queue.Empty:
                pass

    def _playback_loop(self):
        try:
            self.stream = sd.OutputStream(
                samplerate=self.rate_,
                channels=1,
                dtype='int16',
                latency='low',      # Минимальная задержка
                blocksize=1024      # Размер буфера PortAudio
            )
            self.stream.start()
            self.get_logger().info("Audio stream opened successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to open audio stream: {e}")
            self.is_running = False
            return

        while self.is_running:
            try:
                chunk = self.audio_queue.get(timeout=0.5)
                self.stream.write(chunk)
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Playback error: {e}")
                break

        self.stream.close()
        self.get_logger().info("Audio stream closed")

    def destroy_node(self):
        self.is_running = False
        if self.stream:
            self.stream.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupt received, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()