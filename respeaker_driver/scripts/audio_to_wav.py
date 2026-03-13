#!/usr/bin/env python3
"""
Записывает аудио из ROS2 топика в WAV файл.

Использование:
  ros2 run respeaker_driver audio_to_wav.py
  ros2 run respeaker_driver audio_to_wav.py --ros-args -p topic:=/respeaker_driver/audio/main -p filename:=ch0.wav -p duration:=10.0
  
Прослушивание:
  aplay output.wav
  ffplay -autoexit output.wav
"""

import rclpy
from rclpy.node import Node
from robohead_interfaces.msg import AudioData
import struct
import wave


class AudioToWav(Node):
    def __init__(self):
        super().__init__("audio_to_wav")

        self.topic_    = self.declare_parameter("topic", "audio/main").value
        self.filename_ = self.declare_parameter("filename", "output.wav").value
        self.duration_ = self.declare_parameter("duration", 5.0).value
        self.rate_     = self.declare_parameter("sample_rate", 16000).value

        self.samples_ = []
        self.max_samples_ = int(self.duration_ * self.rate_)
        self.saved_ = False

        self.sub_ = self.create_subscription(
            AudioData, self.topic_, self.on_audio, 10)

        self.get_logger().info(
            f"Recording {self.duration_}s from '{self.topic_}' → '{self.filename_}' "
            f"({self.rate_} Hz)")

    def on_audio(self, msg):
        if self.saved_:
            return

        self.samples_.extend(msg.data)

        if len(self.samples_) >= self.max_samples_:
            self.samples_ = self.samples_[:self.max_samples_]
            self.save_wav()
            self.saved_ = True
            self.get_logger().info(
                f"Done! Saved {len(self.samples_)} samples to '{self.filename_}'")
            raise SystemExit

    def save_wav(self):
        with wave.open(self.filename_, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)  # 16 bit
            w.setframerate(self.rate_)
            w.writeframes(
                struct.pack(f"<{len(self.samples_)}h", *self.samples_))

        self.get_logger().info(
            f"WAV saved: {len(self.samples_)} samples, "
            f"{len(self.samples_) / self.rate_:.2f} sec")


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(AudioToWav())
    except SystemExit:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()