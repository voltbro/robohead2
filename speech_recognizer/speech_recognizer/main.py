#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robohead_interfaces.msg import AudioData
from robohead_interfaces.srv import SimpleCommand
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer
import json
import os

# ros2 service call /speech/set_mode robohead_interfaces/srv/SimpleCommand "data: 1"
# colcon build --packages-select speech_recognizer --symlink-install
class SpeechRecognizer(Node):
    def __init__(self):
        super().__init__('speech_recognizer')

        # Параметры
        self.declare_parameter('model_path', '/home/pi/vosk-model-small-ru-0.22')
        self.declare_parameter('wake_phrases', ['слушай робот'])
        self.declare_parameter('commands', ['включи свет', 'остановись', 'вперёд'])

        model_path = self.get_parameter('model_path').value
        self.wake_phrases = self.get_parameter('wake_phrases').value
        self.commands = self.get_parameter('commands').value

        if not os.path.exists(model_path):
            self.get_logger().error(f"Model not found at {model_path}")
            raise FileNotFoundError(f"Vosk model not found: {model_path}")

        self.model = Model(model_path)

        # Инициализация распознавателей
        self.wake_rec = KaldiRecognizer(self.model, 16000)
        wake_grammar_json = json.dumps(self.wake_phrases, ensure_ascii=False)
        self.wake_rec.SetGrammar(wake_grammar_json)

        self.grammar_rec = KaldiRecognizer(self.model, 16000)
        cmd_grammar_json = json.dumps(self.commands, ensure_ascii=False)
        self.grammar_rec.SetGrammar(cmd_grammar_json)

        self.free_rec = KaldiRecognizer(self.model, 16000)  # без грамматики

        # Начинаем в режиме KWS
        self.current_mode = 0  # 0 = KWS, 1 = grammar, 2 = free

        # ROS
        self.audio_sub = self.create_subscription(
            AudioData, '/respeaker_driver/audio/main', self.audio_callback, 10)
        self.wake_pub = self.create_publisher(String, '/speech/wake_phrase', 10)
        self.cmd_pub = self.create_publisher(String, '/speech/command', 10)

        # ЕДИНЫЙ сервис управления
        self.srv = self.create_service(SimpleCommand, '/speech/set_mode', self.set_mode_callback)

        self.get_logger().info("Speech recognizer started in KWS mode (mode=0)")

    def set_mode_callback(self, request, response):
        mode = request.data
        if mode in (0, 1, 2):
            self.current_mode = mode
            response.data = mode
            mode_names = {0: "KWS", 1: "Grammar", 2: "Free"}
            self.get_logger().info(f"Mode switched to {mode_names[mode]}")
        else:
            response.data = self.current_mode
            self.get_logger().warn(f"Ignored invalid mode: {mode}")
        return response

    def audio_callback(self, msg):
        if not msg.data:
            return

        data = bytes(msg.data)

        if self.current_mode == 0:  # KWS
            if self.wake_rec.AcceptWaveform(data):
                res = json.loads(self.wake_rec.Result())
                text = res.get('text', '').strip().lower()
                if text:
                    for phrase in self.wake_phrases:
                        if phrase in text:
                            wake_msg = String()
                            wake_msg.data = phrase
                            self.wake_pub.publish(wake_msg)
                            self.get_logger().info(f"Wake phrase: '{phrase}'")
                            break

        elif self.current_mode == 1:  # Grammar
            if self.grammar_rec.AcceptWaveform(data):
                res = json.loads(self.grammar_rec.Result())
                text = res.get('text', '').strip()
                if text:
                    cmd_msg = String()
                    cmd_msg.data = text
                    self.cmd_pub.publish(cmd_msg)
                    self.get_logger().info(f"Command: '{text}'")

        elif self.current_mode == 2:  # Free
            if self.free_rec.AcceptWaveform(data):
                res = json.loads(self.free_rec.Result())
                text = res.get('text', '').strip()
                if text:
                    cmd_msg = String()
                    cmd_msg.data = text
                    self.cmd_pub.publish(cmd_msg)
                    self.get_logger().info(f"Transcript: '{text}'")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SpeechRecognizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()