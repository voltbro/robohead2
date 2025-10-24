#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robohead_interfaces.msg import AudioData
from robohead_interfaces.srv import SimpleCommand
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer
import json
import os
from ament_index_python.packages import get_package_share_directory

# ros2 service call /speech/set_mode robohead_interfaces/srv/SimpleCommand "data: 1"
# colcon build --packages-select speech_recognizer --symlink-install
class SpeechRecognizer(Node):
    def __init__(self):
        super().__init__('speech_recognizer')

        # Параметры
        self.declare_parameter('default_mode', 0)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('model_path', 'vosk-model-small-ru-0.22')
        self.declare_parameter('wake_phrases', ['слушай робот'])
        self.declare_parameter('commands', ['включи свет', 'остановись', 'вперёд'])
        self.declare_parameter('ros/service_name/set_mode', '~/set_mode')
        self.declare_parameter('ros/topic_name/wake_phrases', '~/wake_phrases')
        self.declare_parameter('ros/topic_name/commands', '~/commands')
        self.declare_parameter('ros/topic_name/audio_input', '/respeaker_driver/audio/main')

        model_path = self.get_parameter('model_path').value

        self.wake_phrases = self.get_parameter('wake_phrases').value
        self.commands = self.get_parameter('commands').value
        srv_name_set_mode = self.get_parameter("ros/service_name/set_mode").value
        topic_name_wake_phrases = self.get_parameter("ros/topic_name/wake_phrases").value
        topic_name_commands = self.get_parameter("ros/topic_name/commands").value
        topic_name_audio_input = self.get_parameter("ros/topic_name/audio_input").value
        sample_rate = self.get_parameter('sample_rate').value
        default_mode = self.get_parameter('default_mode').value





        if not os.path.exists(model_path):
            self.get_logger().error(f"Model not found at {model_path}")
            raise FileNotFoundError(f"Vosk model not found: {model_path}")

        self.model = Model(model_path)

        # Инициализация распознавателей
        self.wake_rec = KaldiRecognizer(self.model, sample_rate)
        wake_grammar_json = json.dumps(self.wake_phrases, ensure_ascii=False)
        self.wake_rec.SetGrammar(wake_grammar_json)

        self.grammar_rec = KaldiRecognizer(self.model, sample_rate)
        cmd_grammar_json = json.dumps(self.commands, ensure_ascii=False)
        self.grammar_rec.SetGrammar(cmd_grammar_json)

        self.free_rec = KaldiRecognizer(self.model, sample_rate)  # без грамматики

        # Начинаем в режиме KWS
        self.current_mode = default_mode  # 0 = off, 1 = KWS, 2 = grammar, 3 = free

        # ROS
        self.audio_sub = self.create_subscription(
            AudioData, topic_name_audio_input, self.audio_callback, 10)
        self.wake_pub = self.create_publisher(String, topic_name_wake_phrases, 10)
        self.cmd_pub = self.create_publisher(String, topic_name_commands, 10)

        # ЕДИНЫЙ сервис управления
        self.srv = self.create_service(SimpleCommand, srv_name_set_mode, self.set_mode_callback)

        self.get_logger().info(f"Speech recognizer started in (mode={self.current_mode})")

    def set_mode_callback(self, request, response):
        mode = request.data
        if mode in (0, 1, 2, 3):
            self.current_mode = mode
            response.data = mode
            mode_names = {0:"Off", 1: "KWS", 2: "Grammar", 3: "Free"}
            self.get_logger().info(f"Mode switched to {mode_names[mode]}")
        else:
            response.data = self.current_mode
            self.get_logger().warn(f"Ignored invalid mode: {mode}")
        return response

    def audio_callback(self, msg):
        if self.current_mode == 0:
            return
        if not msg.data:
            return

        data = bytes(msg.data)

        if self.current_mode == 1:  # KWS
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

        elif self.current_mode == 2:  # Grammar
            if self.grammar_rec.AcceptWaveform(data):
                res = json.loads(self.grammar_rec.Result())
                text = res.get('text', '').strip()
                if text:
                    cmd_msg = String()
                    cmd_msg.data = text
                    self.cmd_pub.publish(cmd_msg)
                    self.get_logger().info(f"Command: '{text}'")

        elif self.current_mode == 3:  # Free
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