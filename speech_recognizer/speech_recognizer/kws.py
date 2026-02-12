#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robohead_interfaces.msg import AudioData
from robohead_interfaces.srv import SimpleCommand
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer, SetLogLevel
import json
import os
from ament_index_python.packages import get_package_share_directory

# ros2 service call /speech/set_mode robohead_interfaces/srv/SimpleCommand "data: 1"
# colcon build --packages-select speech_recognizer --symlink-install
class SpeechRecognizer(Node):
    def __init__(self):
        super().__init__('speech_recognizer')
        
        # Параметры
        default_mode = self.declare_parameter('default_mode', 0).value
        sample_rate = self.declare_parameter('sample_rate', 16000).value
        model_path = self.declare_parameter('model_path', 'vosk-model-small-ru-0.21').value

        package_dir = os.path.dirname(os.path.abspath(__file__))  # .../speech_recognizer/
        model_path = os.path.join(package_dir, 'model', model_path)

        self.wake_phrases = self.declare_parameter('wake_phrases', ['слушай робот']).value
        self.fast_commands = self.declare_parameter('fast_commands', ['тише', 'громче']).value
        srv_name_set_mode = self.declare_parameter('ros.service_name.set_mode', 'set_mode').value
        topic_name_wake_phrases = self.declare_parameter('ros.topic_name.wake_phrases', 'wake_phrases').value
        topic_name_fast_commands = self.declare_parameter('ros.topic_name.fast_commands', 'fast_commands').value
        topic_name_audio_input = self.declare_parameter('ros.topic_name.audio_input', '/respeaker_driver/audio/main').value
      
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model not found at {model_path}")
            raise FileNotFoundError(f"Vosk model not found: {model_path}")

        SetLogLevel(-1) # изменить для отладки vosk
        self.model = Model(model_path)
        # SetLogLevel(4) # изменить для отладки vosk
        # Инициализация распознавателей
        
        self.recognizer = KaldiRecognizer(self.model, sample_rate)
        wake_grammar_json = json.dumps(self.wake_phrases+self.fast_commands, ensure_ascii=False)
        self.recognizer.SetGrammar(wake_grammar_json)

        # Начинаем в режиме KWS
        self.current_mode = default_mode  # 0 = off, 1 = on

        # ROS
        self.audio_sub = self.create_subscription(
            AudioData, topic_name_audio_input, self.audio_callback, 10)

        self.wake_phrases_pub = self.create_publisher(String, topic_name_wake_phrases, 10)
        self.fast_commands_pub = self.create_publisher(String, topic_name_fast_commands, 10)

        # сервис управления
        self.srv = self.create_service(SimpleCommand, srv_name_set_mode, self.set_mode_callback)

        self.get_logger().info(f"INITED")

    def set_mode_callback(self, request, response):
        mode = request.data
        if mode in (0, 1):
            self.last_grammar_result_time = self.get_clock().now()

            self.current_mode = mode
            response.data = mode
            mode_names = {0:"Off", 1: "On"}
            self.get_logger().info(f"Mode switched to {mode_names[mode]}")
        else:
            response.data = self.current_mode
            self.get_logger().warn(f"Ignored invalid mode: {mode}. Mode must be 0 (Off) or 1 (On)")
        return response

    def audio_callback(self, msg):
        if self.current_mode == 0:
            return
        if not msg.data:
            return

        data = bytes(msg.data)

        if self.recognizer.AcceptWaveform(data):
            res = json.loads(self.recognizer.Result())
            text = res.get('text', '').strip().lower()
            if text:
                for phrase in self.wake_phrases:
                    if phrase in text:
                        wake_msg = String()
                        wake_msg.data = phrase
                        self.wake_phrases_pub.publish(wake_msg)
                        self.get_logger().info(f"Wake phrase: '{phrase}'")
                        break
                for phrase in self.fast_commands:
                    if phrase in text:
                        wake_msg = String()
                        wake_msg.data = phrase
                        self.fast_commands_pub.publish(wake_msg)
                        self.get_logger().info(f"Fast command: '{phrase}'")
                        break

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