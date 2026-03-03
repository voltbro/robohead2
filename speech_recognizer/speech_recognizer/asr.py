#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robohead_interfaces.msg import AudioData
from robohead_interfaces.srv import SimpleCommand
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer, SetLogLevel, EndpointerMode
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
        model_path = self.declare_parameter('model_path', 'vosk-model-small-ru-0.22').value

        package_dir = os.path.dirname(os.path.abspath(__file__))  # .../speech_recognizer/
        model_path = os.path.join(package_dir, 'model', model_path)

        self.commands = self.declare_parameter('commands', ['покажи левое ухо', 'поздоровайся', 'сделай фото']).value
        srv_name_set_mode = self.declare_parameter('ros.service_name.set_mode', 'set_mode').value
        topic_name_commands = self.declare_parameter('ros.topic_name.commands', 'commands').value
        topic_name_audio_input = self.declare_parameter('ros.topic_name.audio_input', '/respeaker_driver/audio/main').value
     
        self.timeout = self.declare_parameter('timeout', 10.0).value  # например, 5 секунд
        # self.get_logger().error(f"SPEEEECH RECOGNIZER: {self.timeout}")

        self.last_result_time = None
        self.timeout_text = self.declare_parameter('timeout_text', "__TIMEOUT__").value  # например, 5 секунд
      
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model not found at {model_path}")
            raise FileNotFoundError(f"Vosk model not found: {model_path}")

        SetLogLevel(-1) # изменить для отладки vosk
        self.model = Model(model_path)
        # SetLogLevel(4) # изменить для отладки vosk
        # Инициализация распознавателей

        self.grammar_rec = KaldiRecognizer(self.model, sample_rate)
        cmd_grammar_json = json.dumps(self.commands, ensure_ascii=False)
        self.grammar_rec.SetGrammar(cmd_grammar_json)
        t_start_max = 5.0 # Начали распознавание и ждём это количество сек, чтоб принять решение что тишина
        t_end = 0.8 # Кол-во секунд после последнего слова, после которого считаем что фраза окончена
        t_max = 10.0 # Максимальная длительность одной фразы
        self.grammar_rec.SetEndpointerDelays(t_start_max, t_end, t_max)
        # self.grammar_rec.SetEndpointerDelays(t_start_max=0.5, t_end=0.3, t_max=10.0)

        self.free_rec = KaldiRecognizer(self.model, sample_rate)  # без грамматики

        # Начинаем в режиме KWS
        self.current_mode = default_mode  # 0 = off, 1 = grammar (vosk), 2 = free (vosk)

        # ROS
        self.audio_sub = self.create_subscription(
            AudioData, topic_name_audio_input, self.audio_callback, 10)
        self.cmd_pub = self.create_publisher(String, topic_name_commands, 10)

        # ЕДИНЫЙ сервис управления
        self.srv = self.create_service(SimpleCommand, srv_name_set_mode, self.set_mode_callback)

        self.get_logger().info(f"INITED")

    def set_mode_callback(self, request, response):
        mode = request.data
        if mode in (0, 1, 2):
            self.last_result_time = self.get_clock().now()

            self.current_mode = mode
            response.data = mode
            mode_names = {0:"Off", 1: "Grammar (Vosk)", 2: "Free (Vosk)"}
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

        if self.current_mode == 1:  # Grammar
            is_accept = self.grammar_rec.AcceptWaveform(data)
            is_timeout = (self.get_clock().now() - self.last_result_time ).nanoseconds / 1e9 > self.timeout
            if is_accept:
                self.get_logger().warn(f"SPEEEECH RECOGNI ASR: {is_accept}, {is_timeout}")
                
                partial_result = json.loads(self.grammar_rec.PartialResult()).get('partial', '').strip().lower()
                final_result = json.loads(self.grammar_rec.Result()).get('text', '').strip().lower()

                self.get_logger().warn(f"SPEEEECH RECOGNI ASR partial: {partial_result}")
                self.get_logger().warn(f"SPEEEECH RECOGNI ASR final: {final_result}")


                final_phrase = ''
                partial_phrase = ''

                for phrase in self.commands:
                    if phrase in final_result:
                        final_phrase = phrase
                        break
                    elif phrase in partial_result:
                        partial_phrase = phrase
                
                cmd_msg = String()
                if final_phrase:
                    cmd_msg.data = final_phrase
                elif partial_phrase:
                    cmd_msg.data = partial_phrase
                else:
                    cmd_msg.data = self.timeout_text

                self.cmd_pub.publish(cmd_msg)
                self.get_logger().info(f"Command: '{cmd_msg.data}'")

                self.current_mode = 0
                self.get_logger().info(f"Mode switched to Off")

                self.last_result_time = self.get_clock().now()

        elif self.current_mode == 2:  # Free #TODO
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