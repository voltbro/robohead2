from std_msgs.msg import String
from robohead_interfaces.srv import SimpleCommand
import rclpy
import sys

class SpeechRecognizerKwsConnector:
    
    def __init__(self, controller):
        self.controller = controller

        self.srv_set_mode = None
        self.sub_fast_commands = None
        self.sub_wake_phrases = None
    
    def connect(self):
    
        srv_set_mode_name = self.controller.declare_parameter('speech_recognizer_kws.service_name.set_mode', 'dflt').value
        sub_fast_commands_name = self.controller.declare_parameter('speech_recognizer_kws.topic_name.fast_commands', 'dflt').value
        sub_wake_phrases_name = self.controller.declare_parameter('speech_recognizer_kws.topic_name.wake_phrases', 'dflt').value

        self.sub_fast_commands = self.controller.create_subscription(String, sub_fast_commands_name, self.sub_fast_commands_callback, 1)
        self.sub_wake_phrase = self.controller.create_subscription(String, sub_wake_phrases_name, self.sub_wake_phrases_callback, 1)

        self.srv_set_mode = self.controller.create_client(SimpleCommand, srv_set_mode_name)

        while not self.srv_set_mode.wait_for_service(timeout_sec=self.controller.wait_timeout):
            self.controller.get_logger().info(f'Service {srv_set_mode_name} not available, waiting...')

        self.controller.get_logger().info('speech_recognizer_kws connected')
        return True
    
    def sub_fast_commands_callback(self, msg:String):
        fast_command = msg.data
        self.controller.queue_fast_commands.append(fast_command)
        self.controller.get_logger().info(f'fast_command: |{fast_command}| ')

    def sub_wake_phrases_callback(self, msg:String):
        wake_phrase = msg.data
        self.controller.queue_wake_phrases.append(wake_phrase)
        self.controller.get_logger().info(f'wake_phrase|{wake_phrase}| ')

    def set_mode(self, mode:int=0):
        # mode:
        # 0 off recognition
        # 1 on recognition (wake_phrases + fast_commands)
        req = SimpleCommand.Request()
        req.data = mode
        
        future = self.srv_set_mode.call_async(req)
        # rclpy.spin_until_future_complete(self.controller, future, timeout_sec=self.controller.wait_timeout)
        # return future.result() if future.done() else None


