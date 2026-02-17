from std_msgs.msg import String
from robohead_interfaces.srv import SimpleCommand
import rclpy
import sys

class SpeechRecognizerAsrConnector:
    
    def __init__(self, controller):
        self.controller = controller

        self.srv_set_mode = None
        self.sub_commands = None
    
    def connect(self):
    
        srv_set_mode_name = self.controller.declare_parameter('speech_recognizer_asr.service_name.set_mode', 'dflt').value
        sub_commands_name = self.controller.declare_parameter('speech_recognizer_asr.topic_name.commands', 'dflt').value

        self.sub_commands = self.controller.create_subscription(String, sub_commands_name, self.sub_commands_callback, 1)
        self.srv_set_mode = self.controller.create_client(SimpleCommand, srv_set_mode_name)

        while not self.srv_set_mode.wait_for_service(timeout_sec=self.controller.wait_timeout):
            self.controller.get_logger().info(f'Service {srv_set_mode_name} not available, waiting...')

        self.controller.get_logger().info('speech_recognizer_asr connected')
        return True
    
    def sub_commands_callback(self, msg:String):
        command = msg.data
        self.controller.get_logger().info(f'Command: |{command}| ')
    
    def set_mode(self, mode:int=0):
        # mode:
        # 0 off recognition
        # 1 commands recognition
        # 2 free recognition
        req = SimpleCommand.Request()
        req.data = mode
        
        future = self.srv_set_mode.call_async(req)
        rclpy.spin_until_future_complete(self.controller, future, timeout_sec=self.controller.wait_timeout)
        return future.result() if future.done() else None


