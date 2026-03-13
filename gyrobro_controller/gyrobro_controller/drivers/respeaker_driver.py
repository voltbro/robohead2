from robohead_interfaces.srv import SimpleCommand, Color, ColorPalette
from robohead_interfaces.msg import AudioData
from std_msgs.msg import Int32
import rclpy
import sys

class RespeakerDriverConnector:
    
    def __init__(self, controller):
        self.controller = controller

        self.sub_audio_main = None
        self.audio_main = None
        self.sub_audio_channel_0 = None
        self.sub_audio_channel_1 = None
        self.sub_audio_channel_2 = None
        self.sub_audio_channel_3 = None
        self.sub_audio_channel_4 = None
        self.sub_audio_channel_5 = None
        self.sub_doa = None
        self.doa = None
        
        self.pub_set_color_manual = None

        self.srv_set_brightness = None
        self.srv_set_color_all = None
        self.srv_set_color_palette = None
        self.srv_set_mode = None
    
    def connect(self):
    
        sub_audio_main_name = self.controller.declare_parameter('respeaker_driver.topic_name.audio_main', 'dflt').value
        sub_audio_channel_0_name = self.controller.declare_parameter('respeaker_driver.topic_name.audio_channel_0', 'dflt').value
        sub_audio_channel_1_name = self.controller.declare_parameter('respeaker_driver.topic_name.audio_channel_1', 'dflt').value
        sub_audio_channel_2_name = self.controller.declare_parameter('respeaker_driver.topic_name.audio_channel_2', 'dflt').value
        sub_audio_channel_3_name = self.controller.declare_parameter('respeaker_driver.topic_name.audio_channel_3', 'dflt').value
        sub_audio_channel_4_name = self.controller.declare_parameter('respeaker_driver.topic_name.audio_channel_4', 'dflt').value
        sub_audio_channel_5_name = self.controller.declare_parameter('respeaker_driver.topic_name.audio_channel_5', 'dflt').value
        
        sub_doa_name = self.controller.declare_parameter('respeaker_driver.topic_name.doa', 'dflt').value
        pub_set_color_manual_name = self.controller.declare_parameter('respeaker_driver.topic_name.set_color_manual', 'dflt').value

        srv_set_brightness_name = self.controller.declare_parameter('respeaker_driver.service_name.set_brightness', 'dflt').value
        srv_set_color_all_name = self.controller.declare_parameter('respeaker_driver.service_name.set_color_all', 'dflt').value
        srv_set_color_palette_name = self.controller.declare_parameter('respeaker_driver.service_name.set_color_palette', 'dflt').value
        srv_set_mode_name = self.controller.declare_parameter('respeaker_driver.service_name.set_mode', 'dflt').value

        self.srv_set_brightness = self.controller.create_client(SimpleCommand, srv_set_brightness_name)
        self.srv_set_color_all = self.controller.create_client(Color, srv_set_color_all_name)
        self.srv_set_color_palette = self.controller.create_client(ColorPalette, srv_set_color_palette_name)
        self.srv_set_mode = self.controller.create_client(SimpleCommand, srv_set_mode_name)

        self.sub_audio_main = self.controller.create_subscription(AudioData, sub_audio_main_name, self.sub_audio_main_callback, 1)
        self.sub_doa = self.controller.create_subscription(Int32, sub_doa_name, self.sub_doa_callback, 1)


        while not self.srv_set_brightness.wait_for_service(timeout_sec=self.controller.wait_timeout):
            self.controller.get_logger().info(f'Service {srv_set_brightness_name} not available, waiting...')
        while not self.srv_set_color_all.wait_for_service(timeout_sec=self.controller.wait_timeout):
            self.controller.get_logger().info(f'Service {srv_set_color_all_name} not available, waiting...')
        while not self.srv_set_color_palette.wait_for_service(timeout_sec=self.controller.wait_timeout):
            self.controller.get_logger().info(f'Service {srv_set_color_palette_name} not available, waiting...')
        while not self.srv_set_mode.wait_for_service(timeout_sec=self.controller.wait_timeout):
            self.controller.get_logger().info(f'Service {srv_set_mode_name} not available, waiting...')
        
        self.controller.get_logger().info('respeaker_driver connected')
        return True
    
    def sub_audio_main_callback(self, msg):
        self.audio_main = msg.data

    def sub_doa_callback(self, msg):
        self.doa = msg.data
        # self.controller.get_logger().info(f'Doa: {self.doa}')