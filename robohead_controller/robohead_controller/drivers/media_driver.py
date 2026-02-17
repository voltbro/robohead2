from robohead_interfaces.srv import PlayMedia, SimpleCommand
from sensor_msgs.msg import Image
import rclpy
import sys

class MediaDriverConnector:
    
    def __init__(self, controller):
        self.controller = controller

        self.srv_play_media = None
        self.srv_get_volume = None
        self.srv_set_volume = None
        self.pub_stream = None
        self.stop_command = None
    
    def connect(self):
    
        srv_set_volume_name = self.controller.declare_parameter('media_driver.srv_set_volume_name', 'set_volume').value
        srv_get_volume_name = self.controller.declare_parameter('media_driver.srv_get_volume_name', 'get_volume').value
        srv_play_media_name = self.controller.declare_parameter('media_driver.srv_play_media_name', 'play_media').value
        topic_stream_name = self.controller.declare_parameter('media_driver.topic_stream_name', 'stream').value
        self.stop_command = self.controller.declare_parameter('media_driver.stop_command', '__STOP__').value

        self.srv_play_media = self.controller.create_client(PlayMedia, srv_play_media_name)
        self.srv_get_volume = self.controller.create_client(SimpleCommand, srv_get_volume_name)
        self.srv_set_volume = self.controller.create_client(SimpleCommand, srv_set_volume_name)

        self.pub_stream = self.controller.create_publisher(Image, topic_stream_name, 1)

        while not self.srv_play_media.wait_for_service(timeout_sec=self.controller.wait_timeout):
            self.controller.get_logger().info(f'Service {srv_play_media_name} not available, waiting...')
        while not self.srv_get_volume.wait_for_service(timeout_sec=self.controller.wait_timeout):
            self.controller.get_logger().info(f'Service {srv_get_volume_name} not available, waiting...')
        while not self.srv_set_volume.wait_for_service(timeout_sec=self.controller.wait_timeout):
            self.controller.get_logger().info(f'Service {srv_set_volume_name} not available, waiting...')
        
        self.controller.get_logger().info('media_driver connected')
        return True
    
    def play_media(self, video_path:str="", audio_path:str="", loop:bool=False):
        req = PlayMedia.Request()
        req.path_to_video_file = video_path
        req.path_to_audio_file = audio_path
        req.loop = loop
        
        future = self.srv_play_media.call_async(req)
        rclpy.spin_until_future_complete(self.controller, future, timeout_sec=self.controller.wait_timeout)
        return future.result() if future.done() else None
    
    def stream_publish(self, image_msg:Image):
        """Публикация кадра в видеопоток"""
        self.pub_stream.publish(image_msg)