from robohead_interfaces.srv import PlayMedia, SimpleCommand
from sensor_msgs.msg import Image
import rclpy
import sys
import threading

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
        srv_is_idle_audio_name = self.controller.declare_parameter('media_driver.srv_is_idle_audio_name', 'is_idle/audio').value
        srv_is_idle_display_name = self.controller.declare_parameter('media_driver.srv_is_idle_display_name', 'is_idle/display').value


        topic_stream_name = self.controller.declare_parameter('media_driver.topic_stream_name', 'stream').value
        self.stop_command = self.controller.declare_parameter('media_driver.stop_command', '__STOP__').value

        self.srv_play_media = self.controller.create_client(PlayMedia, srv_play_media_name)
        self.srv_get_volume = self.controller.create_client(SimpleCommand, srv_get_volume_name)
        self.srv_set_volume = self.controller.create_client(SimpleCommand, srv_set_volume_name)
        self.srv_is_idle_audio = self.controller.create_client(SimpleCommand, srv_is_idle_audio_name)
        self.srv_is_idle_display = self.controller.create_client(SimpleCommand, srv_is_idle_display_name)

        self.pub_stream = self.controller.create_publisher(Image, topic_stream_name, 1)

        while not self.srv_play_media.wait_for_service(timeout_sec=self.controller.wait_timeout):
            self.controller.get_logger().info(f'Service {srv_play_media_name} not available, waiting...')
        while not self.srv_get_volume.wait_for_service(timeout_sec=self.controller.wait_timeout):
            self.controller.get_logger().info(f'Service {srv_get_volume_name} not available, waiting...')
        while not self.srv_set_volume.wait_for_service(timeout_sec=self.controller.wait_timeout):
            self.controller.get_logger().info(f'Service {srv_set_volume_name} not available, waiting...')
        while not self.srv_is_idle_audio.wait_for_service(timeout_sec=self.controller.wait_timeout):
            self.controller.get_logger().info(f'Service {srv_is_idle_audio_name} not available, waiting...')
        while not self.srv_is_idle_display.wait_for_service(timeout_sec=self.controller.wait_timeout):
            self.controller.get_logger().info(f'Service {srv_is_idle_display_name} not available, waiting...')
        
        self.controller.get_logger().info('media_driver connected')
        return True
    
    def play_media_spin(self, video_path:str="", audio_path:str="", loop:bool=False):
        req = PlayMedia.Request()
        req.path_to_video_file = video_path
        req.path_to_audio_file = audio_path
        req.loop = loop
        
        future = self.srv_play_media.call_async(req)
        rclpy.spin_until_future_complete(self.controller, future, timeout_sec=self.controller.wait_timeout)
        return future.result() if future.done() else None

    def play_media(self, cancel_event: threading.Event, video_path:str="", audio_path:str="", loop:bool=False, block:bool=True):
        req = PlayMedia.Request()
        req.path_to_video_file = video_path
        req.path_to_audio_file = audio_path
        req.loop = loop
        
        future = self.srv_play_media.call_async(req)
        while not future.done() and not cancel_event.is_set():
            self.controller.rate_.sleep()
        
        while block and not (self.is_idle_audio(cancel_event) and self.is_idle_display(cancel_event)) and not cancel_event.is_set():
            self.controller.rate_.sleep()

        return future.result() if future.done() else None

    def play_audio(self, cancel_event: threading.Event, audio_path:str="", loop:bool=False, block:bool=True):
        req = PlayMedia.Request()
        req.path_to_video_file = ""
        req.path_to_audio_file = audio_path
        req.loop = loop
        
        future = self.srv_play_media.call_async(req)
        while not future.done() and not cancel_event.is_set():
            self.controller.rate_.sleep()
        
        while block and not self.is_idle_audio(cancel_event) and not cancel_event.is_set():
            self.controller.rate_.sleep()

        return future.result() if future.done() else None

    def play_display(self, cancel_event: threading.Event, video_path:str="", loop:bool=False, block:bool=True):
        req = PlayMedia.Request()
        req.path_to_video_file = video_path
        req.path_to_audio_file = ""
        req.loop = loop
        
        future = self.srv_play_media.call_async(req)
        while not future.done() and not cancel_event.is_set():
            self.controller.rate_.sleep()
        
        while block and not self.is_idle_display(cancel_event) and not cancel_event.is_set():
            self.controller.rate_.sleep()

        return future.result() if future.done() else None

    def set_volume(self, cancel_event: threading.Event, volume:int=50):
        req = SimpleCommand.Request()
        req.data = int(volume)

        
        future = self.srv_set_volume.call_async(req)
        while not future.done() and not cancel_event.is_set():
            self.controller.rate_.sleep()

        return future.result() if future.done() else None

    def get_volume(self, cancel_event: threading.Event):
        req = SimpleCommand.Request()
        
        future = self.srv_get_volume.call_async(req)
        while not future.done() and not cancel_event.is_set():
            self.controller.rate_.sleep()

        return future.result().data if future.done() else None

    def is_idle_audio(self, cancel_event: threading.Event):
        req = SimpleCommand.Request()
        
        future = self.srv_is_idle_audio.call_async(req)
        while not future.done() and not cancel_event.is_set():
            self.controller.rate_.sleep()

        return future.result().data if future.done() else None

    def is_idle_display(self, cancel_event: threading.Event):
        req = SimpleCommand.Request()
        
        future = self.srv_is_idle_display.call_async(req)
        while not future.done() and not cancel_event.is_set():
            self.controller.rate_.sleep()

        return future.result().data if future.done() else None
    
    def stream_publish(self, image_msg:Image):
        """Публикация кадра в видеопоток"""
        self.pub_stream.publish(image_msg)
    
    