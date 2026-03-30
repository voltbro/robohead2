from sensor_msgs.msg import Image
import rclpy
import sys

class UsbCamConnector:
    
    def __init__(self, controller):
        self.controller = controller

        self.sub_image_raw = None
        self.image_raw = None
    
    def connect(self):
    
        sub_image_raw_name = self.controller.declare_parameter('usb_cam.topic_name_image_raw', 'dflt').value

        self.sub_image_raw = self.controller.create_subscription(Image, sub_image_raw_name, self.sub_image_raw_callback, 1)

        self.controller.get_logger().info('usb_cam connected')
        return True
    
    def sub_image_raw_callback(self, msg:Image):
        self.image_raw = msg

        # self.controller.get_logger().info(f'Image! ')


