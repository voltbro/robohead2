from robohead_interfaces.srv import Move
from sensor_msgs.msg import Image
import rclpy
import sys

class NeckDriverConnector:
    
    def __init__(self, controller):
        self.controller = controller

        self.srv_neck_set_angle = None
    
    def connect(self):
    
        srv_neck_set_angle_name = self.controller.declare_parameter('neck_driver.srv_neck_set_angle_name', 'dflt').value


        self.srv_neck_set_angle = self.controller.create_client(Move, srv_neck_set_angle_name)

        while not self.srv_neck_set_angle.wait_for_service(timeout_sec=self.controller.wait_timeout):
            self.controller.get_logger().info(f'Service {srv_neck_set_angle_name} not available, waiting...')
        
        self.controller.get_logger().info('neck_driver connected')
        return True
    
    def set_angle(self, vertical:int=0, horizontal:int=0, duration:float=1.0, is_block:bool=True):
        req = Move.Request()
        req.angle_a = vertical
        req.angle_b = horizontal
        req.duration = duration
        req.is_block = is_block

        future = self.srv_neck_set_angle.call_async(req)
        rclpy.spin_until_future_complete(self.controller, future, timeout_sec=self.controller.wait_timeout)
        return future.result() if future.done() else None