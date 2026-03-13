from robohead_interfaces.srv import Move, SimpleCommand
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import rclpy
import sys

class GyrobroConnector:
    
    def __init__(self, controller):
        self.controller = controller

        self.gyrobro_pub_cmd_vel = None
    
    def connect(self):

        self.gyrobro_pub_cmd_vel = self.controller.create_publisher(Twist, "/cmd_vel_external", 1)
        
        self.controller.get_logger().info('gyrobro_driver connected')
        return True
    
    def cmd_vel(self, speed:Twist):
         self.gyrobro_pub_cmd_vel.publish(speed)
