from sensor_msgs.msg import BatteryState
import rclpy
import sys

class SensorDriverConnector:
    
    def __init__(self, controller):
        self.controller = controller

        self.sub_battery_state = None

        self.msg_battery_state = None
        self.battery_voltage = None
        self.battery_current = None
        self.battery_power_supply_status = None
        self.battery_power_supply_technology = None
    
    def connect(self):
    
        sub_battery_state_name = self.controller.declare_parameter('sensor_driver.topic_name', 'dflt').value

        self.sub_battery_state = self.controller.create_subscription(BatteryState, sub_battery_state_name, self.sub_battery_state_callback, 1)

        self.controller.get_logger().info('sensor_driver connected')
        return True
    
    def sub_battery_state_callback(self, msg:BatteryState):
        self.msg_battery_state = msg
        self.battery_voltage = msg.voltage
        self.battery_current = msg.current
        self.battery_power_supply_status = msg.power_supply_status
        self.battery_power_supply_technology = msg.power_supply_technology

        # self.controller.get_logger().info(f'voltage {self.battery_voltage} ')


