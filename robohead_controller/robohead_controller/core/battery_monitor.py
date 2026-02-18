class BatteryMonitor:
    """Мониторинг состояния батареи и реакция на низкое напряжение"""
    
    def __init__(self, controller):
        self.controller = controller
        self.controller.is_allow_work = False

        self.timer = self.controller.create_timer(0.5, self.battery_monitor_tick) # 2 раза в секунду
    
    def battery_monitor_tick(self):
        """Обработчик обновления батареи"""
        if self.controller.sensor_driver.battery_voltage == None:
            return
        if self.controller.action_manager == None:
            return
        # self.controller.get_logger().info(f"voltage {self.controller.sensor_driver.battery_voltage}")
        
        
        if self.controller.sensor_driver.battery_voltage < self.controller.low_voltage_threshold and self.controller.is_allow_work:
            self.controller.get_logger().error("LOW BATTERY! Triggering low_bat action")
            self.controller.is_allow_work = False
            self.controller.action_manager.execute_action('std_low_bat')
        
        elif not self.controller.is_allow_work and self.controller.sensor_driver.battery_voltage >= (self.controller.low_voltage_threshold + self.controller.low_voltage_hysteresis):
            self.controller.get_logger().info("Battery recovered → starting std_wait")
            self.controller.is_allow_work = True
            self.controller.action_manager.execute_action('std_wait')