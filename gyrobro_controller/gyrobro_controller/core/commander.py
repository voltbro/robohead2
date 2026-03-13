
class Commander():
    def __init__(self, controller):

        self.controller = controller
 
        self.state = 'wait_wake_phrase'

        self.timer = self.controller.create_timer(0.1, self.queue_tick) # 2 раза в секунду
    
    def queue_tick(self):
        """Обработчик команд"""

        if len(self.controller.queue_fast_commands) > 0:
            fast_command = self.controller.queue_fast_commands.pop(0)
            self.controller.action_manager.execute_action(fast_command, None, False)


        if self.state == 'wait_wake_phrase':
            if len(self.controller.queue_wake_phrases) > 0:
                self.controller.action_manager.execute_action("std_attention", None, True)
                self.state = 'wait_command'
                # self.controller.get_logger().info("comander 1")
                self.controller.queue_wake_phrases.clear()
                # self.controller.get_logger().info("comander 2")

                self.controller.speech_recognizer_asr.set_mode(1)
                # self.controller.get_logger().info("comander 3")

                self.controller.speech_recognizer_kws.set_mode(0)
                # self.controller.get_logger().info("comander 4")
            


        elif self.state == 'wait_command':
            if len(self.controller.queue_commands) > 0:
                # self.controller.get_logger().info("comander 5")
                self.state = 'wait_wake_phrase'
                self.controller.speech_recognizer_kws.set_mode(1)
                # self.controller.get_logger().info("comander 6")

                command = self.controller.queue_commands.pop(0)
                # self.controller.get_logger().info("comander 7")

                if command != self.controller.speech_recognizer_asr.timeout_text:
                    self.controller.action_manager.execute_action(command, 'std_wait', True)
                else:
                    self.controller.action_manager.execute_action("std_wait", None, True)
                # self.controller.get_logger().info("comander 8")
                
                self.controller.queue_commands.clear()
                



