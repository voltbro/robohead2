#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageForwarder(Node):
    def __init__(self):
        super().__init__('image_forwarder')

        # Подписка на исходный поток
        self.sub = self.create_subscription(
            Image,
            '/robohead_controller/image_raw',
            self.image_callback,
            10)

        # Публикация в медиадрайвер
        self.pub = self.create_publisher(
            Image,
            '/robohead_controller/media_driver/stream',
            10)

        self.get_logger().info('ImageForwarder initialized — forwarding image_raw → media_driver/stream')

    def image_callback(self, msg: Image):
        # Просто пересылаем сообщение без изменений
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImageForwarder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
