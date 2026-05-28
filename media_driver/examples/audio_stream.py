#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robohead_interfaces.msg import AudioData
import numpy as np

class SweepAudioPublisher(Node):
    def __init__(self):
        super().__init__('sweep_audio_publisher')
        
        # Топик должен совпадать с подпиской в media_driver
        self.pub = self.create_publisher(AudioData, '/media_driver/audio_stream', 10)
        
        # Частота публикации: 50 Гц → чанки по 20 мс
        self.timer = self.create_timer(0.02, self.publish_sweep)
        
        self.sample_rate = 16000
        self.chunk_duration = 0.02  # 20 мс
        self.chunk_size = int(self.sample_rate * self.chunk_duration)
        
        # Параметры синусоидального качания частоты
        self.fc = 1000.0       # Центральная частота (Гц)
        self.df = 500.0       # Девиация (амплитуда качания: ±Гц)
        self.fm = 0.5         # Частота модуляции (цикл вверх-вниз за 1/fm секунд)
        # Пример: частота будет плавно меняться от 500 Гц до 1500 Гц с периодом 2 сек
        
        self.current_time = 0.0
        self.get_logger().info(f'Sweep: центр={self.fc} Гц, размах=±{self.df} Гц, цикл={1/self.fm:.1f} сек')

    def publish_sweep(self):
        # Временная ось текущего чанка
        t_chunk = np.linspace(0, self.chunk_duration, self.chunk_size, endpoint=False)
        t_global = self.current_time + t_chunk
        
        # Интегрируем мгновенную частоту для получения фазы
        # f(t) = fc + df * sin(2π·fm·t)
        # phase(t) = 2π·f(t)dt = 2π·[ fc·t + (df/fm)·(1 - cos(2π·fm·t)) ]
        phase = 2 * np.pi * (
            self.fc * t_global + 
            (self.df / self.fm) * (1 - np.cos(2 * np.pi * self.fm * t_global))
        )
        
        # Генерируем сигнал и конвертируем в int16
        wave = np.sin(phase)
        int16_data = (wave * 32767 / 10).astype(np.int16)
        
        msg = AudioData()
        msg.data = int16_data.tolist()
        self.pub.publish(msg)
        
        # Сдвигаем глобальное время для следующего чанка
        self.current_time += self.chunk_duration

def main(args=None):
    rclpy.init(args=args)
    node = SweepAudioPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()