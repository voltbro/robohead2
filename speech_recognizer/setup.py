from setuptools import setup
import os
from pathlib import Path

package_name = 'speech_recognizer'

config_dir = Path('config')
config_files = [str(p) for p in config_dir.iterdir() if p.is_file()]

launch_dir = Path('launch')
launch_files = [str(p) for p in launch_dir.iterdir() if p.is_file()]

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), config_files),
        (os.path.join('share', package_name, 'launch'), launch_files),
    ],
    install_requires=['setuptools', 'vosk'],
    zip_safe=False,
    maintainer='kolez-pi',
    maintainer_email='kolesnikovpi2005@mail.ru',
    description='Vosk-based Russian speech recognizer for ROS 2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'{package_name}_node = {package_name}.main:main',
            f'{package_name}_kws_node = {package_name}.kws:main',
            f'{package_name}_asr_node = {package_name}.asr:main',

        ],
    },
)