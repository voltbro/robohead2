import os
from setuptools import find_packages, setup
from pathlib import Path

package_name = 'turtlebro_controller'

config_dir = Path('config')
config_files = [str(p) for p in config_dir.iterdir() if p.is_file()]

launch_dir = Path('launch')
launch_files = [str(p) for p in launch_dir.iterdir() if p.is_file()]

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), config_files),
        (os.path.join('share', package_name, 'launch'), launch_files),
        ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kolezo-pi',
    maintainer_email='kolesnikovpi2005@mail.ru',
    description='Robohead voice-controlled robot controller',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = turtlebro_controller.main:main',
        ],
    },
)