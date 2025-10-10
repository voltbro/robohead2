from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'respeaker_driver'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kolezo_pi',
    maintainer_email='kolesnikovpi2005@mail.ru',
    description='ROS respeaker usb mic array package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = respeaker_driver.main:main',
            'example_record = respeaker_driver.example_record:main',
        ],
    },
)
