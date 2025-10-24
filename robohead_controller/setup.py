from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robohead_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Устанавливаем launch-файлы
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Устанавливаем config-файлы
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # ❌ НЕ устанавливаем actions/ — чтобы можно было редактировать без пересборки
        # Если захотите — раскомментируйте:
        # (os.path.join('share', package_name, 'actions'), glob('actions/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='kolesnikovpi2005@mail.ru',
    description='Robohead voice-controlled robot controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = robohead_controller.main:main',
            # Можно добавить и launch-скрипт, но обычно запускают через ros2 launch
        ],
    },
)