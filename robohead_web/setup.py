import os
from pathlib import Path
from setuptools import find_packages, setup

package_name = 'robohead_web'
launch_dir = Path('launch')
launch_files = [str(p) for p in launch_dir.iterdir() if p.is_file()] if launch_dir.exists() else []
static_dir = Path('robohead_web/static')
static_files = [str(p) for p in static_dir.rglob('*') if p.is_file()] if static_dir.exists() else []

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), launch_files),
        (os.path.join('share', package_name, 'static'), static_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kolezo-pi',
    maintainer_email='kolesnikovpi2005@mail.ru',
    description='Web interface for Robohead ROS2 controls',
    license='TODO',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['server = robohead_web.server:main']},
)
