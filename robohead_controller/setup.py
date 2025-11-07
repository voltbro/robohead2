import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robohead_controller'

# Путь к директории setup.py
here = os.path.abspath(os.path.dirname(__file__))

# Собираем действия
action_data_files = []
actions_path = os.path.join(here, 'actions')
if os.path.exists(actions_path):
    for root, dirs, files in os.walk(actions_path):
        for file in files:
            source = os.path.join("/".join(root.split('/')[root.split('/').index(package_name)+1:]), file)
            # print(root)
            # print(dirs)
            # print(files)
            rel_root = os.path.relpath(root, here)
            dest = os.path.join('share', package_name, rel_root)
            action_data_files.append((dest, [source]))
else:
    print(f"WARNING: 'actions' directory not found at {actions_path}")


# print("OOOOO: ", action_data_files)



setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'launch'), ['launch/speech_recognizer.launch.py']),
        (os.path.join('share', package_name, 'config'), ['config/speech_recognizer.yaml']),
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
        ],
    },
)