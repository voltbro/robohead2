# from setuptools import find_packages, setup

# package_name = 'speech_recognizer'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='pi',
#     maintainer_email='kolesnikovpi2005@mail.ru',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )


from setuptools import setup

package_name = 'speech_recognizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'vosk'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Vosk-based Russian speech recognizer for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_node = speech_recognizer.speech_node:main',
        ],
    },
)