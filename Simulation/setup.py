from setuptools import setup
import os
from glob import glob

package_name = 'async_lidar'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Required by ROS2 — do not remove these two lines
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # This line registers your launch file so:
        #   ros2 launch async_lidar launch_async_lidar.py   works
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Asynchronous lidar simulation — Glennie et al. 2025',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # FORMAT: 'command_name = package.module:main_function'
            # These become the ros2 run commands:
            #   ros2 run async_lidar transmitter_node
            #   ros2 run async_lidar receiver_node
            #   ros2 run async_lidar fusion_node
            'transmitter_node = async_lidar.transmitter_node:main',
            'receiver_node     = async_lidar.receiver_node:main',
            'fusion_node       = async_lidar.fusion_node:main',
        ],
    },
)
