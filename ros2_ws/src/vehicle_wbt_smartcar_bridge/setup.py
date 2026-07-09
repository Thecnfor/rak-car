"""Setup script for vehicle_wbt_smartcar_bridge."""
import os
from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = 'vehicle_wbt_smartcar_bridge'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'resource/{PACKAGE_NAME}']),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join('share', PACKAGE_NAME, 'config'), glob('config/*.yaml')),
        (os.path.join('share', PACKAGE_NAME, 'launch'), glob('launch/*.launch.py')),
        # Install the entry-point script under lib/<pkg>/ where
        # `ros2 launch` and `ros2 run` expect to find it. The script
        # is a real .py file checked into script/, not a setuptools
        # entry_point (which would land in bin/ and not be discoverable).
        (os.path.join('lib', PACKAGE_NAME), glob('script/*')),
    ],
    install_requires=['setuptools', 'vehicle_wbt_smartcar_hw', 'vehicle_wbt_smartcar_msgs'],
    zip_safe=True,
    maintainer='Thecnfor',
    maintainer_email='w5555wdnmd@gmail.com',
    description='ROS2 service bridge: Baidu SmartCar 2026 MyCar API → ROS2 primitives',
    license='Proprietary',
    tests_require=['pytest'],
)