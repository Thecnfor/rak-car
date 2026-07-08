"""Setup script for vehicle_wbt_smartcar_sdk (dev-box-side Python SDK)."""
from setuptools import find_packages, setup

PACKAGE_NAME = 'vehicle_wbt_smartcar_sdk'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'resource/{PACKAGE_NAME}']),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Thecnfor',
    maintainer_email='w5555wdnmd@gmail.com',
    description='Dev-box SDK: MyCar API 1:1 mirror, talks ROS2 to the bridge',
    license='Proprietary',
    tests_require=['pytest'],
)