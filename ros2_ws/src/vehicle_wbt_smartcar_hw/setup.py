"""Setup script for vehicle_wbt_smartcar_hw."""
from setuptools import find_packages, setup

PACKAGE_NAME = 'vehicle_wbt_smartcar_hw'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'resource/{PACKAGE_NAME}']),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['pyserial'],
    zip_safe=True,
    maintainer='Thecnfor',
    maintainer_email='w5555wdnmd@gmail.com',
    description='Hardware protocol layer: MC602 controller over pyserial',
    license='Proprietary',
    tests_require=['pytest'],
)