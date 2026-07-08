from setuptools import setup

package_name = 'vehicle_wbt_smartcar_hw'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={package_name: ['arm_cfg.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['pyserial', 'pyyaml'],
    zip_safe=True,
    maintainer='RAK-Car Team',
    maintainer_email='rak-car@todo.todo',
    description='MC602 下位机协议层,SDK 字节 1:1 对齐',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={},
)
