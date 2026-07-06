from glob import glob
import os

from setuptools import find_packages, setup

PACKAGE_NAME = "vehicle_wbt_platform"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="Thecnfor",
    maintainer_email="thecnfor@users.noreply.github.com",
    description="Platform-level ROS2 sidecar for vehicle_wbt",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sidecar = vehicle_wbt_platform.__main__:main",
        ],
    },
)
