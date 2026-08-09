from glob import glob
import os

from setuptools import find_packages, setup

PACKAGE_NAME = "cognition"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Thecnfor",
    maintainer_email="thecnfor@users.noreply.github.com",
    description="rak Python business layer (inference bridge + business nodes)",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "inference-bridge = cognition.inference.bridge:main",
            "lane-follower = cognition.lane.lane_follower:main",
            "detector-node = cognition.detector.detector_node:main",
            "vision-overlay = cognition.visualization.vision_overlay:main",
        ],
    },
)
