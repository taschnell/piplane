import os
from glob import glob
from setuptools import find_packages, setup

package_name = "piplane"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools", "board", "micro_ros_agent", "ros2launch"],
    zip_safe=True,
    maintainer="taschnell",
    maintainer_email="schnellteo@hotmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "crsf_node = src.crsf_pub:main",
            "Controller = src.controller:main",
            "imu_publisher = src.imu_pub:main",
        ],
    },
)
