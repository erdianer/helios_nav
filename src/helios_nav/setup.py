from glob import glob
import os

from setuptools import find_packages, setup

package_name = "helios_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="helios dev",
    maintainer_email="dev@example.com",
    description="Helios 底盘 2D 建图与导航包（slam_toolbox + map_server + amcl + nav2）",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "laser_preprocess = helios_nav.laser_preprocess:main",
            "cmd_vel_relay = helios_nav.cmd_vel_relay:main",
            "odom_publisher = helios_nav.odom_publisher:main",
        ],
    },
)
