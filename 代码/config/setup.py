from glob import glob
from setuptools import find_packages, setup

package_name = "home_patrol"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zyc",
    maintainer_email="zyc@example.com",
    description="Room-by-room Nav2 patrol controller for a home service robot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "patrol_node = home_patrol.patrol_node:main",
            "save_waypoint = home_patrol.waypoint_recorder:main",
            "map_http_server = home_patrol.map_http_server:main",
            "navigation_manager = home_patrol.navigation_manager:main",
        ],
    },
)
