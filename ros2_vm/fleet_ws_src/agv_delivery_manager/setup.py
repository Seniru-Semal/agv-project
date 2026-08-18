import os
from glob import glob

from setuptools import find_packages, setup


package_name = "agv_delivery_manager"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.json")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seniru",
    maintainer_email="seniru@example.com",
    description="Delivery workflow manager for AGV workbench and stores tasks",
    license="MIT",
    entry_points={
        "console_scripts": [
            (
                "delivery_manager_node = "
                "agv_delivery_manager.delivery_manager_node:main"
            ),
        ],
    },
)
