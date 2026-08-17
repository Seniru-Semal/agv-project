import os

from glob import glob
from setuptools import (
    find_packages,
    setup,
)


package_name = (
    "agv_fleet_manager"
)


setup(
    name=package_name,
    version="0.5.0",

    packages=find_packages(
        exclude=["test"]
    ),

    data_files=[
        (
            "share/ament_index/"
            "resource_index/packages",
            [
                "resource/"
                + package_name
            ],
        ),

        (
            "share/"
            + package_name,
            [
                "package.xml"
            ],
        ),

        (
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            glob(
                "launch/*.launch.py"
            ),
        ),

        (
            os.path.join(
                "share",
                package_name,
                "config",
            ),
            glob(
                "config/*.json"
            ),
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="seniru",

    maintainer_email=(
        "seniru@todo.todo"
    ),

    description=(
        "Two-AGV exact-path fleet "
        "manager with destination "
        "admission, alternate routing, "
        "rolling reservations, dock "
        "protection and obstacle resume"
    ),

    license="Apache-2.0",

    entry_points={
        "console_scripts": [
            (
                "fleet_manager_node = "
                "agv_fleet_manager."
                "fleet_manager_node:main"
            ),

            (
                "dock_safe_fleet_manager_node = "
                "agv_fleet_manager."
                "dock_safe_fleet_manager_node:main"
            ),

            (
                "auto_resume_dock_safe_"
                "fleet_manager_node = "
                "agv_fleet_manager."
                "auto_resume_dock_safe_"
                "fleet_manager_node:main"
            ),

            (
                "planned_route_"
                "fleet_manager_node = "
                "agv_fleet_manager."
                "planned_route_"
                "fleet_manager_node:main"
            ),

            (
                "fleet_hmi_node = "
                "agv_fleet_manager."
                "fleet_hmi_node:main"
            ),
        ],
    },
)
