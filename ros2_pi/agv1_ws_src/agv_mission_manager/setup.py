from glob import glob

from setuptools import (
    find_packages,
    setup,
)


package_name = (
    "agv_mission_manager"
)


setup(
    name=package_name,
    version="0.1.0",

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
            "share/"
            + package_name
            + "/config",
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
        "seniru@example.com"
    ),

    description=(
        "AGV1 coordinate mission manager "
        "with exact-path fleet protocol"
    ),

    license="MIT",

    tests_require=[
        "pytest",
    ],

    entry_points={
        "console_scripts": [
            (
                "mission_manager_node = "
                "agv_mission_manager."
                "mission_manager_node:main"
            ),
        ],
    },
)
