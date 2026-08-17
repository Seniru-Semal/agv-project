from setuptools import find_packages, setup


package_name = "agv_fleet_gate"


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
        "Incremental route-release gate "
        "for the AGV mission manager"
    ),

    license="Apache-2.0",

    entry_points={
        "console_scripts": [
            (
                "fleet_gate_node = "
                "agv_fleet_gate."
                "fleet_gate_node:main"
            ),
        ],
    },
)
