from setuptools import find_packages, setup

package_name = "agv_rfid"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seniru",
    maintainer_email="seniru@todo.todo",
    description="RFID reader node for AGV fleet node identification",
    license="TODO",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "rfid_reader_node = agv_rfid.rfid_reader_node:main",
        ],
    },
)
