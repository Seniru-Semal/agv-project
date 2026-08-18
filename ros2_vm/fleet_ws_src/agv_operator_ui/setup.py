import os
from glob import glob

from setuptools import find_packages, setup


package_name = "agv_operator_ui"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seniru",
    maintainer_email="seniru@example.com",
    description="PyQt5 operator GUIs for AGV supervisor, stores, and workbenches",
    license="MIT",
    entry_points={
        "console_scripts": [
            "agv_supervisor_gui = agv_operator_ui.supervisor_gui_node:main",
            "agv_workbench_gui = agv_operator_ui.workbench_gui_node:main",
            "agv_stores_gui = agv_operator_ui.stores_gui_node:main",
        ],
    },
)
