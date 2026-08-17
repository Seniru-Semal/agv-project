from setuptools import find_packages, setup

package_name = "agv_safety"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seniru",
    maintainer_email="seniru@example.com",
    description="Safety manager for AGV robot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "safety_manager_node = agv_safety.safety_manager_node:main",
        ],
    },
)
