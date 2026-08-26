from setuptools import setup

package_name = "robot_base_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/base_bridge.launch.py"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="user@example.com",
    description="ROS2 serial bridge for the STM32 robot base controller.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "base_bridge = robot_base_bridge.base_bridge_node:main",
        ],
    },
)
