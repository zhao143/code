from setuptools import setup

package_name = "robot_gateway"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/gateway.launch.py"]),
        ("share/" + package_name + "/web", ["web/index.html"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="user@example.com",
    description="HTTP/Web gateway and SQLite telemetry recorder for the KICKPI robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "gateway = robot_gateway.gateway_node:main",
        ],
    },
)
