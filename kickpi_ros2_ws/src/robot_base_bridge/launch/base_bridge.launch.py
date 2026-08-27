from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyAS5"),
            DeclareLaunchArgument("baud", default_value="115200"),
            Node(
                package="robot_base_bridge",
                executable="base_bridge",
                name="base_bridge",
                output="screen",
                parameters=[
                    {
                        "port": LaunchConfiguration("port"),
                        "baud": LaunchConfiguration("baud"),
                    }
                ],
            ),
        ]
    )
