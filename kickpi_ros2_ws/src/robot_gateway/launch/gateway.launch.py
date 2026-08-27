from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成 KICKPI Web/API 网关启动描述。"""
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8080"),
            DeclareLaunchArgument("db_path", default_value="/root/robot_data/robot.db"),
            DeclareLaunchArgument("token", default_value="robot-dev-token"),
            DeclareLaunchArgument("control_timeout_s", default_value="0.3"),
            Node(
                package="robot_gateway",
                executable="gateway",
                name="robot_gateway",
                output="screen",
                parameters=[
                    {
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                        "db_path": LaunchConfiguration("db_path"),
                        "token": LaunchConfiguration("token"),
                        "control_timeout_s": LaunchConfiguration("control_timeout_s"),
                    }
                ],
            ),
        ]
    )
