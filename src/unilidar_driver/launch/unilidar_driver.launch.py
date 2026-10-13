from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    local_port = LaunchConfiguration("local_port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("unilidar_driver"), "config", "unilidar_params.yaml"]
            ),
            description="驱动参数文件",
        ),
        DeclareLaunchArgument(
            "local_port",
            default_value="6201",
            description="本机监听 UDP 端口",
        ),
        Node(
            package="unilidar_driver",
            executable="unilidar_driver_node",
            name="unilidar_driver_node",
            output="screen",
            parameters=[params_file, {"local_port": local_port}],
        ),
    ])
