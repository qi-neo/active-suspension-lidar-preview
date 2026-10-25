from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    local_port = LaunchConfiguration("local_port")
    transport = LaunchConfiguration("transport")
    serial_device = LaunchConfiguration("serial_device")
    serial_baud = LaunchConfiguration("serial_baud")

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
            description="本机监听 UDP 端口（transport=udp 时生效）",
        ),
        DeclareLaunchArgument(
            "transport",
            default_value="serial",
            description="传输层：serial=L1 原生 TTL UART 经 CH340 类模块转 USB（实物默认）；"
                        "udp=以太网备选",
        ),
        DeclareLaunchArgument(
            "serial_device",
            default_value="/dev/ttyUSB0",
            description="串口设备名（transport=serial 时生效）",
        ),
        DeclareLaunchArgument(
            "serial_baud",
            default_value="2000000",
            description="串口波特率（transport=serial 时生效，L1 官方 SDK 默认 2000000）",
        ),
        Node(
            package="unilidar_driver",
            executable="unilidar_driver_node",
            name="unilidar_driver_node",
            output="screen",
            parameters=[params_file, {
                "local_port": local_port,
                "transport": transport,
                "serial_device": serial_device,
                "serial_baud": serial_baud,
            }],
        ),
    ])
