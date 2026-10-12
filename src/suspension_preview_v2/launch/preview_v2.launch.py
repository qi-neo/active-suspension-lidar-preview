"""路面前瞄节点（v2）启动文件。

用法示例:
    # 默认（读 config/preview_params.yaml，串口下发开启）
    ros2 launch suspension_preview_v2 preview_v2.launch.py

    # 只发 ROS 话题，不碰串口（推荐的第一次验证方式）
    ros2 launch suspension_preview_v2 preview_v2.launch.py enable_serial:=false

    # 现场临时改参数
    ros2 launch suspension_preview_v2 preview_v2.launch.py baud_rate:=460800 preview_dist:=8.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('suspension_preview_v2')
    default_params = os.path.join(pkg_share, 'config', 'preview_params.yaml')

    declared_args = [
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'enable_serial', default_value='true',
            description='是否打开串口下发（false 时只发 ROS 话题）'),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyS0',
            description='串口设备路径'),
        DeclareLaunchArgument(
            'baud_rate', default_value='115200',
            description='串口波特率'),
        DeclareLaunchArgument(
            'preview_dist', default_value='5.0',
            description='预瞄距离 (m)'),
    ]

    node = Node(
        package='suspension_preview_v2',
        executable='road_preview_node',
        name='road_preview_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'enable_serial': LaunchConfiguration('enable_serial'),
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'preview_dist': LaunchConfiguration('preview_dist'),
            },
        ],
    )

    return LaunchDescription(declared_args + [node])
