"""闭环控制节点启动文件，用法见包 README。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('suspension_control_v3')
    default_params = os.path.join(pkg_share, 'config', 'control_params.yaml')

    declared_args = [
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='参数 YAML 文件路径'),
        DeclareLaunchArgument('enable_driver', default_value='true',
                              description='是否打开作动器串口（false 时只算不下发）'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyS0',
                              description='作动器串口设备'),
        DeclareLaunchArgument('baud_rate', default_value='115200',
                              description='串口波特率'),
        DeclareLaunchArgument('preview_node', default_value='true',
                              description='是否同时启动 v2 预瞄节点'),
    ]

    control_node = Node(
        package='suspension_control_v3',
        executable='suspension_control_node',
        name='suspension_control_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'enable_driver': LaunchConfiguration('enable_driver'),
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
            },
        ],
    )

    preview_node = Node(
        package='suspension_preview_v2',
        executable='road_preview_node',
        name='road_preview_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('preview_node')),
    )

    return LaunchDescription(declared_args + [preview_node, control_node])
