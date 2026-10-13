from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # v1 的参数都是 src/preview_node.cpp 顶部的宏，改参数要重新编译，
    # 这里只有节点名与话题重映射是运行时可调的。
    cloud_topic = LaunchConfiguration("cloud_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "cloud_topic",
            default_value="/unilidar/cloud",
            description="订阅的点云话题",
        ),
        Node(
            package="suspension_preview",
            executable="preview_node",
            name="suspension_preview_node",
            output="screen",
            remappings=[("/unilidar/cloud", cloud_topic)],
        ),
    ])
