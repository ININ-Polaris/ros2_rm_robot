import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    left_config = os.path.join(
        get_package_share_directory("rm_driver"), "config", "rm_75_config_left.yaml"
    )

    right_config = os.path.join(
        get_package_share_directory("rm_driver"), "config", "rm_75_config_right.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="rm_driver",  # 功能包。
                executable="rm_driver",  # 节点。
                parameters=[left_config],  # 接入参数文件
                output="screen",
                namespace="left",
                remappings=[("/left/joint_states", "/joint_states")],
            ),
            Node(
                package="rm_driver",  # 功能包。
                executable="rm_driver",  # 节点。
                parameters=[right_config],  # 接入参数文件
                output="screen",
                namespace="right",
                remappings=[("/right/joint_states", "/joint_states")],
            ),
        ]
    )
