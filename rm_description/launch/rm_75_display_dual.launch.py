import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            os.path.join(
                get_package_share_directory("rm_description"),
                "urdf",
                "rm_75_dual.urdf.xacro",
            ),
        ]
    )

    print(robot_description)

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                respawn=True,
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
        ]
    )
