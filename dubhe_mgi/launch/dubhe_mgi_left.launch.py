from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ns_arg = DeclareLaunchArgument("ns", default_value="")
    group_arg = DeclareLaunchArgument("planning_group", default_value="left_arm")
    ref_arg = DeclareLaunchArgument("reference_frame", default_value="lift")

    ns = LaunchConfiguration("ns")
    group = LaunchConfiguration("planning_group")
    ref = LaunchConfiguration("reference_frame")

    moveit_config = (
        MoveItConfigsBuilder("rm_75_dual", package_name="rm_75_config")
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner", "chomp"])
        .to_moveit_configs()
    )

    client = Node(
        package="dubhe_mgi",
        executable="dubhe_mgi_node",
        # namespace=ns,
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"planning_group": group, "reference_frame": ref, "robot_ns": ns},
        ],
        arguments=["--ros-args", "--log-level", "debug"],
    )

    return LaunchDescription([ns_arg, group_arg, ref_arg, client])
