import pprint
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rm_75_dual", package_name="rm_75_config")
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner", "chomp"])
        .to_moveit_configs()
    )

    ld = LaunchDescription()

    # 启动rviz
    my_generate_moveit_rviz_launch(ld, moveit_config)

    # generate_rsp_launch(ld, moveit_config)

    return ld


def my_generate_moveit_rviz_launch(ld, moveit_config):
    """Launch file for rviz"""

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=True))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    rviz_parameters = [
        moveit_config.to_dict(),
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
    ]

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=[
            "-d",
            LaunchConfiguration("rviz_config"),
        ],
        parameters=rviz_parameters,
    )

    return ld
