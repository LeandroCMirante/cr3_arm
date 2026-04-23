from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "cr3_custom_gazebo",
            package_name="cr3_moveit_config",
        )
        .to_moveit_configs()
    )

    example_node = Node(
        package="cr3_moveit_examples",
        executable="cr3_first_program",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([example_node])