from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveIt config package
    moveit_config = (
        MoveItConfigsBuilder(
            "cr3_custom_gazebo",
            package_name="cr3_moveit_config",
        )
        .to_moveit_configs()
    )

    # Reuse the Gazebo launch that already:
    # - opens Gazebo
    # - spawns the robot
    # - loads joint_state_broadcaster
    # - loads cr3_arm_controller
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("cr3_custom_gazebo"),
                "launch",
                "cr3_gazebo.launch.py",
            ])
        )
    )

    # move_group for planning/execution
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    # RViz with MoveIt plugin
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("cr3_moveit_config"),
        "config",
        "moveit.rviz",
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        gazebo_launch,
        move_group_node,
        rviz_node,
    ])