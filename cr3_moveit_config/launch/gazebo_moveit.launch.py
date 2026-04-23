from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "cr3_custom_gazebo",
            package_name="cr3_moveit_config",
        )
        .to_moveit_configs()
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("cr3_custom_gazebo"),
                "launch",
                "cr3_gazebo.launch.py",
            ])
        )
    )

    publish_monitored_planning_scene = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(LaunchConfiguration("disable_capabilities"), value_type=str),
        "publish_planning_scene": publish_monitored_planning_scene,
        "publish_geometry_updates": publish_monitored_planning_scene,
        "publish_state_updates": publish_monitored_planning_scene,
        "publish_transforms_updates": publish_monitored_planning_scene,
        "monitor_dynamics": False,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_configuration,
            {"use_sim_time": True},
        ],
    )

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
        DeclareLaunchArgument("allow_trajectory_execution", default_value="true"),
        DeclareLaunchArgument("publish_monitored_planning_scene", default_value="true"),
        DeclareLaunchArgument("capabilities", default_value=""),
        DeclareLaunchArgument("disable_capabilities", default_value=""),

        gazebo_launch,
        move_group_node,
        rviz_node,
    ])