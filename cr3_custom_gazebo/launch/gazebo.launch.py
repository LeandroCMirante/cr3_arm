from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_cr3_gazebo = FindPackageShare("cr3_custom_gazebo")
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")

    world_file = PathJoinSubstitution([
        pkg_cr3_gazebo,
        "worlds",
        "empty.world"
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_ros_gz_sim,
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": ["-r ", world_file]
        }.items()
    )

    return LaunchDescription([
        gazebo
    ])