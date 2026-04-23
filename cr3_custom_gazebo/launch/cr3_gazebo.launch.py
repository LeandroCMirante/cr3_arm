from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo = FindPackageShare("cr3_custom_gazebo")
    pkg_description = FindPackageShare("cr3_custom_description")
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")

    world_file = PathJoinSubstitution([
        pkg_gazebo,
        "worlds",
        "empty.world"
    ])

    xacro_file = PathJoinSubstitution([
        pkg_description,
        "urdf",
        "robots",
        "cr3_custom_gazebo.urdf.xacro"
    ])

    robot_description = {
        "robot_description": Command(["xacro ", xacro_file])
    }

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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen"
    )

    # Spawn the robot from the robot_description parameter
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "cr3_custom",
            "-topic", "robot_description",
            "-world", "default",
            "-z", "0.05"
        ],
        output="screen"
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "joint_state_broadcaster"
        ],
        output="screen"
    )

    load_arm_controller = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "cr3_arm_controller"
        ],
        output="screen"
    )

    delayed_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_joint_state_broadcaster]
        )
    )

    delayed_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_controller]
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
    ])