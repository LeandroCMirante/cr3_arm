from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("cr3_custom_gazebo", package_name="cr3_moveit_config")
        .to_moveit_configs()
    )

    # 1. Robot State Publisher (Gera os TFs virtuais a partir do URDF e do /joint_states)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # 2. Move Group (Sem sim_time, rodando em tempo real)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )

    # 3. MoveIt Servo
    servo_params_file = PathJoinSubstitution([
        FindPackageShare("cr3_moveit_config"),
        "config",
        "servo_parameters.yaml"
    ])

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params_file,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": False}
        ],
        output="screen",
    )

    return LaunchDescription([rsp_node, move_group_node, servo_node])