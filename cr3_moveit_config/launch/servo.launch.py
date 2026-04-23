from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Carrega as configurações do MoveIt (URDF, SRDF, Kinematics, etc)
    moveit_config = (
        MoveItConfigsBuilder("cr3_custom_gazebo", package_name="cr3_moveit_config")
        .to_moveit_configs()
    )

    # Aponta para o seu arquivo de parâmetros do servo
    servo_params_file = PathJoinSubstitution([
        FindPackageShare("cr3_moveit_config"),
        "config",
        "servo_parameters.yaml"
    ])

    # Cria o nó do MoveIt Servo
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params_file,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True} # Essencial para funcionar com o Gazebo
        ],
        output="screen",
    )

    return LaunchDescription([servo_node])