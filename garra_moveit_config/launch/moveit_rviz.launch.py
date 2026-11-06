from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "garra_final",
        package_name="garra_moveit_config"
    ).to_moveit_configs()
    # So o nome do solver: com parametros numericos do MoveIt o RViz nao carrega o modelo
    # ("... of type double, setting it to string"). Limites e planejamento ficam no move_group.
    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
            parameters=[{
                "robot_description_kinematics.garra.kinematics_solver":
                    "kdl_kinematics_plugin/KDLKinematicsPlugin",
            }],
        ),
    ])
