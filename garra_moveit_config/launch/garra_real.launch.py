"""MoveIt com a garra fisica. Calibre os servos antes (scripts/calibrar_servos.py).

Nao usa generate_demo_launch: ele inclui o rsp.launch.py e o move_group.launch.py do pacote,
que recriam a configuracao com o hardware simulado e ignorariam o driver real.

Variaveis de ambiente opcionais:
    GARRA_PORTA        porta serial do Arduino (padrao: /dev/ttyUSB0)
    GARRA_CALIBRACAO   arquivo de calibracao (padrao: config/calibracao_servos.yaml do pacote)

Argumento: use_rviz:=false para rodar sem RViz.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_rsp_launch,
    generate_spawn_controllers_launch,
)


def generate_launch_description():
    pacote = get_package_share_directory('garra_moveit_config')
    moveit_config = (
        MoveItConfigsBuilder('garra_final', package_name='garra_moveit_config')
        .robot_description(mappings={
            'hardware_plugin': 'garra_hardware/GarraServoSystem',
            'porta_serial': os.environ.get('GARRA_PORTA', '/dev/ttyUSB0'),
            'calibracao_file': os.environ.get(
                'GARRA_CALIBRACAO', os.path.join(pacote, 'config', 'calibracao_servos.yaml')),
        })
        .to_moveit_configs()
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        generate_rsp_launch(moveit_config),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[os.path.join(pacote, 'config', 'ros2_controllers.yaml')],
            remappings=[('/controller_manager/robot_description', '/robot_description')],
            output='screen',
        ),
        generate_spawn_controllers_launch(moveit_config),
        generate_move_group_launch(moveit_config),
        # Sem parametros do MoveIt no RViz: com eles o modelo nao carrega. Veja moveit_rviz.launch.py.
        Node(
            package='rviz2',
            executable='rviz2',
            output='log',
            arguments=['-d', os.path.join(pacote, 'config', 'moveit.rviz')],
            parameters=[{
                'robot_description_kinematics.garra.kinematics_solver':
                    'kdl_kinematics_plugin/KDLKinematicsPlugin',
            }],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
