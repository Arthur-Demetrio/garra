"""Atalho: sobe a coleta inteira incluindo os launchers individuais.

    pegar_cubo.launch.py + ajuste_base.launch.py + alcance_cubo.launch.py
    + camera_cubo.launch.py (camera:=camera_cubo) ou tf_camera.launch.py (camera:=vision)

Cada parte tambem pode ser subida sozinha, em terminais separados (veja cada arquivo).
Precisa do MoveIt da garra rodando, ou use garra:=real / garra:=demo para incluir aqui. A coleta so
comeca quando chega /coleta/pedido (executor_prova.py do robo); fora disso a base nao se mexe.

Argumentos:
    garra:=nenhuma|real|demo            inclui o MoveIt da garra (padrao nenhuma: ja esta rodando)
    camera:=camera_cubo|vision|nenhuma  padrao camera_cubo; com vision, suba o cube_vision em outro
                                        terminal (com o .venv da camera)
    base:=true|false                    roda o ajuste_base.py (padrao true)
    girar, sempre_ativo, id             repassados ao alcance_cubo.launch.py
    apoio                               altura da superficie dos cubos, repassada ao camera_cubo
    velocidade                          repassado ao pegar_cubo.launch.py

Exemplos:
    ros2 launch garra_coleta coleta.launch.py garra:=real
    ros2 launch garra_coleta coleta.launch.py camera:=vision
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def montar(context):
    a = {k: LaunchConfiguration(k).perform(context) for k in
         ('garra', 'camera', 'base', 'girar', 'sempre_ativo', 'velocidade', 'id', 'apoio')}
    pasta = os.path.join(get_package_share_directory('garra_coleta'), 'launch')
    moveit = os.path.join(get_package_share_directory('garra_moveit_config'), 'launch')

    def incluir(arq, pasta=pasta, **args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pasta, arq)),
                                        launch_arguments=args.items())

    acoes = []
    if a['garra'] == 'real':
        acoes.append(incluir('garra_real.launch.py', pasta=moveit))
    elif a['garra'] == 'demo':
        acoes.append(incluir('demo.launch.py', pasta=moveit))
    acoes.append(incluir('pegar_cubo.launch.py', velocidade=a['velocidade']))
    if a['base'] in ('true', '1', 'sim'):
        acoes.append(incluir('ajuste_base.launch.py'))
    vision = 'true' if a['camera'] == 'vision' else 'false'
    acoes.append(incluir('alcance_cubo.launch.py', vision=vision, girar=a['girar'],
                         sempre_ativo=a['sempre_ativo'], id=a['id']))
    if a['camera'] == 'vision':
        acoes.append(incluir('tf_camera.launch.py'))
    elif a['camera'] == 'camera_cubo':
        acoes.append(incluir('camera_cubo.launch.py', id=a['id'], apoio=a['apoio']))
    return acoes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('garra', default_value='nenhuma', choices=['nenhuma', 'real', 'demo']),
        DeclareLaunchArgument('camera', default_value='camera_cubo',
                              choices=['camera_cubo', 'vision', 'nenhuma']),
        DeclareLaunchArgument('base', default_value='true'),
        DeclareLaunchArgument('girar', default_value='false'),
        DeclareLaunchArgument('sempre_ativo', default_value='false'),
        DeclareLaunchArgument('velocidade', default_value='0.2'),
        DeclareLaunchArgument('id', default_value=''),
        DeclareLaunchArgument('apoio', default_value='0.0',
                              description='altura (m) da superficie dos cubos (camera_cubo): 0 = chao'),
        OpaqueFunction(function=montar),
    ])
