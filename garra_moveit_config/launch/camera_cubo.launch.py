"""scripts/camera_cubo.py: webcam + AprilTag (abre uma janela). Publica em /cubo_detectado.

    ros2 launch garra_moveit_config camera_cubo.launch.py
    ros2 launch garra_moveit_config camera_cubo.launch.py auto:=false topico:=/pegar_cubo  # direto na garra
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import OpaqueFunction

sys.path.insert(0, os.path.dirname(__file__))
from coleta_comum import SIM, argumentos, script, valores  # noqa: E402

ARGS = [
    ('topico', '/cubo_detectado', 'onde publicar (o alcance_cubo.py escuta /cubo_detectado)'),
    ('auto', 'true', 'manda o cubo sozinho quando ele fica parado (sem apertar P)'),
    ('camera', '', 'indice da webcam (vazio = acha a "Web Camera")'),
    ('id', '', 'ID do AprilTag do cubo (vazio = o mais perto)'),
]


def montar(context):
    a = valores(context, ARGS)
    extra = ['--topico', a['topico']]
    if a['auto'] in SIM:
        extra.append('--auto')
    for n in ('camera', 'id'):
        if a[n]:
            extra += [f'--{n}', a[n]]
    return [script('camera_cubo.py', *extra)]


def generate_launch_description():
    return LaunchDescription(argumentos(ARGS) + [OpaqueFunction(function=montar)])
