"""ajuste_base: executa /robo/mover pelo canal 'especial' do arbitro do robo.

    ros2 launch garra_coleta ajuste_base.launch.py
    ros2 launch garra_coleta ajuste_base.launch.py sem_arbitro:=true topico_cmd:=/cmd_vel  # simulacao
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import OpaqueFunction

sys.path.insert(0, os.path.dirname(__file__))
from coleta_comum import SIM, argumentos, no, valores  # noqa: E402

ARGS = [
    ('vel', '0.05', 'm/s maximo'),
    ('vel_giro', '0.30', 'rad/s maximo'),
    ('max_desloc', '0.15', 'm maximo por eixo por pedido'),
    ('max_giro', '30', 'graus maximo por pedido'),
    ('timeout', '15', 's maximo por pedido'),
    ('topico_cmd', '/cmd_vel_nav_especial', 'onde publicar a velocidade'),
    ('sem_arbitro', 'false', 'nao chama /navegacao/especial (so simulacao)'),
]


def montar(context):
    a = valores(context, ARGS)
    extra = []
    for n in ('vel', 'vel_giro', 'max_desloc', 'max_giro', 'timeout', 'topico_cmd'):
        extra += [f'--{n.replace("_", "-")}', a[n]]
    if a['sem_arbitro'] in SIM:
        extra.append('--sem-arbitro')
    return [no('ajuste_base', *extra)]


def generate_launch_description():
    return LaunchDescription(argumentos(ARGS) + [OpaqueFunction(function=montar)])
