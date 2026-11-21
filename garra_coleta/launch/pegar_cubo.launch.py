"""pegar_cubo escutando /pegar_cubo (precisa do MoveIt da garra rodando).

    ros2 launch garra_coleta pegar_cubo.launch.py
    ros2 launch garra_coleta pegar_cubo.launch.py velocidade:=0.1 so_planejar:=true
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import OpaqueFunction

sys.path.insert(0, os.path.dirname(__file__))
from coleta_comum import SIM, argumentos, no, valores  # noqa: E402

ARGS = [
    ('velocidade', '0.2', 'escala de velocidade da garra (0-1)'),
    ('fechamento', '', 'graus do J4 ao fechar no cubo (vazio = padrao do script)'),
    ('descer', '', 'mm extras de descida antes de fechar (vazio = padrao do script)'),
    ('so_planejar', 'false', 'so planeja, nao mexe a garra'),
    # 2026-11-21: chave dos dedos retirada (nao fechava nem apertada a mao): sem sensor por padrao.
    # Ao recolocar a chave, volte para 'false'.
    ('sem_sensor', 'true', 'nao confere a chave dos dedos'),
]


def montar(context):
    a = valores(context, ARGS)
    extra = ['--velocidade', a['velocidade']]
    for nome in ('fechamento', 'descer'):
        if a[nome]:
            extra += [f'--{nome}', a[nome]]
    extra += [f'--{n.replace("_", "-")}' for n in ('so_planejar', 'sem_sensor') if a[n] in SIM]
    return [no('pegar_cubo', *extra)]


def generate_launch_description():
    return LaunchDescription(argumentos(ARGS) + [OpaqueFunction(function=montar)])
