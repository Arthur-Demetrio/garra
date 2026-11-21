"""alcance_cubo: decide entre pegar o cubo (/pegar_cubo) e mover a base (/robo/mover).

So age durante uma coleta (/coleta/pedido), a menos que sempre_ativo:=true (SO PARA TESTES).
Com vision:=true le o cube_vision (/vision/cubes) e precisa do tf_camera.launch.py.

    ros2 launch garra_coleta alcance_cubo.launch.py
    ros2 launch garra_coleta alcance_cubo.launch.py vision:=true
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import OpaqueFunction

sys.path.insert(0, os.path.dirname(__file__))
from coleta_comum import SIM, argumentos, no, valores  # noqa: E402

ARGS = [
    ('vision', 'false', 'le /vision/cubes do cube_vision em vez de /cubo_detectado'),
    ('id', '', 'com vision: so o cubo com este ID do AprilTag (vazio = o mais perto)'),
    ('girar', 'false', 'base sem andar de lado: gira e anda para frente'),
    ('sempre_ativo', 'false', 'age sem /coleta/pedido (SO PARA TESTES: move a base a qualquer hora)'),
    ('max_ajustes', '4', 'ajustes da base por coleta antes de desistir'),
    ('max_pegadas', '2', 'tentativas da garra por coleta antes de desistir'),
    ('plataforma', 'false', 'alinha o Y pela plataforma Y (precisa do plataforma_y.py) em vez da base'),
]


def montar(context):
    a = valores(context, ARGS)
    extra = ['--max-ajustes', a['max_ajustes'], '--max-pegadas', a['max_pegadas']]
    extra += [f'--{n.replace("_", "-")}' for n in ('vision', 'girar', 'sempre_ativo', 'plataforma')
              if a[n] in SIM]
    if a['id']:
        extra += ['--id', a['id']]
    return [no('alcance_cubo', *extra)]


def generate_launch_description():
    return LaunchDescription(argumentos(ARGS) + [OpaqueFunction(function=montar)])
