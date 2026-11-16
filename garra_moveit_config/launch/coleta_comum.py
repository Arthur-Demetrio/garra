"""Funcoes comuns dos launchers da coleta (nao e um launch; importado pelos *.launch.py daqui)."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM = {'true', '1', 'sim'}


def pasta_scripts():
    """scripts/ do repo: GARRA_SCRIPTS ou <workspace>/scripts, ao lado de install/."""
    pacote = get_package_share_directory('garra_moveit_config')
    # install/garra_moveit_config/share/garra_moveit_config -> raiz do workspace
    raiz = os.path.abspath(os.path.join(pacote, '..', '..', '..', '..'))
    return os.environ.get('GARRA_SCRIPTS', os.path.join(raiz, 'scripts'))


def argumentos(padroes):
    """DeclareLaunchArgument para cada (nome, padrao, descricao)."""
    return [DeclareLaunchArgument(n, default_value=v, description=d) for n, v, d in padroes]


def valores(context, padroes):
    return {n: LaunchConfiguration(n).perform(context) for n, _, _ in padroes}


def script(nome, *args):
    """Roda scripts/<nome> com o python3 do sistema (saida direto no terminal)."""
    return ExecuteProcess(
        cmd=['python3', '-u', os.path.join(pasta_scripts(), nome), *args],
        name=nome.removesuffix('.py'), output='screen', emulate_tty=True,
        sigterm_timeout='5', sigkill_timeout='5')


def tf_camera():
    """static_transform_publisher robo_base -> camera_optical_frame com a pose do camera_cubo.py."""
    sys.path.insert(0, pasta_scripts())
    import camera_cubo
    from scipy.spatial.transform import Rotation
    R, t, origem = camera_cubo.pose_camera()
    qx, qy, qz, qw = Rotation.from_matrix(R).as_quat()
    print(f'[tf_camera] pose da camera ({origem}): t={[round(float(v), 4) for v in t]} '
          f'q={[round(float(v), 6) for v in (qx, qy, qz, qw)]}')
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_camera',
        arguments=['--x', str(t[0]), '--y', str(t[1]), '--z', str(t[2]),
                   '--qx', str(qx), '--qy', str(qy), '--qz', str(qz), '--qw', str(qw),
                   '--frame-id', 'robo_base', '--child-frame-id', 'camera_optical_frame'],
        output='screen')
