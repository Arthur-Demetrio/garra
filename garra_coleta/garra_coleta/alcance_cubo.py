#!/usr/bin/env python3
"""Decide se a garra alcanca o cubo visto pela camera; se nao alcanca, manda a base se mexer.

Recebe a posicao do cubo (geometry_msgs/Point em /cubo_detectado, frame robo_base: X frente,
Y esquerda, Z cima, em metros) e confere se ela esta na JANELA DE COLETA medida na garra real:
  - X entre X_MIN e X_MAX (cubo no chao; 0.40-0.49 com a calibracao de 2026-11-16);
  - Y a ate Y_TOLERANCIA do plano da garra (o braco e planar, Y_PLANO).

  dentro da janela -> publica o Point em /pegar_cubo (o pegar_cubo.py pega o cubo);
  fora da janela   -> publica geometry_msgs/Pose2D em /robo/mover com o deslocamento RELATIVO
                      que a base deve fazer (frame robo_base atual) para o cubo cair no centro
                      da janela (o ajuste_base.py executa). Depois de mandar a base, ignora a
                      camera ate chegar /robo/mover/resultado (no maximo --espera s) e mais
                      --assentar s, para medir de novo com o robo ja parado.

So age DURANTE UMA COLETA: o executor_prova.py do robo (--garra), ja parado na mesa, publica
std_msgs/String em /coleta/pedido; este script ajusta a base e chama a garra ate pegar, e responde
em /coleta/resultado: 'ok' ou 'falhou: motivo' (desiste apos --max-ajustes ajustes da base ou
--max-pegadas tentativas da garra). Fora de uma coleta, as deteccoes sao ignoradas: o robo NAO se
mexe sozinho enquanto navega. --sempre-ativo desliga essa trava (so para testes).

Pose2D em /robo/mover:
  padrao (base holonomica): x, y = quanto andar para frente/esquerda; theta = 0.
  --girar (base diferencial): primeiro gira theta (rad, anti-horario) no lugar, depois anda x
                              para frente; y = 0.

Se a garra for recalibrada ou mudar de lugar, atualize X_MIN, X_MAX e Y_PLANO.

Fluxo:
  terminal 1: ros2 launch ... garra_real.launch.py
  terminal 2: ros2 run garra_coleta pegar_cubo                              (escuta /pegar_cubo)
  terminal 3: ros2 run garra_coleta alcance_cubo
  terminal 3b: ros2 run garra_coleta ajuste_base                         (move a base do robo)
  terminal 4: ros2 run garra_coleta camera_cubo --topico /cubo_detectado

Com o no cube_vision do repo niviton/-WORK (ros2_ws/src/cube_vision) no lugar do camera_cubo.py:
  ros2 run garra_coleta alcance_cubo --vision
  Le vision_msgs/Detection3DArray em /vision/cubes (frame camera_optical_frame) e converte para
  robo_base pelo TF; e preciso publicar robo_base -> camera_optical_frame (static_transform_publisher)
  e ter o pacote vision_msgs instalado (sudo apt install ros-$ROS_DISTRO-vision-msgs).
  Com varios cubos, usa o mais perto da base (menor X); --id escolhe um ID (cubo_<id>).

Teste sem camera:
  ros2 topic pub --once /cubo_detectado geometry_msgs/msg/Point "{x: 0.60, y: 0.05, z: 0.021}"
  ros2 topic echo /robo/mover
"""

import argparse
import math
import sys
import signal
import time

from geometry_msgs.msg import Point, Pose2D
from std_msgs.msg import String
import rclpy
from rclpy.executors import ExternalShutdownException

TOPICO_CUBO = '/cubo_detectado'
TOPICO_GARRA = '/pegar_cubo'
TOPICO_BASE = '/robo/mover'
TOPICO_RESULTADO = '/robo/mover/resultado'
TOPICO_GARRA_RES = '/pegar_cubo/resultado'
TOPICO_COLETA = '/coleta/pedido'          # do executor_prova.py (--garra) do robo
TOPICO_COLETA_RES = '/coleta/resultado'

# Janela de coleta no chao (robo_base, m). Calibracao de 2026-11-16: 0,40-~0,49 (pegada real em 0,465). Alem de -90 graus quase bateu no robo (2026-11-17): nao usar.
# 2026-11-21 (robo no chao): com a CORRECAO_X do pegar_cubo.py, a garra real so alcanca a partir de
# ~13,5 cm da borda (0.41): X_MIN 0.415 com folga. X_ALVO fixo em 17 cm (0.445), o ponto conferido na
# pegada e na entrega (antes era o meio da janela).
X_MIN = 0.415
X_MAX = 0.49
Y_PLANO = -0.029      # plano da garra (Y do TCP)
Y_TOLERANCIA = 0.015  # mesmo valor do aviso do camera_cubo.py
X_ALVO = 0.445


def alcanca(x, y):
    return X_MIN <= x <= X_MAX and abs(y - Y_PLANO) <= Y_TOLERANCIA


def deslocamento_holonomico(x, y):
    """Quanto a base anda (frente, esquerda) para o cubo ir para (X_ALVO, Y_PLANO)."""
    return Pose2D(x=x - X_ALVO, y=y - Y_PLANO, theta=0.0)


def deslocamento_girando(x, y):
    """Gira theta no lugar e depois anda x para frente, deixando o cubo em (X_ALVO, Y_PLANO).

    Girar theta leva o cubo de (r, phi) para (r, phi - theta). Para o Y final ser Y_PLANO:
    r * sin(phi - theta) = Y_PLANO. Depois, andar para frente so muda o X.
    """
    r = math.hypot(x, y)
    if r <= abs(Y_PLANO):
        return None   # cubo em cima do centro da base: nao ha giro que resolva
    phi = math.atan2(y, x)
    theta = phi - math.asin(Y_PLANO / r)
    x_girado = r * math.cos(phi - theta)
    return Pose2D(x=x_girado - X_ALVO, y=0.0, theta=theta)


def girar(q, v):
    """Aplica o quaternion q (x, y, z, w) ao vetor v."""
    qx, qy, qz, qw = q
    tx = 2 * (qy * v[2] - qz * v[1])
    ty = 2 * (qz * v[0] - qx * v[2])
    tz = 2 * (qx * v[1] - qy * v[0])
    return (v[0] + qw * tx + qy * tz - qz * ty,
            v[1] + qw * ty + qz * tx - qx * tz,
            v[2] + qw * tz + qx * ty - qy * tx)


def ouvir_vision(no, recebido, id_alvo):
    """Assina /vision/cubes (cube_vision do -WORK) e entrega Points em robo_base."""
    from rclpy.time import Time
    import tf2_ros
    from vision_msgs.msg import Detection3DArray

    tf_buffer = tf2_ros.Buffer()
    no._tf_listener = tf2_ros.TransformListener(tf_buffer, no)

    def detectado(msg):
        cubos = []
        for d in msg.detections:
            nome = d.results[0].hypothesis.class_id if d.results else ''
            if id_alvo is not None and nome != f'cubo_{id_alvo}':
                continue
            frame = d.header.frame_id or msg.header.frame_id
            try:
                t = tf_buffer.lookup_transform('robo_base', frame, Time()).transform
            except tf2_ros.TransformException as e:
                print(f'Sem TF robo_base <- {frame}: {e}', flush=True)
                return
            p = d.bbox.center.position
            r = t.rotation
            x, y, z = girar((r.x, r.y, r.z, r.w), (p.x, p.y, p.z))
            cubos.append(Point(x=x + t.translation.x, y=y + t.translation.y, z=z + t.translation.z))
        if cubos:
            recebido(min(cubos, key=lambda c: c.x))

    no.create_subscription(Detection3DArray, '/vision/cubes', detectado, 10)



def argumentos_sem_ros():
    """sys.argv sem os --ros-args que o ros2 launch/run acrescentam."""
    try:
        from rclpy.utilities import remove_ros_args
        return remove_ros_args(sys.argv)[1:]
    except ImportError:
        return sys.argv[1:]

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--girar', action='store_true',
                    help='base diferencial: gira e anda para frente (sem andar de lado)')
    ap.add_argument('--assentar', type=float, default=1.0,
                    help='s esperando depois do resultado da base, antes de medir de novo (padrao 1)')
    ap.add_argument('--espera', type=float, default=20.0,
                    help='s maximo esperando o resultado da base (padrao 20)')
    ap.add_argument('--espera-garra', type=float, default=90.0,
                    help='s maximo esperando o resultado do pegar_cubo.py (padrao 90)')
    ap.add_argument('--max-ajustes', type=int, default=4,
                    help='ajustes da base por coleta antes de desistir (padrao 4)')
    ap.add_argument('--max-pegadas', type=int, default=2,
                    help='tentativas da garra por coleta antes de desistir (padrao 2)')
    ap.add_argument('--sempre-ativo', action='store_true',
                    help='age a qualquer momento, sem esperar /coleta/pedido (so para testes!)')
    ap.add_argument('--vision', action='store_true',
                    help='le /vision/cubes do cube_vision (-WORK) em vez de /cubo_detectado')
    ap.add_argument('--id', type=int, default=None, help='com --vision: so o cubo com este ID')
    ap.add_argument('--z-medido', action='store_true',
                    help='com --vision: manda o z medido; sem ele, z = 0.021 (cubo no chao)')
    args = ap.parse_args(argumentos_sem_ros())

    rclpy.init()
    no = rclpy.create_node('alcance_cubo')
    pub_garra = no.create_publisher(Point, TOPICO_GARRA, 10)
    pub_base = no.create_publisher(Pose2D, TOPICO_BASE, 10)
    pub_coleta = no.create_publisher(String, TOPICO_COLETA_RES, 10)
    # ativo: so age durante uma coleta (pedida em /coleta/pedido) ou sempre, com --sempre-ativo.
    # espera: 'base' ou 'garra' enquanto aguarda o resultado delas (a camera e ignorada).
    st = dict(ativo=args.sempre_ativo, espera=None, ate=0.0, ajustes=0, pegadas=0, pedido='')

    def terminar(resultado):
        print(f'Coleta: {resultado}', flush=True)
        if not args.sempre_ativo:
            pub_coleta.publish(String(data=resultado))
            st['ativo'] = False

    def pedido_coleta(m):
        print(f'Pedido de coleta: {m.data}', flush=True)
        st.update(ativo=True, espera=None, ate=0.0, ajustes=0, pegadas=0, pedido=m.data)

    def resultado_base(m):
        print(f'Base: {m.data}', flush=True)
        if st['espera'] == 'base':
            st['espera'], st['ate'] = None, time.monotonic() + args.assentar

    def resultado_garra(m):
        print(f'Garra: {m.data}', flush=True)
        if st['espera'] != 'garra':
            return
        st['espera'], st['ate'] = None, time.monotonic() + args.assentar
        if m.data == 'ok':
            terminar('ok')
        elif st['pegadas'] >= args.max_pegadas:
            terminar(f'falhou: garra ({m.data}) em {st["pegadas"]} tentativa(s)')

    no.create_subscription(String, TOPICO_COLETA, pedido_coleta, 10)
    no.create_subscription(String, TOPICO_RESULTADO, resultado_base, 10)
    no.create_subscription(String, TOPICO_GARRA_RES, resultado_garra, 10)

    def recebido(p):
        agora = time.monotonic()
        if st['espera'] and agora > st['ate']:
            print(f'Sem resultado da {st["espera"]} a tempo; medindo de novo.', flush=True)
            st['espera'] = None
        if not st['ativo'] or st['espera'] or agora < st['ate']:
            return
        if alcanca(p.x, p.y):
            print(f'Cubo em x={p.x:.3f} y={p.y:.3f}: ao alcance -> {TOPICO_GARRA}', flush=True)
            if not pub_garra.get_subscription_count():
                print(f'  AVISO: ninguem escutando {TOPICO_GARRA} (o pegar_cubo.py esta rodando?)')
            pub_garra.publish(p)
            st['pegadas'] += 1
            st['espera'], st['ate'] = 'garra', agora + args.espera_garra
            return
        if st['ajustes'] >= args.max_ajustes:
            terminar(f'falhou: cubo fora do alcance apos {st["ajustes"]} ajuste(s) da base')
            return
        mover = deslocamento_girando(p.x, p.y) if args.girar else deslocamento_holonomico(p.x, p.y)
        if mover is None:
            print(f'Cubo em x={p.x:.3f} y={p.y:.3f}: em cima da base, nao da para alinhar.', flush=True)
            return
        print(f'Cubo em x={p.x:.3f} y={p.y:.3f}: fora do alcance -> {TOPICO_BASE} '
              f'x={mover.x:+.3f} m y={mover.y:+.3f} m theta={math.degrees(mover.theta):+.1f} graus',
              flush=True)
        if not pub_base.get_subscription_count():
            print(f'  AVISO: ninguem escutando {TOPICO_BASE} (o ajuste_base.py esta rodando?)')
        pub_base.publish(mover)
        st['ajustes'] += 1
        st['espera'], st['ate'] = 'base', agora + args.espera

    if args.vision:
        def recebido_vision(p):
            if not args.z_medido:
                p.z = 0.021   # o z medido pela camera sai ~3 cm alto
            recebido(p)
        ouvir_vision(no, recebido_vision, args.id)
    else:
        no.create_subscription(Point, TOPICO_CUBO, recebido, 10)
    print(f'Janela da garra: X {X_MIN:.2f}-{X_MAX:.2f} m, Y {Y_PLANO:+.3f} +- {Y_TOLERANCIA:.3f} m. '
          f'Esperando {"/vision/cubes" if args.vision else TOPICO_CUBO}...', flush=True)
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # 2o Ctrl+C (ros2 launch) nao interrompe o encerramento
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()


