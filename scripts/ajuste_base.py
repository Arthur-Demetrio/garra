#!/usr/bin/env python3
"""Faz o ajuste curto da base pedido pelo alcance_cubo.py, pelo canal 'especial' do arbitro do robo.

Escuta geometry_msgs/Pose2D em /robo/mover: deslocamento RELATIVO no frame do robo (base_link) no
momento do pedido. Primeiro gira theta (rad, anti-horario) no lugar, depois anda x (frente) e
y (esquerda) no frame ja girado. Com a base mecanum, o alcance_cubo.py manda theta = 0.

Integracao com o Robo-WORK (github.com/OttavioSilva/Robo-WORK, arbitro_nav_principal.py):
  1. pede a autoridade 'especial' em /navegacao/especial (std_srvs/SetBool, data=true);
  2. mantem o heartbeat em /navegacao/heartbeat (TwistStamped, frame_id 'especial') a 20 Hz;
  3. publica a velocidade em /cmd_vel_nav_especial (passa pelo suavizador e pelo collision monitor);
  4. ao terminar (ou falhar), zera a velocidade e devolve a autoridade ao nav2 (data=false).
Nunca publica em /cmd_vel direto. So use com o alinhador ocioso (depois do encosto na mesa): ele
tambem usa o canal 'especial'.

A posicao vem de /odom. A odometria lateral do robo superestima (medido por eles em 06/11), entao
o ajuste e aproximado: o alcance_cubo.py confere de novo pela camera e pede outro se precisar.

Resultado em /robo/mover/resultado (std_msgs/String): 'ok ...' ou 'falhou: motivo'.

Limites (padrao): 0,05 m/s, 0,30 rad/s, ate 15 cm por eixo e 30 graus por pedido, 15 s por pedido.

Uso:
  python3 scripts/ajuste_base.py
  python3 scripts/ajuste_base.py --sem-arbitro --topico-cmd /cmd_vel   # teste em simulacao
Teste (o robo ANDA 5 cm para a frente):
  ros2 topic pub --once /robo/mover geometry_msgs/msg/Pose2D "{x: 0.05, y: 0.0, theta: 0.0}"
"""

import argparse
import math
import signal
import time

from geometry_msgs.msg import Pose2D, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from std_srvs.srv import SetBool

TOPICO_PEDIDO = '/robo/mover'
TOPICO_RESULTADO = '/robo/mover/resultado'
SERVICO_DONO = '/navegacao/especial'
TOPICO_HEARTBEAT = '/navegacao/heartbeat'
DONO = 'especial'
PERIODO = 0.05          # s (heartbeat e controle a 20 Hz; o arbitro corta com 0,5 s sem heartbeat)
ODOM_VELHA = 0.5        # s sem /odom -> aborta
TOL_XY = 0.01           # m
TOL_THETA = math.radians(2.0)
GANHO = 1.5             # 1/s: velocidade = GANHO * erro (saturada)


def yaw_de(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angulo(a):
    return math.atan2(math.sin(a), math.cos(a))


def saturar(v, lim):
    return max(-lim, min(lim, v))


class AjusteBase:
    def __init__(self, args):
        self.args = args
        self.no = rclpy.create_node('ajuste_base')
        self.pub_cmd = self.no.create_publisher(TwistStamped, args.topico_cmd, 10)
        self.pub_hb = self.no.create_publisher(TwistStamped, TOPICO_HEARTBEAT, 1)
        self.pub_res = self.no.create_publisher(String, TOPICO_RESULTADO, 10)
        self.cli = self.no.create_client(SetBool, SERVICO_DONO)
        self.no.create_subscription(Odometry, '/odom', self.ao_odom, 20)
        self.no.create_subscription(Pose2D, TOPICO_PEDIDO, self.ao_pedido, 10)
        self.odom = None            # (x, y, yaw, instante)
        self.pedido = None

    def ao_odom(self, m):
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_de(p.orientation), time.monotonic())

    def ao_pedido(self, m):
        if self.pedido is not None:
            print('Pedido ignorado: ainda executando o anterior.', flush=True)
            return
        self.pedido = m

    # --- arbitro ---------------------------------------------------------------------------
    def trocar_dono(self, especial):
        if self.args.sem_arbitro:
            return True
        if not self.cli.wait_for_service(timeout_sec=1.0):
            print(f'Servico {SERVICO_DONO} indisponivel (o arbitro do robo esta rodando?).', flush=True)
            return False
        fut = self.cli.call_async(SetBool.Request(data=especial))
        rclpy.spin_until_future_complete(self.no, fut, timeout_sec=2.0)
        r = fut.result()
        if r is None or not r.success:
            print(f'O arbitro recusou a troca para {"especial" if especial else "nav2"}.', flush=True)
            return False
        return True

    def enviar(self, vx=0.0, vy=0.0, wz=0.0):
        agora = self.no.get_clock().now().to_msg()
        hb = TwistStamped()
        hb.header.stamp, hb.header.frame_id = agora, DONO
        self.pub_hb.publish(hb)
        m = TwistStamped()
        m.header.stamp, m.header.frame_id = agora, 'base_link'
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = vx, vy, wz
        self.pub_cmd.publish(m)

    # --- execucao --------------------------------------------------------------------------
    def validar(self, p):
        a = self.args
        if abs(p.x) > a.max_desloc or abs(p.y) > a.max_desloc:
            return f'deslocamento ({p.x:+.3f}, {p.y:+.3f}) m acima do limite de {a.max_desloc:.2f} m'
        if abs(p.theta) > math.radians(a.max_giro):
            return f'giro de {math.degrees(p.theta):+.1f} graus acima do limite de {a.max_giro:.0f}'
        if self.odom is None or time.monotonic() - self.odom[3] > ODOM_VELHA:
            return 'sem /odom recente'
        return None

    def executar(self, p):
        x0, y0, yaw0, _ = self.odom
        # Alvo em odom: gira theta e depois anda (x, y) no frame ja girado.
        yaw_alvo = yaw0 + p.theta
        c, s = math.cos(yaw_alvo), math.sin(yaw_alvo)
        alvo = (x0 + c * p.x - s * p.y, y0 + s * p.x + c * p.y)
        fim = time.monotonic() + self.args.timeout
        girando = abs(p.theta) > TOL_THETA
        while rclpy.ok():
            rclpy.spin_once(self.no, timeout_sec=PERIODO)
            if time.monotonic() > fim:
                return 'falhou: tempo esgotado'
            x, y, yaw, t = self.odom
            if time.monotonic() - t > ODOM_VELHA:
                return 'falhou: /odom parou'
            e_yaw = angulo(yaw_alvo - yaw)
            ex, ey = alvo[0] - x, alvo[1] - y
            # erro de posicao no frame do robo
            ebx = math.cos(yaw) * ex + math.sin(yaw) * ey
            eby = -math.sin(yaw) * ex + math.cos(yaw) * ey
            if girando:
                if abs(e_yaw) <= TOL_THETA:
                    girando = False
                    continue
                self.enviar(wz=saturar(GANHO * e_yaw, self.args.vel_giro))
                continue
            if math.hypot(ebx, eby) <= TOL_XY and abs(e_yaw) <= 2 * TOL_THETA:
                return (f'ok: erro final ({ebx * 100:+.1f}, {eby * 100:+.1f}) cm, '
                        f'{math.degrees(e_yaw):+.1f} graus (pela odom)')
            vx, vy = GANHO * ebx, GANHO * eby
            k = min(1.0, self.args.vel / max(math.hypot(vx, vy), 1e-9))
            self.enviar(vx * k, vy * k, saturar(GANHO * e_yaw, self.args.vel_giro))
        return 'falhou: ROS encerrado'

    def atender(self, p):
        print(f'Pedido: x={p.x:+.3f} m y={p.y:+.3f} m theta={math.degrees(p.theta):+.1f} graus', flush=True)
        erro = self.validar(p)
        if erro:
            return 'falhou: ' + erro
        if not self.trocar_dono(True):
            return 'falhou: sem autoridade especial no arbitro'
        try:
            return self.executar(p)
        finally:
            # Ctrl+C no meio do movimento: o robo PRECISA parar e devolver a autoridade. Se o ROS ja
            # caiu, o arbitro zera sozinho em 0,5 s sem heartbeat.
            anterior = signal.signal(signal.SIGINT, signal.SIG_IGN)
            try:
                for _ in range(3):        # zera e garante que o suavizador recebeu
                    self.enviar()
                    time.sleep(PERIODO)
                if not self.trocar_dono(False):
                    print('ATENCAO: nao devolveu a autoridade ao nav2.', flush=True)
            except Exception as e:
                print(f'ATENCAO: falha ao parar ({e}); o arbitro zera sem heartbeat em 0,5 s.', flush=True)
            finally:
                signal.signal(signal.SIGINT, anterior)

    def rodar(self):
        print(f'Esperando pedidos em {TOPICO_PEDIDO} (comandos em {self.args.topico_cmd}'
              f'{", SEM arbitro" if self.args.sem_arbitro else ""})...', flush=True)
        while rclpy.ok():
            rclpy.spin_once(self.no, timeout_sec=0.1)
            if self.pedido is None:
                continue
            res = self.atender(self.pedido)
            print(res, flush=True)
            self.pub_res.publish(String(data=res))
            self.pedido = None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--vel', type=float, default=0.05, help='m/s maximo (padrao 0.05)')
    ap.add_argument('--vel-giro', type=float, default=0.30, help='rad/s maximo (padrao 0.30)')
    ap.add_argument('--max-desloc', type=float, default=0.15, help='m maximo por eixo (padrao 0.15)')
    ap.add_argument('--max-giro', type=float, default=30.0, help='graus maximo por pedido (padrao 30)')
    ap.add_argument('--timeout', type=float, default=15.0, help='s maximo por pedido (padrao 15)')
    ap.add_argument('--topico-cmd', default='/cmd_vel_nav_especial',
                    help='onde publicar a velocidade (padrao /cmd_vel_nav_especial)')
    ap.add_argument('--sem-arbitro', action='store_true',
                    help='nao chama /navegacao/especial (so para simulacao)')
    args = ap.parse_args()

    rclpy.init()
    a = AjusteBase(args)
    try:
        a.rodar()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # 2o Ctrl+C (ros2 launch) nao interrompe o encerramento
        a.no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
