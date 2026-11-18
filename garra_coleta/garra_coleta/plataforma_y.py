#!/usr/bin/env python3
"""Leva a plataforma do eixo Y ate um y pedido, com a posicao medida pela camera (malha fechada).

O servo da plataforma e de rotacao continua: nao sabe onde esta. Quem sabe e a camera_cubo, que acha
o AprilTag preso na plataforma e publica a posicao dele em /garra/plataforma_y/posicao (Point,
robo_base). Este no compara com o alvo e manda andar/parar em /garra/plataforma_y (Int8: -1 esquerda,
0 para, +1 direita), que o driver da garra repassa ao firmware ("Y E|P|D").

Topicos:
  /garra/plataforma_y/alvo       std_msgs/Float64  y (m, robo_base) desejado para o PLANO DA GARRA
  /garra/plataforma_y/resultado  std_msgs/String   "ok y=0.0512" ou "erro: <motivo>"

Seguranca: para se a camera perder o tag por --sem-camera-s, se passar --tempo-max-s, ou se o y
pedido estiver fora de [--y-min, --y-max]. O driver tambem para se este no morrer (300 ms sem
mensagem) e o firmware para em 500 ms sem comando.

Perto do alvo (--zona-lenta) anda em pulsos curtos com pausas, para a camera alcancar e nao passar
do ponto. Chegou quando o erro fica abaixo de --tolerancia com a plataforma parada.

Uso (garra_real.launch.py e camera_cubo --tag-plataforma N rodando):
  ros2 run garra_coleta plataforma_y
  ros2 topic pub --once /garra/plataforma_y/alvo std_msgs/msg/Float64 "{data: 0.05}"

Teste do sentido: ros2 run garra_coleta plataforma_y --testar-sentido  (anda 0,5 s para a DIREITA
e mostra quanto o y mudou; se o y aumentou, use --direita-aumenta-y).
"""

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Int8, String

TOPICO_COMANDO = '/garra/plataforma_y'
TOPICO_POSICAO = '/garra/plataforma_y/posicao'
TOPICO_ALVO = '/garra/plataforma_y/alvo'
TOPICO_RESULTADO = '/garra/plataforma_y/resultado'
PERIODO_S = 0.05      # 20 Hz: bem abaixo dos 300 ms do driver


class PlataformaY:
    def __init__(self, no, args):
        self.no, self.a = no, args
        self.pub = no.create_publisher(Int8, TOPICO_COMANDO, 1)
        self.pub_res = no.create_publisher(String, TOPICO_RESULTADO, 10)
        no.create_subscription(Point, TOPICO_POSICAO, self._posicao, 10)
        no.create_subscription(Float64, TOPICO_ALVO, self._alvo, 10)
        self.y = None             # y do PLANO DA GARRA (tag + deslocamento)
        self.visto_em = 0.0
        self.alvo = None
        self.inicio = 0.0
        self.parado_desde = None  # para conferir o erro so com a plataforma assentada
        self.pulso_ate = 0.0      # fase de pulso: anda ate aqui, depois pausa ate pausa_ate
        self.pausa_ate = 0.0
        self.comando = 0
        no.create_timer(PERIODO_S, self._passo)

    def _posicao(self, p):
        self.y = p.y + self.a.deslocamento
        self.visto_em = time.monotonic()

    def _alvo(self, msg):
        y = msg.data
        if not self.a.y_min <= y <= self.a.y_max:
            self._fim(f'erro: alvo y={y:.3f} fora de [{self.a.y_min}, {self.a.y_max}]')
            return
        self.alvo, self.inicio, self.parado_desde = y, time.monotonic(), None
        self.no.get_logger().info(f'Alvo y={y:.3f} m')

    def _fim(self, texto):
        self.alvo = None
        self._mandar(0)
        self.pub_res.publish(String(data=texto))
        (self.no.get_logger().info if texto.startswith('ok') else self.no.get_logger().error)(texto)

    def _mandar(self, valor):
        self.comando = valor
        self.pub.publish(Int8(data=valor))

    def _sentido(self, erro):
        """Comando que diminui o erro (erro = alvo - y)."""
        direita = 1 if self.a.direita_aumenta_y else -1
        return direita if erro > 0 else -direita

    def _passo(self):
        agora = time.monotonic()
        if self.alvo is None:
            if self.comando != 0:
                self._mandar(0)
            return
        if self.y is None or agora - self.visto_em > self.a.sem_camera_s:
            self._fim('erro: camera nao ve o tag da plataforma')
            return
        if agora - self.inicio > self.a.tempo_max_s:
            self._fim(f'erro: tempo esgotado (y={self.y:.3f}, alvo {self.alvo:.3f})')
            return

        erro = self.alvo - self.y
        if abs(erro) <= self.a.tolerancia:
            self._mandar(0)
            if self.parado_desde is None:
                self.parado_desde = agora
            elif agora - self.parado_desde >= self.a.assentar_s:
                self._fim(f'ok y={self.y:.4f}')
            return
        self.parado_desde = None

        if abs(erro) > self.a.zona_lenta:
            self._mandar(self._sentido(erro))
            return
        # Perto: pulso curto, pausa para a camera medir, de novo.
        if agora < self.pulso_ate:
            self._mandar(self._sentido(erro))
        elif agora < self.pausa_ate:
            self._mandar(0)
        else:
            self.pulso_ate = agora + self.a.pulso_s
            self.pausa_ate = self.pulso_ate + self.a.pausa_s
            self._mandar(self._sentido(erro))


def testar_sentido(no, args):
    """Anda 0,5 s para a direita e mostra a variacao do y do tag."""
    estado = {'y': None}
    no.create_subscription(Point, TOPICO_POSICAO, lambda p: estado.update(y=p.y), 10)
    pub = no.create_publisher(Int8, TOPICO_COMANDO, 1)

    def esperar(seg, valor=None):
        fim = time.monotonic() + seg
        while time.monotonic() < fim:
            if valor is not None:
                pub.publish(Int8(data=valor))
            rclpy.spin_once(no, timeout_sec=PERIODO_S)

    esperar(1.0)
    y0 = estado['y']
    if y0 is None:
        print(f'Sem posicao em {TOPICO_POSICAO}: a camera_cubo esta rodando com --tag-plataforma?')
        return 1
    esperar(0.5, 1)
    esperar(1.0, 0)
    dy = estado['y'] - y0
    print(f'Direita por 0,5 s: y {y0:.4f} -> {estado["y"]:.4f} (dy = {dy * 1000:+.1f} mm, '
          f'~{abs(dy) / 0.5 * 1000:.0f} mm/s)')
    print('Use --direita-aumenta-y' if dy > 0 else 'Padrao certo (direita diminui o y)')
    return 0


def main():
    from rclpy.utilities import remove_ros_args
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--direita-aumenta-y', action='store_true',
                    help='o comando "direita" (1600 us) aumenta o y de robo_base (padrao: diminui)')
    ap.add_argument('--deslocamento', type=float, default=0.0,
                    help='y do plano da garra menos o y do tag (m), medido uma vez')
    ap.add_argument('--tolerancia', type=float, default=0.005, help='erro aceito (m)')
    ap.add_argument('--zona-lenta', type=float, default=0.03, help='abaixo desse erro anda em pulsos (m)')
    ap.add_argument('--pulso-s', type=float, default=0.10)
    ap.add_argument('--pausa-s', type=float, default=0.25)
    ap.add_argument('--assentar-s', type=float, default=0.3, help='tempo parado dentro da tolerancia')
    ap.add_argument('--sem-camera-s', type=float, default=0.3)
    ap.add_argument('--tempo-max-s', type=float, default=15.0)
    ap.add_argument('--y-min', type=float, default=-0.25, help='limite do curso (m, robo_base)')
    ap.add_argument('--y-max', type=float, default=0.25)
    ap.add_argument('--testar-sentido', action='store_true')
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    no = rclpy.create_node('plataforma_y')
    try:
        if args.testar_sentido:
            return testar_sentido(no, args)
        PlataformaY(no, args)
        print(f'Esperando alvos em {TOPICO_ALVO} (std_msgs/Float64, y em m). Ctrl+C para sair.')
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        # Sem mensagens, o driver para a plataforma em 300 ms.
        no.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
