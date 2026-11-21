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
  ros2 run garra_coleta plataforma_y --seguir-cubo   # segue o y do cubo visto (/cubo_visto)
  ros2 run garra_coleta plataforma_y --centralizar -0.029   # vai ao plano da garra do alcance_cubo, 1 vez

Perto das bordas (a menos de --zona-borda de --y-min/--y-max) anda devagar. Para encostar numa borda
com o teste da chave (homing, feito pelo firmware):
  ros2 topic pub --once /garra/plataforma_y/home std_msgs/msg/String "{data: E}"   # ou D
  ros2 topic echo /garra/plataforma_y/home/resultado                               # "E OK"

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
TOPICO_CUBO = '/cubo_visto'   # camera_cubo: cubo alvo a cada quadro (modo --seguir-cubo)
SEGURAR_MM = 6.0         # depois de chegar, corrige sozinho se o plano sair mais que isso do alvo
TRAVA_S = 1.5            # andando sem o y mudar TRAVA_MM neste tempo = travada (para)
TRAVA_MM = 3.0
ESTAVEL_INICIAL_S = 1.0  # --centralizar: tag visto sem falhas por este tempo antes de comecar
TENTATIVAS_INICIAL = 3   # --centralizar: novas tentativas se a camera perder o tag no caminho
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
        self.cubo_em = 0.0
        self.saltos = 0
        self.cubo_hist = []       # (t, y) recentes do cubo, para so aceitar um alvo estavel
        self.centralizar = args.centralizar   # alvo inicial pendente; vira None quando comeca
        self.centralizar_alvo = args.centralizar
        self.centralizando = False            # movimento inicial em andamento
        self.tentativas = 0
        self.visto_desde = None               # inicio da sequencia continua de leituras do tag
        self.trava_ref = (0.0, None)          # (t, y) da ultima vez que o y andou TRAVA_MM
        self.segurar = None                   # ultimo alvo atingido: corrige se sair SEGURAR_MM
        self.segurando = False                # correcao silenciosa em andamento (nao publica resultado)
        self.alvo_atingido = None
        if args.seguir_cubo:
            no.create_subscription(Point, TOPICO_CUBO, self._cubo, 10)
            no.get_logger().info(f'Seguindo o y do cubo em {TOPICO_CUBO}')
        no.create_timer(PERIODO_S, self._passo)

    def _cubo(self, p):
        """Modo seguir: MEMORIZA o y do cubo e alinha com ele, sem precisar continuar vendo o cubo
        (a garra passa na frente dele e da camera). Leituras do cubo durante o movimento sao
        ignoradas; um alvo novo so e aceito com a plataforma parada e o cubo estavel por
        --estavel-s (o mesmo y, +-5 mm), e se ele estiver a mais de --histerese do plano da garra."""
        agora = time.monotonic()
        self.cubo_em = agora
        if self.alvo is not None:
            return                                   # alinhando: alvo ja memorizado
        self.cubo_hist = [(t, y) for t, y in self.cubo_hist if agora - t <= self.a.estavel_s]
        self.cubo_hist.append((agora, p.y))
        ys = [y for _, y in self.cubo_hist]
        if agora - self.cubo_hist[0][0] < self.a.estavel_s * 0.8 or max(ys) - min(ys) > 0.005:
            return                                   # cubo ainda nao estavel (ou leitura pulando)
        y = sum(ys) / len(ys)
        if self.y is None or abs(y - self.y) <= self.a.histerese:
            return                                   # ja alinhado
        alvo = min(max(y, self.a.y_min), self.a.y_max)
        self.alvo, self.inicio, self.parado_desde = alvo, agora, None
        self.saltos = 0
        self.no.get_logger().info(f'Cubo em y={y:.3f} (plano da garra em {self.y:.3f}): alinhando em {alvo:.3f}')

    def _posicao(self, p):
        agora = time.monotonic()
        y = p.y + self.a.deslocamento
        # A plataforma anda ~2 cm/s: um salto maior que --max-salto entre leituras seguidas e leitura
        # falsa (tag meio tampado pela garra). Descarta; se so vierem leituras assim por mais de
        # --sem-camera-s, o _passo para por "camera nao ve o tag".
        if (self.y is not None and agora - self.visto_em < self.a.sem_camera_s
                and abs(y - self.y) > self.a.max_salto):
            self.saltos += 1
            return
        if self.visto_desde is None or agora - self.visto_em > self.a.sem_camera_s:
            self.visto_desde = agora
        self.y, self.visto_em = y, agora
        # Posicao inicial so depois de ver o tag por ESTAVEL_INICIAL_S seguidos: logo que a camera abre,
        # alguns quadros atrasam e o movimento parava por "camera nao ve o tag" (2026-11-21).
        if self.centralizar is not None and agora - self.visto_desde >= ESTAVEL_INICIAL_S:
            alvo, self.centralizar = self.centralizar, None
            self.centralizando = True
            self.no.get_logger().info(f'Centralizando: tag visto, plano da garra em y={y:.3f}')
            self._alvo(Float64(data=alvo))

    def _alvo(self, msg):
        y = msg.data
        if not self.a.y_min <= y <= self.a.y_max:
            self._fim(f'erro: alvo y={y:.3f} fora de [{self.a.y_min}, {self.a.y_max}]')
            return
        agora = time.monotonic()
        self.segurando, self.alvo_atingido = False, y   # alvo novo (externo) cancela o segurar em curso
        # Posicao inicial: se o plano ja esta no alvo, ou o alvo e junto de uma borda (a menos de
        # --zona-borda) e o plano ja esta entre o alvo e essa borda, nao anda.
        if self.centralizando and self.y is not None:
            borda = self.a.y_max if y > 0 else self.a.y_min
            junto_da_borda = abs(borda - y) <= self.a.zona_borda
            alem = abs(borda - self.y) < abs(borda - y) and (self.y - y) * (borda - y) > 0
            if abs(y - self.y) <= self.a.tolerancia or (junto_da_borda and alem):
                self._fim(f'ok y={self.y:.4f} (ja na posicao inicial, sem andar)')
                return
        self.alvo, self.inicio, self.parado_desde = y, agora, None
        self.alvo_atingido = y
        self.trava_ref = (agora, self.y)
        self.no.get_logger().info(f'Alvo y={y:.3f} m')

    def _fim(self, texto):
        self.alvo = None
        self._mandar(0)
        if self.segurando:
            # Correcao do segurar: nao publica (quem pediu o alvo ja recebeu o "ok"); so registra.
            self.segurando = False
            (self.no.get_logger().info if texto.startswith('ok') else self.no.get_logger().warn)(
                f'Segurando posicao: {texto}')
            if not texto.startswith('ok'):
                self.segurar = None
            return
        self.segurar = self.alvo_atingido if texto.startswith('ok') else None
        if self.centralizando:
            if texto.startswith('erro: camera nao ve') and self.tentativas < TENTATIVAS_INICIAL:
                self.tentativas += 1
                self.centralizar, self.visto_desde = self.centralizar_alvo, None
                self.no.get_logger().warn(f'Posicao inicial: {texto}; tentando de novo '
                                          f'({self.tentativas}/{TENTATIVAS_INICIAL}) quando o tag voltar')
                return
            self.centralizando = False
        self.pub_res.publish(String(data=texto))
        (self.no.get_logger().info if texto.startswith('ok') else self.no.get_logger().error)(texto)

    def _mandar(self, valor):
        self.comando = valor
        self.pub.publish(Int8(data=valor))

    def _sentido(self, erro):
        """Comando que diminui o erro (erro = alvo - y)."""
        direita = 1 if self.a.direita_aumenta_y else -1
        return direita if erro > 0 else -direita

    def _velocidade(self, erro):
        """Sentido do _sentido; devagar (+-2, "Y L E/D" no firmware) a menos de --zona-borda do
        limite para onde esta indo, para nao chegar rapido na borda nem na chave."""
        cmd = self._sentido(erro)
        limite = self.a.y_max if erro > 0 else self.a.y_min
        if self.a.zona_borda > 0 and abs(limite - self.y) < self.a.zona_borda:
            return 2 * cmd
        return cmd

    def _passo(self):
        agora = time.monotonic()
        if self.alvo is None:
            if self.comando != 0:
                self._mandar(0)
            # Segurar: a plataforma nao tem freio e escorregou ~16 cm durante uma pegada (2026-11-21).
            # Com o tag visivel e o plano a mais de SEGURAR_MM do ultimo alvo, volta para ele.
            if (self.a.segurar_mm > 0 and self.segurar is not None and self.y is not None
                    and agora - self.visto_em <= self.a.sem_camera_s
                    and abs(self.segurar - self.y) > self.a.segurar_mm / 1000.0):
                self.no.get_logger().warn(f'Plataforma saiu do lugar (y={self.y:.4f}, alvo '
                                          f'{self.segurar:.4f}): voltando')
                self.segurando = True
                alvo = self.segurar
                self.alvo, self.inicio, self.parado_desde = alvo, agora, None
                self.alvo_atingido = alvo
                self.trava_ref = (agora, self.y)
            return
        if self.y is None or agora - self.visto_em > self.a.sem_camera_s:
            self._fim('erro: camera nao ve o tag da plataforma'
                      + (f' (ultimo y={self.y:.4f})' if self.y is not None else ''))
            return
        if agora - self.inicio > self.a.tempo_max_s:
            self._fim(f'erro: tempo esgotado (y={self.y:.3f}, alvo {self.alvo:.3f})')
            return

        # Travou: mandando andar e o y nao muda TRAVA_MM em TRAVA_S (fim de curso, cabo preso, camera
        # errando a posicao perto da borda). 2026-11-21: ficou 30 s forcando no limite da esquerda.
        ref_t, ref_y = self.trava_ref
        if ref_y is None or abs(self.y - ref_y) >= TRAVA_MM / 1000.0:
            self.trava_ref = (agora, self.y)
        elif agora - ref_t > TRAVA_S:
            self._fim(f'erro: travada (y={self.y:.4f} nao mudou em {TRAVA_S:.1f} s, alvo {self.alvo:.3f})')
            return

        erro = self.alvo - self.y
        if abs(erro) <= self.a.tolerancia:
            self.trava_ref = (agora, self.y)
            self._mandar(0)
            if self.parado_desde is None:
                self.parado_desde = agora
            elif agora - self.parado_desde >= self.a.assentar_s:
                self._fim(f'ok y={self.y:.4f}' + (f' ({self.saltos} leituras falsas descartadas)' if self.saltos else ''))
                self.cubo_hist = []
            return
        self.parado_desde = None

        if abs(erro) > self.a.zona_lenta:
            self._mandar(self._velocidade(erro))
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
    # 2026-11-20: -0.0073 (dedos -2,0 cm na trena, tag -1,27 cm), com a calibracao da camera do suporte.
    # 2026-11-21: camera recalibrada no chao (+ y corrigido em +1,06 cm com um cubo a -3 cm na trena): dedos
    # a ~-5 cm na trena com o tag a ~-2,1 cm -> -0.029; com a altura do tag corrigida para 0.174 no camera_cubo
    # (ajuste por 2 pontos na trena: -3 e +13 cm) -> -0.0346; na 1a coleta completa a garra fechou ~5 mm a
    # esquerda do cubo (2,5 mm da tolerancia + ~2,5 mm daqui) -> -0.0321; na 2a, com a plataforma a <1 mm
    # do cubo pela camera, ainda ~3 mm a esquerda -> -0.0291. Remedir se a camera for recalibrada.
    ap.add_argument('--deslocamento', type=float, default=-0.0291,
                    help='y do plano da garra menos o y do tag (m), medido uma vez')
    ap.add_argument('--centralizar', type=float, default=None, metavar='Y',
                    help='leva o plano da garra a este y (m) assim que a camera ver o tag, uma vez '
                         '(o coleta.launch.py usa -0.029, o Y_PLANO do alcance_cubo)')
    # 3 mm (era 5): a 5 mm a garra fechava em cima do cubo; um pulso da zona lenta anda ~2,6 mm.
    ap.add_argument('--tolerancia', type=float, default=0.003, help='erro aceito (m)')
    ap.add_argument('--zona-lenta', type=float, default=0.03, help='abaixo desse erro anda em pulsos (m)')
    ap.add_argument('--pulso-s', type=float, default=0.10)
    ap.add_argument('--pausa-s', type=float, default=0.25)
    ap.add_argument('--assentar-s', type=float, default=0.3, help='tempo parado dentro da tolerancia')
    ap.add_argument('--sem-camera-s', type=float, default=0.3)
    # 30 s (era 15): perto das bordas anda devagar e em pulsos; de -1,7 a +11 cm esgotou os 15 s (2026-11-21).
    ap.add_argument('--tempo-max-s', type=float, default=30.0)
    # TEMPORARIO (2026-11-20, sem homing ate a peca nova da chave): 20 mm antes dos contatos medidos,
    # chaves em -12,3 cm (direita, calculado) e 13,7 cm (esquerda, medido), com a camera recalibrada
    # e o tag a 25,7 cm (curso de 26 cm conferido na trena). Trocar pelo homing depois.
    ap.add_argument('--y-min', type=float, default=-0.103, help='limite do curso, direita (m, robo_base)')
    ap.add_argument('--y-max', type=float, default=0.117, help='limite do curso, esquerda (m, robo_base)')
    ap.add_argument('--zona-borda', type=float, default=0.02,
                    help='a menos disso de --y-min/--y-max anda devagar (m); 0 desliga')
    ap.add_argument('--segurar-mm', type=float, default=SEGURAR_MM,
                    help=f'depois de chegar, corrige se sair mais que isso (mm); 0 desliga (padrao {SEGURAR_MM:.0f})')
    ap.add_argument('--testar-sentido', action='store_true')
    ap.add_argument('--seguir-cubo', action='store_true',
                    help=f'alinha o plano da garra com o y do cubo visto ({TOPICO_CUBO}), sem parar')
    ap.add_argument('--estavel-s', type=float, default=1.0,
                    help='com --seguir-cubo: tempo que o cubo precisa ficar parado para virar alvo (s)')
    ap.add_argument('--max-salto', type=float, default=0.015,
                    help='salto maximo do y do tag entre leituras; maior = leitura falsa (m)')
    ap.add_argument('--histerese', type=float, default=0.01,
                    help='com --seguir-cubo: so realinha se o cubo andar mais que isso (m)')
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
