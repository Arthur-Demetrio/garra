#!/usr/bin/env python3
"""Pega um cubo numa posicao informada e volta para a pose pronta.

Coordenadas no frame robo_base (centro da base do robo no chao; X frente, Z cima), em metros.
O braco e planar: o cubo precisa estar no plano da garra (Y do TCP, ~-0,029 m).

Sequencia: abrir dedos -> pre-pegada (recuada na direcao de aproximacao) -> reta ate o cubo ->
descida extra (--descer) ->
fechar dedos -> conferir a chave dos dedos (/garra/cubo_preso) -> subir em reta -> pronta.
Se a chave disser que nao ha cubo, abre os dedos, sobe e volta para pronta (codigo 2).

A inclinacao da garra e escolhida sozinha: o script testa de -90 (para baixo) a 0 graus
(horizontal) e usa o meio da faixa que funciona para o cubo pedido.

Uso (com garra_real.launch.py ou demo.launch.py rodando):
    python3 scripts/pegar_cubo.py 0.46 0.021              # cubo no chao, 46 cm a frente do centro
    python3 scripts/pegar_cubo.py 0.46 0.021 --so-planejar  # so planeja, nao mexe a garra
    python3 scripts/pegar_cubo.py 0.46 0.021 --soltar       # devolve o cubo no mesmo lugar
    python3 scripts/pegar_cubo.py                           # escuta /pegar_cubo e pega a cada posicao
    ros2 topic pub --once /pegar_cubo geometry_msgs/msg/Point "{x: 0.46, z: 0.021}"

No topico, so x e z sao usados; o y e ignorado enquanto o braco nao tiver o motor do eixo Y.
"""

import argparse
import math
import sys

from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, Pose, Quaternion
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from cliente_garra import ClienteGarra, PRONTA, SUCESSO

FRAME = 'robo_base'
TOPICO = '/pegar_cubo'
# TEMPORARIO: aberta em 65 graus (o normal e 0.0).
ABERTA = math.radians(65.0)
# Graus do comando do J4 ao fechar no cubo. Em 95 graus os dedos forcam o cubo e prendem as
# engrenagens; use o menor valor que ainda segura o cubo (--fechamento).
FECHAMENTO_GRAUS = 85.0
# mm que a ponta desce alem do que o modelo permite antes de fechar: a malha P_1 encosta no chao
# do modelo antes da garra real. 5 mm ja pegou o cubo no chao (teste de 2026-11-12); padrao 10 mm.
DESCER_MM = 10.0
RECUO = 0.06          # m: pre-pegada recuada na direcao de aproximacao
SUBIDA = 0.05         # m: quanto sobe depois de pegar
INCLINACOES = range(-90, 1, 5)  # graus; negativo = ponta para baixo
# m acima do centro do cubo onde fica a ponta. Perto do chao o centro (21 mm) fica fora do
# alcance; ate 15 mm acima ainda prende o cubo de 42 mm pela metade de cima.
ALTURAS_PEGADA = (0.0, 0.005, 0.010, 0.015)
# Chave fim de curso dos dedos (firmware "S", publicada pelo driver). True = cubo preso.
TOPICO_SENSOR = '/garra/cubo_preso'
# O firmware fecha os dedos em rampa (~73 graus/s) e o driver informa o comando na hora: a acao dos
# dedos termina antes de eles fecharem de verdade. Espera ate a chave fechar, no maximo este tempo.
ESPERA_SENSOR = 1.5


def orientacao(inclinacao):
    """Quaternion do tool0 em robo_base com a aproximacao (+Z do tool0) inclinada no plano XZ.

    Eixos do tool0: X = -Y do robo (eixo das juntas), Z = aproximacao, Y = Z x X.
    """
    c, s = math.cos(inclinacao), math.sin(inclinacao)
    m = [[0.0, s, c],
         [-1.0, 0.0, 0.0],
         [0.0, -c, s]]  # colunas: eixos X, Y, Z do tool0
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        k = 2.0 * math.sqrt(tr + 1.0)
        q = (0.25 * k, (m[2][1] - m[1][2]) / k, (m[0][2] - m[2][0]) / k, (m[1][0] - m[0][1]) / k)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        k = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
        q = ((m[2][1] - m[1][2]) / k, 0.25 * k, (m[0][1] + m[1][0]) / k, (m[0][2] + m[2][0]) / k)
    elif m[1][1] > m[2][2]:
        k = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
        q = ((m[0][2] - m[2][0]) / k, (m[0][1] + m[1][0]) / k, 0.25 * k, (m[1][2] + m[2][1]) / k)
    else:
        k = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
        q = ((m[1][0] - m[0][1]) / k, (m[0][2] + m[2][0]) / k, (m[1][2] + m[2][1]) / k, 0.25 * k)
    return Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])


def pose(x, y, z, inclinacao):
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation = orientacao(inclinacao)
    return p


def para_base_link(d):
    """Deslocamento de robo_base para base_link (girado 180 graus em Z)."""
    return (-d[0], -d[1], d[2])


class Coleta:

    def __init__(self, c, velocidade):
        self.c = c
        self.v = velocidade
        self.dedos = ActionClient(c, GripperCommand, '/garra_dedos_controller/gripper_cmd')
        self.cubo_preso = None   # None = sensor nunca publicou (simulacao ou firmware antigo)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        c.create_subscription(Bool, TOPICO_SENSOR, self._sensor, qos)

    def _sensor(self, msg):
        self.cubo_preso = msg.data

    def conferir_cubo(self):
        """Le a chave dos dedos depois de fechar. Retorna True, False ou None (sem sensor)."""
        fim = self.c.get_clock().now().nanoseconds + int(ESPERA_SENSOR * 1e9)
        while self.c.get_clock().now().nanoseconds < fim and not self.cubo_preso:
            rclpy.spin_once(self.c, timeout_sec=0.05)
        return self.cubo_preso

    def plano_de_pegada(self, x, z, q_ini):
        """Melhor inclinacao para o cubo em (x, z). Retorna dict com as etapas ou None."""
        y = self.c.fk(q_ini, FRAME).position.y
        for dz in ALTURAS_PEGADA:
            validas = self._inclinacoes_validas(x, y, z + dz)
            if validas:
                print(f'Ponta {dz * 1000:.0f} mm acima do centro do cubo. Inclinacoes possiveis: '
                      + ', '.join(str(v['graus']) for v in validas))
                return validas[len(validas) // 2]
        return None

    def _inclinacoes_validas(self, x, y, z):
        validas = []
        for graus in INCLINACOES:
            a = math.radians(graus)
            ax, az = math.cos(a), math.sin(a)
            q_pega = self.c.ik(pose(x, y, z, a), PRONTA, frame=FRAME)
            if q_pega is None:
                continue
            q_pre = self.c.ik(pose(x - RECUO * ax, y, z - RECUO * az, a), q_pega, frame=FRAME)
            if q_pre is None:
                continue
            entrada = self.c.linha_cartesiana(
                q_pre, para_base_link((RECUO * ax, 0.0, RECUO * az)), velocidade=self.v)
            if entrada.fracao < 0.99:
                continue
            saida = self.c.linha_cartesiana(
                q_pega, (0.0, 0.0, SUBIDA), evitar_colisao=False, velocidade=self.v)
            if saida.fracao < 0.99:
                continue
            validas.append(dict(graus=graus, q_pre=q_pre, q_pega=q_pega, entrada=entrada, saida=saida))
        return validas

    def mover_dedos(self, posicao):
        if not self.dedos.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('Acao dos dedos indisponivel.')
        objetivo = GripperCommand.Goal()
        objetivo.command.position = posicao
        envio = self.dedos.send_goal_async(objetivo)
        rclpy.spin_until_future_complete(self.c, envio)
        resultado = envio.result().get_result_async()
        rclpy.spin_until_future_complete(self.c, resultado)
        return resultado.result().result

    def executar(self, nome, trajetoria):
        print(f'-> {nome}')
        codigo = self.c.executar(trajetoria)
        if codigo != SUCESSO:
            raise RuntimeError(f'{nome}: execucao falhou (codigo {codigo})')

    def ir_para(self, nome, q_alvo):
        traj, codigo = self.c.planejar_juntas(self.c.juntas_atuais(), q_alvo, self.v)
        if codigo != SUCESSO:
            raise RuntimeError(f'{nome}: planejamento falhou (codigo {codigo})')
        self.executar(nome, traj)


def pegar(c, coleta, x, z, args):
    """Pega (e opcionalmente solta) o cubo com centro em (x, z) de robo_base. Retorna 0 se deu certo."""
    q_atual = c.juntas_atuais()
    plano = coleta.plano_de_pegada(x, z, q_atual)
    if plano is None:
        print(f'Cubo em x={x:.3f} z={z:.3f} fora do alcance (nenhuma inclinacao serve).')
        return 1
    print(f'Inclinacao escolhida: {plano["graus"]} graus')
    descer = args.descer / 1000.0
    if descer > 0.0:
        teste = c.linha_cartesiana(plano['q_pega'], (0.0, 0.0, -descer), evitar_colisao=False)
        if teste.fracao < 0.99:
            print(f'Nao da para descer {args.descer:.0f} mm a mais (limite das juntas).')
            return 1
        print(f'Desce mais {args.descer:.0f} mm antes de fechar.')
    if args.so_planejar:
        return 0

    coleta.mover_dedos(ABERTA)
    coleta.ir_para('pre-pegada', plano['q_pre'])
    # Replaneja a entrada a partir de onde a garra parou, para o ponto inicial bater.
    a = math.radians(plano['graus'])
    entrada = c.linha_cartesiana(
        c.juntas_atuais(), para_base_link((RECUO * math.cos(a), 0.0, RECUO * math.sin(a))),
        velocidade=args.velocidade)
    coleta.executar('entrada', entrada.trajetoria)
    if descer > 0.0:
        ajuste = c.linha_cartesiana(
            c.juntas_atuais(), (0.0, 0.0, -descer), evitar_colisao=False,
            velocidade=args.velocidade)
        coleta.executar('descida extra', ajuste.trajetoria)
    coleta.mover_dedos(math.radians(args.fechamento))
    if not args.sem_sensor:
        preso = coleta.conferir_cubo()
        if preso is None:
            print(f'AVISO: nada em {TOPICO_SENSOR} (simulacao ou firmware sem sensor); seguindo sem conferir.')
        elif not preso:
            print('Sensor dos dedos: cubo NAO preso. Abrindo, subindo e voltando para pronta.')
            coleta.mover_dedos(ABERTA)
            saida = c.linha_cartesiana(
                c.juntas_atuais(), (0.0, 0.0, SUBIDA), evitar_colisao=False,
                velocidade=args.velocidade)
            coleta.executar('subida sem cubo', saida.trajetoria)
            coleta.ir_para('pronta', PRONTA)
            return 2
        else:
            print('Sensor dos dedos: cubo preso.')
    saida = c.linha_cartesiana(
        c.juntas_atuais(), (0.0, 0.0, SUBIDA), evitar_colisao=False, velocidade=args.velocidade)
    coleta.executar('subida', saida.trajetoria)
    if args.soltar:
        # Volta para a mesma altura em que pegou, incluindo a descida extra.
        descida = c.linha_cartesiana(
            c.juntas_atuais(), (0.0, 0.0, -SUBIDA), evitar_colisao=False,
            velocidade=args.velocidade)
        coleta.executar('descida', descida.trajetoria)
        coleta.mover_dedos(ABERTA)
        if descer > 0.0:
            volta = c.linha_cartesiana(
                c.juntas_atuais(), (0.0, 0.0, descer), evitar_colisao=False,
                velocidade=args.velocidade)
            coleta.executar('saida da descida extra', volta.trajetoria)
        recuo = c.linha_cartesiana(
            c.juntas_atuais(),
            para_base_link((-RECUO * math.cos(a), 0.0, -RECUO * math.sin(a))),
            velocidade=args.velocidade)
        coleta.executar('recuo', recuo.trajetoria)
    coleta.ir_para('pronta', PRONTA)
    print('Pronto.')
    return 0


def escutar(c, coleta, args):
    """Fica no topico TOPICO e pega um cubo a cada geometry_msgs/Point recebido."""
    pendentes = []
    c.create_subscription(Point, TOPICO, pendentes.append, 10)
    print(f'Esperando posicoes em {TOPICO} (geometry_msgs/msg/Point, x e z em robo_base). '
          'Ctrl+C para sair. Exemplo:')
    print(f'  ros2 topic pub --once {TOPICO} geometry_msgs/msg/Point "{{x: 0.46, z: 0.021}}"')
    while rclpy.ok():
        rclpy.spin_once(c, timeout_sec=0.1)
        if not pendentes:
            continue
        p = pendentes.pop(0)
        if pendentes:
            print(f'Descartando {len(pendentes)} posicao(oes) recebida(s) durante o movimento.')
            pendentes.clear()
        # O y e ignorado: o braco ainda e planar (sem o motor do eixo Y).
        print(f'Recebido x={p.x:.3f} z={p.z:.3f}')
        try:
            pegar(c, coleta, p.x, p.z, args)
        except RuntimeError as e:
            print(f'ERRO: {e}')
        print(f'Esperando a proxima posicao em {TOPICO}...')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('x', type=float, nargs='?',
                    help=f'centro do cubo em X de robo_base (m); sem x e z, escuta {TOPICO}')
    ap.add_argument('z', type=float, nargs='?', help='centro do cubo em Z de robo_base (m); no chao: 0.021')
    ap.add_argument('--velocidade', type=float, default=0.2, help='escala de velocidade (0-1)')
    ap.add_argument('--fechamento', type=float, default=FECHAMENTO_GRAUS,
                    help=f'graus do J4 ao fechar no cubo (padrao {FECHAMENTO_GRAUS:.0f}, max 100)')
    ap.add_argument('--descer', type=float, default=DESCER_MM,
                    help='mm a descer em reta depois de chegar no cubo, sem checar colisao com o '
                         f'chao (o modelo e conservador perto do chao); padrao {DESCER_MM:.0f}, max 20')
    ap.add_argument('--so-planejar', action='store_true', help='planeja sem mexer a garra')
    ap.add_argument('--soltar', action='store_true', help='devolve o cubo no mesmo lugar')
    ap.add_argument('--sem-sensor', action='store_true',
                    help=f'nao confere a chave dos dedos ({TOPICO_SENSOR}) depois de fechar')
    args = ap.parse_args()
    if (args.x is None) != (args.z is None):
        ap.error('informe x e z juntos, ou nenhum dos dois para escutar o topico')
    if not 0.0 <= args.descer <= 20.0:
        ap.error('--descer deve ficar entre 0 e 20 mm')
    if not 0.0 <= args.fechamento <= 100.0:
        ap.error('--fechamento deve ficar entre 0 e 100 graus')

    rclpy.init()
    c = ClienteGarra('pegar_cubo')
    try:
        c.aguardar()
        coleta = Coleta(c, args.velocidade)
        if args.x is None:
            escutar(c, coleta, args)
            return 0
        return pegar(c, coleta, args.x, args.z, args)
    except RuntimeError as e:
        print(f'ERRO: {e}')
        return 1
    except KeyboardInterrupt:
        return 0
    finally:
        c.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
