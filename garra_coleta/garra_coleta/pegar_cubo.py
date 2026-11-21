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
    ros2 run garra_coleta pegar_cubo 0.46 0.021              # cubo no chao, 46 cm a frente do centro
    ros2 run garra_coleta pegar_cubo 0.46 0.021 --so-planejar  # so planeja, nao mexe a garra
    ros2 run garra_coleta pegar_cubo 0.46 0.021 --soltar       # devolve o cubo no mesmo lugar
    ros2 run garra_coleta pegar_cubo 0.46 0.021 --entregar     # SO solta o cubo que ja esta na garra
    ros2 run garra_coleta pegar_cubo                           # escuta /pegar_cubo e /entregar_cubo
    ros2 topic pub --once /pegar_cubo geometry_msgs/msg/Point "{x: 0.46, z: 0.021}"
    ros2 topic pub --once /entregar_cubo geometry_msgs/msg/Point "{x: 0.46, z: 0.021}"

Entrega (--entregar ou /entregar_cubo): com o cubo ja preso nos dedos, leva-o ate (x, z) - o
centro de onde o cubo deve ficar, ex.: z = altura da mesa + 0.021 -, pela mesma aproximacao da
pegada (pre-pegada -> reta), abre os dedos, recua em reta e volta para pronta. Nao abre os dedos
antes de chegar (diferente do --soltar, que pega e devolve).

Cada posicao recebida gera um resultado em /pegar_cubo/resultado ou /entregar_cubo/resultado
(std_msgs/String): 'ok', 'sem_cubo' (so na pegada: a chave dos dedos nao achou o cubo) ou
'falhou: motivo'.
No topico, so x e z sao usados; o y e ignorado enquanto o braco nao tiver o motor do eixo Y.
"""

import argparse
import math
import signal
import sys

from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, Pose, Quaternion
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from garra_coleta.cliente_garra import ClienteGarra, PRONTA, SUCESSO

FRAME = 'robo_base'
TOPICO = '/pegar_cubo'
# Resultado de cada posicao recebida no topico: 'ok', 'sem_cubo' ou 'falhou: motivo'.
TOPICO_RESULTADO = '/pegar_cubo/resultado'
TOPICO_ENTREGA = '/entregar_cubo'
TOPICO_ENTREGA_RESULTADO = '/entregar_cubo/resultado'
# TEMPORARIO: fechamento em 90 desde 2026-11-17 (engrenagem dos dedos remontada; o normal e 0.0).
# Aberta em 50 graus desde 2026-11-17; 60 graus desde 2026-11-20.
ABERTA = math.radians(60.0)
# Graus do comando do J4 ao fechar no cubo. Em 95 graus os dedos forcam o cubo e prendem as
# engrenagens; use o menor valor que ainda segura o cubo (--fechamento).
FECHAMENTO_GRAUS = 90.0
# mm que a ponta desce alem do que o modelo permite antes de fechar. Era 10 mm para compensar a
# calibracao antiga (garra real acima do modelo). Com as escalas de 2026-11-16 a garra real fica ate
# ~1,5 cm ABAIXO do modelo com o braco esticado: padrao 0, para nao empurrar os dedos no chao.
# 2026-11-20: com os zeros novos (pose do modelo em q=0), testado na bancada a 18,5 cm da borda:
# 0 e 10 mm fecham acima do cubo; 15 mm fecha no meio das laterais. Conferido tambem a 13 cm
# (garra a -90 graus) e a 21 cm (-65 graus): meio das laterais nos dois.
# 2026-11-21: os testes anteriores (15 mm em 2026-11-20 e 15-30 mm em 2026-11-21) foram com o robo num
# SUPORTE de ~5 cm e o cubo na mesa, ou seja, o cubo 50 mm abaixo do z passado: nao valem. Com o robo no
# chao (cubo no nivel das rodas, z = 0.021), 10 mm prende bem a 17 cm sem encostar no chao.
DESCER_MM = 10.0
# Se a descida inteira nao cabe nas juntas (perto do alcance maximo), desce o que der, desde que seja
# ao menos esta fracao do pedido; abaixo disso desiste (os dedos fechariam alto demais no cubo).
DESCER_MIN_FRACAO = 0.5
# Descida extra na ENTREGA (--entregar / /entregar_cubo). Separada da pegada: com o cubo nos dedos, descer
# os mesmos mm empurra o cubo contra o chao (2026-11-21, com 10 mm).
DESCER_ENTREGA_MM = 0.0
# Correcao de x medida na garra real: (x do cubo em robo_base, m) -> quanto somar ao x mandado ao modelo.
# 2026-11-21, robo no chao: a 17 cm da borda (0.445) acerta; a 13 cm (0.405) a garra fechava em ~14,5 cm
# (1,5 cm a frente); a 14 cm (0.415, -11 mm interpolado) e a 21 cm (0.485, sem correcao) pega e entrega
# certinho. Entre pontos interpola; fora, usa o ponto mais perto.
CORRECAO_X = [(0.405, -0.015), (0.445, 0.0), (0.485, 0.0)]
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


def para_garra_base(d):
    """Deslocamento de robo_base para garra_base (girado 180 graus em Z)."""
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

    def plano_de_pegada(self, x, z, q_ini, descer=0.0):
        """Melhor inclinacao para o cubo em (x, z). Retorna dict com as etapas ou None.

        Com descer > 0, prefere as inclinacoes em que a descida extra inteira cabe nas juntas; o
        dict leva 'descer' = quanto da para descer de fato (m) na inclinacao escolhida."""
        y = self.c.fk(q_ini, FRAME).position.y
        for dz in ALTURAS_PEGADA:
            validas = self._inclinacoes_validas(x, y, z + dz)
            if validas:
                print(f'Ponta {dz * 1000:.0f} mm acima do centro do cubo. Inclinacoes possiveis: '
                      + ', '.join(str(v['graus']) for v in validas))
                for v in validas:
                    v['descer'] = self._descida_possivel(v['q_pega'], descer)
                inteiras = [v for v in validas if v['descer'] >= descer - 1e-4]
                escolha = inteiras or sorted(validas, key=lambda v: v['descer'])[-1:]
                return escolha[len(escolha) // 2]
        return None

    def _descida_possivel(self, q_pega, descer):
        """Quanto (m) da para descer em reta a partir de q_pega, ate descer."""
        if descer <= 0.0:
            return 0.0
        teste = self.c.linha_cartesiana(q_pega, (0.0, 0.0, -descer), evitar_colisao=False)
        return descer if teste.fracao >= 0.99 else descer * max(teste.fracao, 0.0)

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
                q_pre, para_garra_base((RECUO * ax, 0.0, RECUO * az)), velocidade=self.v)
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


def descida_usada(plano, pedido_mm):
    """Descida extra (m) a usar no plano, ou None se nao der nem DESCER_MIN_FRACAO do pedido."""
    pedido = pedido_mm / 1000.0
    if pedido <= 0.0:
        return 0.0
    possivel = plano['descer']
    if possivel >= pedido - 1e-4:
        print(f'Desce mais {pedido_mm:.0f} mm.')
        return pedido
    if possivel < DESCER_MIN_FRACAO * pedido:
        print(f'Nao da para descer {pedido_mm:.0f} mm a mais (limite das juntas; so '
              f'{possivel * 1000:.0f} mm).')
        return None
    print(f'AVISO: so da para descer {possivel * 1000:.0f} de {pedido_mm:.0f} mm (limite das juntas): '
          f'os dedos fecham {(pedido - possivel) * 1000:.0f} mm mais alto.')
    return possivel


def x_corrigido(x):
    """x a mandar para o modelo para a garra real chegar em x (interpola CORRECAO_X; fora, o mais perto)."""
    pontos = sorted(CORRECAO_X)
    if x <= pontos[0][0]:
        dx = pontos[0][1]
    elif x >= pontos[-1][0]:
        dx = pontos[-1][1]
    else:
        for (x0, d0), (x1, d1) in zip(pontos, pontos[1:]):
            if x0 <= x <= x1:
                dx = d0 + (d1 - d0) * (x - x0) / (x1 - x0)
                break
    if abs(dx) >= 0.0005:
        print(f'Correcao de x: {x:.3f} -> {x + dx:.3f} m ({dx * 1000:+.0f} mm)')
    return x + dx


def pegar(c, coleta, x, z, args):
    """Pega (e opcionalmente solta) o cubo com centro em (x, z) de robo_base. Retorna 0 se deu certo."""
    x = x_corrigido(x)
    q_atual = c.juntas_atuais()
    plano = coleta.plano_de_pegada(x, z, q_atual, args.descer / 1000.0)
    if plano is None:
        print(f'Cubo em x={x:.3f} z={z:.3f} fora do alcance (nenhuma inclinacao serve).')
        return 1
    print(f'Inclinacao escolhida: {plano["graus"]} graus')
    descer = descida_usada(plano, args.descer)
    if descer is None:
        return 1
    if args.so_planejar:
        return 0

    coleta.mover_dedos(ABERTA)
    coleta.ir_para('pre-pegada', plano['q_pre'])
    # Replaneja a entrada a partir de onde a garra parou, para o ponto inicial bater.
    a = math.radians(plano['graus'])
    entrada = c.linha_cartesiana(
        c.juntas_atuais(), para_garra_base((RECUO * math.cos(a), 0.0, RECUO * math.sin(a))),
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
            para_garra_base((-RECUO * math.cos(a), 0.0, -RECUO * math.sin(a))),
            velocidade=args.velocidade)
        coleta.executar('recuo', recuo.trajetoria)
    coleta.ir_para('pronta', PRONTA)
    print('Pronto.')
    return 0


def entregar(c, coleta, x, z, args):
    """Solta o cubo que ja esta nos dedos com o centro em (x, z) de robo_base. Retorna 0 se deu certo.

    Mesma aproximacao da pegada (pre-pegada recuada -> reta), sem abrir os dedos antes de chegar.
    """
    x = x_corrigido(x)
    plano = coleta.plano_de_pegada(x, z, c.juntas_atuais(), args.descer_entrega / 1000.0)
    if plano is None:
        print(f'Destino x={x:.3f} z={z:.3f} fora do alcance (nenhuma inclinacao serve).')
        return 1
    print(f'Inclinacao escolhida: {plano["graus"]} graus')
    descer = descida_usada(plano, args.descer_entrega)
    if descer is None:
        return 1
    if args.so_planejar:
        return 0

    a = math.radians(plano['graus'])
    coleta.ir_para('pre-entrega (com o cubo)', plano['q_pre'])
    entrada = c.linha_cartesiana(
        c.juntas_atuais(), para_garra_base((RECUO * math.cos(a), 0.0, RECUO * math.sin(a))),
        velocidade=args.velocidade)
    coleta.executar('entrada (com o cubo)', entrada.trajetoria)
    if descer > 0.0:
        ajuste = c.linha_cartesiana(
            c.juntas_atuais(), (0.0, 0.0, -descer), evitar_colisao=False, velocidade=args.velocidade)
        coleta.executar('descida extra', ajuste.trajetoria)
    coleta.mover_dedos(ABERTA)
    print('Dedos abertos: cubo entregue.')
    if descer > 0.0:
        volta = c.linha_cartesiana(
            c.juntas_atuais(), (0.0, 0.0, descer), evitar_colisao=False, velocidade=args.velocidade)
        coleta.executar('saida da descida extra', volta.trajetoria)
    recuo = c.linha_cartesiana(
        c.juntas_atuais(), para_garra_base((-RECUO * math.cos(a), 0.0, -RECUO * math.sin(a))),
        velocidade=args.velocidade)
    coleta.executar('recuo', recuo.trajetoria)
    coleta.ir_para('pronta', PRONTA)
    print('Pronto.')
    return 0


def escutar(c, coleta, args):
    """Pega um cubo a cada Point em TOPICO e entrega a cada Point em TOPICO_ENTREGA."""
    pendentes = []      # (acao, Point)
    c.create_subscription(Point, TOPICO, lambda p: pendentes.append(('pegar', p)), 10)
    c.create_subscription(Point, TOPICO_ENTREGA, lambda p: pendentes.append(('entregar', p)), 10)
    pubs = {'pegar': c.create_publisher(String, TOPICO_RESULTADO, 10),
            'entregar': c.create_publisher(String, TOPICO_ENTREGA_RESULTADO, 10)}
    print(f'Esperando posicoes em {TOPICO} (pegar) e {TOPICO_ENTREGA} (entregar) '
          '(geometry_msgs/msg/Point, x e z em robo_base). Ctrl+C para sair. Exemplo:')
    print(f'  ros2 topic pub --once {TOPICO} geometry_msgs/msg/Point "{{x: 0.46, z: 0.021}}"')
    while rclpy.ok():
        rclpy.spin_once(c, timeout_sec=0.1)
        if not pendentes:
            continue
        acao, p = pendentes.pop(0)
        if pendentes:
            print(f'Descartando {len(pendentes)} pedido(s) recebido(s) durante o movimento.')
            for a, _ in pendentes:
                pubs[a].publish(String(data='falhou: descartado (garra ocupada)'))
            pendentes.clear()
        # O y e ignorado: o braco ainda e planar (sem o motor do eixo Y).
        print(f'Recebido ({acao}) x={p.x:.3f} z={p.z:.3f}')
        try:
            if acao == 'pegar':
                codigo = pegar(c, coleta, p.x, p.z, args)
                res = {0: 'ok', 2: 'sem_cubo'}.get(codigo, 'falhou: fora do alcance')
            else:
                res = 'ok' if entregar(c, coleta, p.x, p.z, args) == 0 else 'falhou: fora do alcance'
        except RuntimeError as e:
            print(f'ERRO: {e}')
            res = f'falhou: {e}'
        pubs[acao].publish(String(data=res))
        print('Esperando o proximo pedido...')



def argumentos_sem_ros():
    """sys.argv sem os --ros-args que o ros2 launch/run acrescentam."""
    try:
        from rclpy.utilities import remove_ros_args
        return remove_ros_args(sys.argv)[1:]
    except ImportError:
        return sys.argv[1:]

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
                         f'chao (o modelo e conservador perto do chao); padrao {DESCER_MM:.0f}, max 35')
    ap.add_argument('--descer-entrega', type=float, default=DESCER_ENTREGA_MM,
                    help=f'como --descer, mas na entrega (cubo nos dedos); padrao {DESCER_ENTREGA_MM:.0f}')
    ap.add_argument('--so-planejar', action='store_true', help='planeja sem mexer a garra')
    ap.add_argument('--soltar', action='store_true', help='devolve o cubo no mesmo lugar')
    ap.add_argument('--entregar', action='store_true',
                    help='so solta em (x, z) o cubo que ja esta nos dedos (nao pega)')
    ap.add_argument('--sem-sensor', action='store_true',
                    help=f'nao confere a chave dos dedos ({TOPICO_SENSOR}) depois de fechar')
    args = ap.parse_args(argumentos_sem_ros())
    if (args.x is None) != (args.z is None):
        ap.error('informe x e z juntos, ou nenhum dos dois para escutar o topico')
    if args.entregar and (args.x is None or args.soltar):
        ap.error('--entregar precisa de x e z e nao combina com --soltar')
    if not 0.0 <= args.descer <= 35.0:  # era 20; subido em 2026-11-21 nos testes com o robo no suporte
        ap.error('--descer deve ficar entre 0 e 35 mm')
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
        if args.entregar:
            return entregar(c, coleta, args.x, args.z, args)
        return pegar(c, coleta, args.x, args.z, args)
    except RuntimeError as e:
        print(f'ERRO: {e}')
        return 1
    except KeyboardInterrupt:
        return 0
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # 2o Ctrl+C (ros2 launch) nao interrompe o encerramento
        c.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
