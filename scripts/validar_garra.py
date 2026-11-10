"""Validacao da garra em simulacao, antes de ligar o hardware.

Uso, com o MoveIt rodando (ros2 launch garra_moveit_config demo.launch.py):

    python3 scripts/validar_garra.py

Move o robo simulado. NAO rode contra a garra fisica. Sai com codigo 0 se tudo passar.
"""

import math
import random
import sys

import rclpy

from cliente_garra import ClienteGarra, HOME, JUNTAS, LIMITES, PRONTA, SUCESSO


TOL_POSICAO = 0.001   # m: erro aceito da IK
TOL_RETA = 0.001      # m: desvio aceito em relacao a reta pedida
TOL_FINAL = 0.03      # rad: erro de junta aceito ao fim de uma execucao
MIN_IK = 0.95         # o KDL e aleatorio e pode estourar o tempo perto dos limites
TCP_HOME = (-0.0710, 0.0290, 0.4245)  # m, em base_link: pose do CAD (servos em 90 graus), dedos fixos


class Relatorio:

    def __init__(self):
        self.falhas = 0

    def secao(self, titulo):
        print(f'\n--- {titulo} ---')

    def item(self, ok, texto):
        print(f'[{"PASSOU" if ok else "FALHOU"}] {texto}')
        if not ok:
            self.falhas += 1


def distancia(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def xyz(pose):
    return (pose.position.x, pose.position.y, pose.position.z)


def testar_tcp(c, r):
    r.secao('TCP')
    erro = distancia(xyz(c.fk(HOME)), TCP_HOME)
    r.item(erro < 0.001, f'tool0 na ponta da garra em q=0 (erro {erro * 1000:.2f} mm)')
    # Sentido da garra fisica: J3 positiva dobra a garra para a frente (-X), como J1.
    # O eixo vem corrigido por scripts/ajustar_urdf.py (EIXOS).
    d = xyz(c.fk([0.0, 0.0, math.pi / 2]))
    # A garra ja parte inclinada ~25 graus para a frente, entao o TCP avanca ~7 cm em X.
    r.item(d[0] < TCP_HOME[0] - 0.05, f'J3 positiva leva a garra para a frente, -X (X={d[0]:.3f})')


def testar_colisao(c, r):
    r.secao('Autocolisao')
    for nome, q, esperado in [
        ('home (q=0)', HOME, True),
        ('pronta', PRONTA, True),
        ('esticado para a frente e para baixo: contra o chao', [1.5, -1.5, -0.5], False),
        ('J1=-60 graus: contra a estrutura do robo', [math.radians(-60), 0.0, 0.0], False),
    ]:
        valido, pares = c.valido(q)
        detalhe = f' {pares[:2]}' if pares else ''
        r.item(valido == esperado, f'{nome}: {"livre" if valido else "em colisao"}{detalhe}')


def testar_ik(c, r, amostras=100):
    r.secao(f'IK: ida e volta em {amostras} configuracoes sem colisao')
    rng = random.Random(42)
    ok = 0
    testadas = 0
    pior = 0.0
    while testadas < amostras:
        q = [rng.uniform(lo, hi) for lo, hi in LIMITES]
        if not c.valido(q)[0]:
            continue
        testadas += 1
        pose = c.fk(q)
        alvo = xyz(pose)
        # Semente proxima, como acontece em uma trajetoria real
        semente = [
            min(max(v + rng.uniform(-0.3, 0.3), lo), hi)
            for v, (lo, hi) in zip(q, LIMITES)
        ]
        sol = c.ik(pose, semente)
        if sol is None:
            continue
        erro = distancia(alvo, xyz(c.fk(sol)))
        pior = max(pior, erro)
        ok += erro < TOL_POSICAO
    r.item(ok / amostras >= MIN_IK,
           f'{ok}/{amostras} resolvidas, pior erro de posicao {pior * 1000:.3f} mm')


def testar_limites_trajetoria(c, r, nome, trajetoria):
    pontos = trajetoria.joint_trajectory.points
    nomes = list(trajetoria.joint_trajectory.joint_names)
    fora = 0
    for p in pontos:
        for valor, (lo, hi), j in zip(p.positions, LIMITES, JUNTAS):
            assert nomes[JUNTAS.index(j)] == j
            fora += not (lo - 1e-6 <= valor <= hi + 1e-6)
    r.item(fora == 0, f'{nome}: {len(pontos)} pontos dentro dos limites das juntas')


def mover(c, r, nome, alvo):
    q = c.juntas_atuais()
    trajetoria, codigo = c.planejar_juntas(q, alvo)
    r.item(codigo == SUCESSO, f'planejar {nome} (codigo {codigo})')
    if codigo != SUCESSO:
        return False
    testar_limites_trajetoria(c, r, nome, trajetoria)
    codigo = c.executar(trajetoria)
    erro = max(abs(a - b) for a, b in zip(c.juntas_atuais(), alvo))
    r.item(codigo == SUCESSO and erro < TOL_FINAL,
           f'executar {nome} (codigo {codigo}, erro final {erro:.4f} rad)')
    return codigo == SUCESSO


def testar_retas(c, r):
    r.secao('Retas cartesianas a partir de "pronta"')
    if not mover(c, r, 'ate pronta', PRONTA):
        return
    q = c.juntas_atuais()
    for nome, d in [
        # Curtas: com a inclinacao fixa, a garra tem pouca folga em linha reta (ver PRONTA).
        ('X -2 cm', (-0.02, 0, 0)),
        ('X +2 cm', (0.02, 0, 0)),
        ('Z +2 cm', (0, 0, 0.02)),
        ('Z -2 cm', (0, 0, -0.02)),
    ]:
        res = c.linha_cartesiana(q, d)
        extra = f', {res.mensagem}' if res.mensagem else ''
        r.item(
            res.fracao >= 0.999 and res.desvio_max < TOL_RETA,
            f'{nome}: {res.fracao * 100:.0f}% da reta, desvio {res.desvio_max * 1000:.3f} mm, '
            f'maior salto de junta {res.salto_max:.3f} rad{extra}')
    fora = c.linha_cartesiana(q, (0, 0.01, 0))
    r.item(fora.fracao < 0.2, f'Y +1 cm recusado, fora do plano: {fora.fracao * 100:.0f}%')

    r.secao('Execucao de reta (ida e volta)')
    ida = c.linha_cartesiana(q, (-0.02, 0, -0.02))
    r.item(ida.fracao >= 0.999, f'calcular ida: {ida.fracao * 100:.0f}% {ida.mensagem}')
    if ida.fracao < 0.999:
        return
    inicio = xyz(c.fk(c.juntas_atuais()))
    codigo = c.executar(ida.trajetoria)
    fim = xyz(c.fk(c.juntas_atuais()))
    alvo = (inicio[0] - 0.02, inicio[1], inicio[2] - 0.02)
    erro = distancia(fim, alvo)
    r.item(codigo == SUCESSO and erro < 0.002,
           f'executar ida (codigo {codigo}, TCP a {erro * 1000:.2f} mm do alvo)')
    volta = c.linha_cartesiana(c.juntas_atuais(), (0.02, 0, 0.02))
    r.item(volta.fracao >= 0.999, f'calcular volta: {volta.fracao * 100:.0f}% {volta.mensagem}')
    if volta.fracao >= 0.999:
        codigo = c.executar(volta.trajetoria)
        erro = distancia(xyz(c.fk(c.juntas_atuais())), inicio)
        r.item(codigo == SUCESSO and erro < 0.002,
               f'executar volta (codigo {codigo}, TCP a {erro * 1000:.2f} mm do inicio)')


def testar_planejamento(c, r):
    r.secao('Planejamento com desvio de colisao')
    if not mover(c, r, 'de volta a home', HOME):
        return
    if not mover(c, r, 'home -> pronta', PRONTA):
        return
    pose = c.fk([0.2, -1.1, -0.2])  # garra para a frente, quase horizontal
    alvo = xyz(pose)
    trajetoria, codigo = c.planejar_pose(c.juntas_atuais(), pose)
    r.item(codigo == SUCESSO, f'planejar ate uma pose do TCP (codigo {codigo})')
    if codigo == SUCESSO:
        codigo = c.executar(trajetoria)
        erro = distancia(xyz(c.fk(c.juntas_atuais())), alvo)
        r.item(codigo == SUCESSO and erro < 0.003,
               f'executar (codigo {codigo}, TCP a {erro * 1000:.2f} mm do alvo)')
    mover(c, r, 'retorno a pronta', PRONTA)


def main():
    rclpy.init()
    c = ClienteGarra('validar_garra')
    r = Relatorio()
    try:
        print('Aguardando os servicos e a acao do MoveIt...', flush=True)
        c.aguardar()
        print('MoveIt encontrado. Iniciando validacao...', flush=True)
        testar_tcp(c, r)
        testar_colisao(c, r)
        testar_ik(c, r)
        testar_retas(c, r)
        testar_planejamento(c, r)
    finally:
        c.destroy_node()
        rclpy.try_shutdown()
    print(f'\n{"TUDO PASSOU" if r.falhas == 0 else f"{r.falhas} VERIFICACAO(OES) FALHARAM"}')
    return 0 if r.falhas == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
