#!/usr/bin/env python3
"""Move juntas do braco pelo MoveIt, sem o RViz (planeja com OMPL e executa).

Com o MoveIt rodando (garra_real.launch.py ou demo.launch.py):

    python3 scripts/mover_junta.py J1 0.5             # so o J1 para 0.5 rad; J2 e J3 ficam
    python3 scripts/mover_junta.py J1 0.5 J3 -0.2     # varias juntas
    python3 scripts/mover_junta.py home               # [0, 0, 0] (vertical do CAD)
    python3 scripts/mover_junta.py pronta
    python3 scripts/mover_junta.py                    # so mostra as juntas atuais

Opcoes: --graus (valores em graus), --velocidade 0.1 (escala 0-1, padrao 0.1).
"""

import argparse
import math
import os
import sys

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cliente_garra import ClienteGarra, HOME, JUNTAS, LIMITES, PRONTA, SUCESSO  # noqa: E402

NOMES = {'J1': 0, 'J2': 1, 'J3': 2}
POSES = {'home': HOME, 'pronta': PRONTA}


def mostrar(q):
    return ', '.join(f'J{i + 1} {v:+.3f} rad ({math.degrees(v):+.1f} graus)' for i, v in enumerate(q))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('alvo', nargs='*', help='home | pronta | J<n> <valor> [J<n> <valor> ...]')
    parser.add_argument('--graus', action='store_true', help='valores das juntas em graus')
    parser.add_argument('--velocidade', type=float, default=0.1)
    args = parser.parse_args()

    rclpy.init()
    cliente = ClienteGarra('mover_junta')
    try:
        cliente.aguardar()
        q_ini = cliente.juntas_atuais()
        print('Atual: ' + mostrar(q_ini))
        if not args.alvo:
            return 0

        if len(args.alvo) == 1 and args.alvo[0].lower() in POSES:
            q_alvo = list(POSES[args.alvo[0].lower()])
        else:
            if len(args.alvo) % 2:
                parser.error('use pares J<n> <valor>, ou home / pronta')
            q_alvo = list(q_ini)
            for nome, texto in zip(args.alvo[::2], args.alvo[1::2]):
                if nome.upper() not in NOMES:
                    parser.error(f'junta {nome} desconhecida: use J1, J2 ou J3')
                valor = float(texto)
                q_alvo[NOMES[nome.upper()]] = math.radians(valor) if args.graus else valor

        for i, (v, (baixo, alto)) in enumerate(zip(q_alvo, LIMITES)):
            if not baixo <= v <= alto:
                print(f'J{i + 1} = {v:.3f} rad fora do limite [{baixo}, {alto}] ({JUNTAS[i]})')
                return 1

        print('Alvo:  ' + mostrar(q_alvo))
        trajetoria, codigo = cliente.planejar_juntas(q_ini, q_alvo, args.velocidade)
        if codigo != SUCESSO:
            print(f'Planejamento falhou (codigo {codigo}): colisao ou alvo invalido?')
            return 1
        codigo = cliente.executar(trajetoria)
        if codigo != SUCESSO:
            print(f'Execucao falhou (codigo {codigo})')
            return 1
        print('Chegou: ' + mostrar(cliente.juntas_atuais()))
        return 0
    finally:
        cliente.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
