"""Mede a escala real (us_por_rad) de J1-J3 com um inclinometro e corrige o arquivo de calibracao.

O calibrar_servos.py mede zero, sentido e faixa, mas a escala vem do valor nominal do servo
(180 graus em 544-2400 us). Este script mede a escala de verdade, sem mexer no resto.

Fale direto com o firmware: NAO rode com o ROS usando a mesma porta.

    python3 scripts/medir_escala.py [--porta /dev/ttyUSB0] [--angulo 0.8]

Para cada junta escolhida, com as outras no zero (q=0, vertical):
  1. a junta vai ao zero: encoste o celular (app de inclinometro/nivel) no segmento logo depois
     dela e anote o angulo;
  2. a junta vai a +ANGULO rad (para a frente): leia o angulo de novo, no mesmo lugar;
  3. digite quantos graus o segmento girou entre as duas leituras.
So a diferenca importa: o zero do celular e o lado em que ele encosta nao afetam a medida.

Cuidado: ao abrir a porta o Arduino reinicia e os servos vao para 90 graus na velocidade maxima.
Confira se o braco tem espaco livre para a frente antes de cada movimento.
"""

import argparse
import math
import os
import re
import sys
import time

import yaml

from calibrar_servos import GRAUS_POR_S, JUNTAS, SAIDA_PADRAO, US_POR_GRAU, Firmware


def ler_angulo(texto):
    while True:
        resposta = input(texto).strip().replace(',', '.')
        if resposta == '':
            return None
        try:
            return float(resposta)
        except ValueError:
            print('Digite um numero (graus), ou Enter para pular a junta.')


def esperar(pulso_de, pulso_para):
    time.sleep(abs(pulso_para - pulso_de) / US_POR_GRAU / GRAUS_POR_S + 1.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--porta', default='/dev/ttyUSB0')
    parser.add_argument('--arquivo', default=SAIDA_PADRAO)
    parser.add_argument('--angulo', type=float, default=0.8,
                        help='angulo do teste em rad, positivo = para a frente (padrao 0.8)')
    args = parser.parse_args()

    with open(args.arquivo) as arquivo:
        calibracao = yaml.safe_load(arquivo)['juntas']

    fw = Firmware(args.porta)
    if not fw.conectar():
        print('O firmware nao respondeu.')
        return 1
    print('Firmware conectado. Levando J1-J3 ao zero (vertical).')

    zeros = [int(calibracao[nome]['zero_us']) for nome in JUNTAS]
    pulsos = list(zeros)
    if fw.mover(pulsos) != 'K':
        print('O firmware recusou o comando.')
        return 1
    time.sleep(3.0)

    novas = {}
    for i, nome in enumerate(JUNTAS):
        c = calibracao[nome]
        alvo = round(c['zero_us'] + args.angulo * c['us_por_rad'])
        if not c['min_us'] <= alvo <= c['max_us']:
            print(f'\nJ{i + 1}: {args.angulo} rad sai da faixa calibrada; use um --angulo menor.')
            continue
        if input(f'\nJ{i + 1} ({nome}): medir? [s/N] ').strip().lower() != 's':
            continue

        a0 = ler_angulo(f'J{i + 1} no zero. Angulo lido no celular (graus): ')
        if a0 is None:
            continue
        input(f'Enter para mover J{i + 1} a {args.angulo} rad ({math.degrees(args.angulo):.0f} graus '
              'no modelo, para a frente)...')
        pulsos[i] = alvo
        fw.mover(pulsos)
        esperar(zeros[i], alvo)
        a1 = ler_angulo('Angulo lido no celular, no mesmo lugar (graus): ')

        pulsos[i] = zeros[i]
        fw.mover(pulsos)
        esperar(alvo, zeros[i])
        if a1 is None:
            continue

        girou = abs(a1 - a0)
        if girou < 5:
            print('Girou menos de 5 graus: medida descartada.')
            continue
        escala = math.copysign((alvo - c['zero_us']) / math.radians(girou), c['us_por_rad'])
        erro = 100 * (abs(escala) / abs(c['us_por_rad']) - 1)
        print(f'J{i + 1}: girou {girou:.1f} graus (modelo: {math.degrees(args.angulo):.1f}). '
              f'us_por_rad {c["us_por_rad"]} -> {escala:.1f} ({erro:+.1f}%)')
        novas[nome] = escala

    if not novas:
        print('\nNada medido; arquivo nao alterado.')
        return 0
    if input(f'\nGravar as novas escalas em {args.arquivo}? [s/N] ').strip().lower() != 's':
        print('Nada gravado.')
        return 0
    with open(args.arquivo) as arquivo:
        texto = arquivo.read()
    for nome, escala in novas.items():
        texto, n = re.subn(rf'^(  {nome}:.*us_por_rad: )-?[0-9.]+', rf'\g<1>{escala:.1f}',
                           texto, flags=re.MULTILINE)
        assert n == 1, nome
    with open(args.arquivo, 'w') as arquivo:
        arquivo.write(texto)
    print('Gravado. Recompile se nao usar --symlink-install, e reveja os limites das juntas.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
