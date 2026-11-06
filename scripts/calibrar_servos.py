"""Mede zero, sentido e faixa de cada servo e grava o arquivo de calibracao.

Fale direto com o firmware: NAO rode com o ROS usando a mesma porta.

    python3 scripts/calibrar_servos.py --versao 180 [--porta /dev/ttyACM0]

Cuidado: ao selecionar uma junta, o servo vai direto ao pulso inicial (1500 us, ou o zero ja
medido). So as juntas selecionadas recebem sinal. Comece com o elo livre para nao bater, e
mantenha a mao na fonte. Os passos sao limitados a MAX_PASSO; a faixa e por sua conta: pare
antes de forcar a estrutura ou os cabos.
"""

import argparse
import math
import os
import select
import sys
import termios
import time


JUNTAS = ['Revolucionar_46', 'Revolucionar_49', 'Revolucionar_52']
MAX_PASSO = 50
PULSO_MIN = 500
PULSO_MAX = 2500
SAIDA_PADRAO = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'garra_moveit_config', 'config', 'calibracao_servos.yaml')

AJUDA = """Comandos:
  j <1|2|3>     seleciona a junta
  + [us] / - [us]  move o pulso (padrao 10 us, maximo por comando: {passo} us)
  zero          marca o pulso atual como a pose q=0 do CAD
  min / max     marca o pulso atual como limite inferior / superior de pulso
  sentido <+|-> + se o angulo da junta no modelo cresce quando o pulso cresce
  estado        mostra o que ja foi medido
  salvar        grava o arquivo (exige todas as juntas completas)
  sair""".format(passo=MAX_PASSO)


class Firmware:

    def __init__(self, porta):
        self.fd = os.open(porta, os.O_RDWR | os.O_NOCTTY)
        tty = termios.tcgetattr(self.fd)
        tty[0] = tty[1] = tty[3] = 0
        tty[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        tty[4] = tty[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, tty)
        self.buffer = b''

    def _linha(self, timeout):
        fim = time.monotonic() + timeout
        while b'\n' not in self.buffer:
            resto = fim - time.monotonic()
            if resto <= 0 or not select.select([self.fd], [], [], resto)[0]:
                return None
            self.buffer += os.read(self.fd, 128)
        linha, self.buffer = self.buffer.split(b'\n', 1)
        return linha.decode(errors='replace').strip()

    def conectar(self, timeout=6.0):
        # Abrir a porta reinicia o Mega. Enviar dados durante o boot pode prende-lo no bootloader.
        time.sleep(2.5)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        fim = time.monotonic() + timeout
        while time.monotonic() < fim:
            os.write(self.fd, b'V\n')
            if self._linha(0.3) == 'GARRA 1':
                return True
        return False

    def mover(self, pulsos):
        corpo = 'M ' + ' '.join(str(p) for p in pulsos)
        soma = 0
        for c in corpo.encode():
            soma ^= c
        os.write(self.fd, f'{corpo}*{soma:02X}\n'.encode())
        return self._linha(1.0)


def faltando(m):
    return [k for k in ('zero', 'min', 'max', 'sentido') if k not in m]


def gravar(caminho, medidas, us_por_rad, versao):
    linhas = [f'# Gerado por calibrar_servos.py (servo de {versao} graus).', 'juntas:']
    for nome in JUNTAS:
        m = medidas[nome]
        linhas.append(
            f'  {nome}: {{zero_us: {m["zero"]}, us_por_rad: {m["sentido"] * us_por_rad:.1f}, '
            f'min_us: {m["min"]}, max_us: {m["max"]}}}')
    with open(caminho, 'w') as arquivo:
        arquivo.write('\n'.join(linhas) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--versao', type=int, choices=[180, 270], required=True)
    parser.add_argument('--porta', default='/dev/ttyACM0')
    parser.add_argument('--saida', default=SAIDA_PADRAO)
    args = parser.parse_args()
    # 180 graus: faixa do Servo.write usada nas medicoes (544 a 2400 us).
    us_por_rad = (1856.0 if args.versao == 180 else 2000.0) / math.radians(args.versao)

    fw = Firmware(args.porta)
    if not fw.conectar():
        print('O firmware nao respondeu.')
        return 1
    print('Firmware conectado.\n' + AJUDA)

    medidas = {nome: {} for nome in JUNTAS}
    pulsos = [0, 0, 0]
    atual = None

    def enviar():
        resposta = fw.mover(pulsos)
        if resposta != 'K':
            print(f'Firmware respondeu: {resposta}')
        return resposta == 'K'

    for linha in sys.stdin:
        partes = linha.split()
        if not partes:
            continue
        cmd = partes[0]
        if cmd == 'sair':
            break
        if cmd == 'salvar':
            pendente = {n: faltando(medidas[n]) for n in JUNTAS if faltando(medidas[n])}
            invalidas = [n for n in JUNTAS if not pendente.get(n) and
                         not medidas[n]['min'] <= medidas[n]['zero'] <= medidas[n]['max']]
            if pendente:
                print(f'Faltam medidas, nada gravado: {pendente}')
            elif invalidas or any(medidas[n]['min'] >= medidas[n]['max'] for n in JUNTAS):
                print(f'Exige min < max e zero dentro da faixa, nada gravado: {invalidas}')
            else:
                gravar(args.saida, medidas, us_por_rad, args.versao)
                print(f'Gravado em {args.saida}. Recompile se nao usar --symlink-install.')
            continue
        if cmd == 'estado':
            for nome in JUNTAS:
                print(f'{nome}: {medidas[nome]}')
            continue
        if cmd == 'j':
            if len(partes) != 2 or partes[1] not in ('1', '2', '3'):
                print('Use: j 1, j 2 ou j 3')
                continue
            atual = int(partes[1]) - 1
            if pulsos[atual] == 0:
                pulsos[atual] = medidas[JUNTAS[atual]].get('zero', 1500)
                enviar()
            print(f'{JUNTAS[atual]}: pulso {pulsos[atual]} us')
            continue
        if atual is None:
            print('Selecione uma junta antes: j 1, j 2 ou j 3')
            continue
        m = medidas[JUNTAS[atual]]
        if cmd in ('+', '-'):
            try:
                passo = int(partes[1]) if len(partes) > 1 else 10
            except ValueError:
                print('Passo invalido.')
                continue
            if not 0 < passo <= MAX_PASSO:
                print(f'O passo deve estar entre 1 e {MAX_PASSO} us.')
                continue
            novo = pulsos[atual] + (passo if cmd == '+' else -passo)
            if not PULSO_MIN <= novo <= PULSO_MAX:
                print(f'Fora de {PULSO_MIN} a {PULSO_MAX} us.')
                continue
            pulsos[atual] = novo
            enviar()
            print(f'pulso {novo} us')
        elif cmd in ('zero', 'min', 'max'):
            m[cmd] = pulsos[atual]
            print(f'{cmd} = {pulsos[atual]} us')
        elif cmd == 'sentido' and len(partes) == 2 and partes[1] in ('+', '-'):
            m['sentido'] = 1 if partes[1] == '+' else -1
            print(f'sentido {partes[1]}')
        else:
            print(AJUDA)
    return 0


if __name__ == '__main__':
    sys.exit(main())
