"""Mede zero, sentido e faixa de cada servo e grava o arquivo de calibracao.

Fale direto com o firmware: NAO rode com o ROS usando a mesma porta.

    python3 scripts/calibrar_servos.py --versao 180 [--porta /dev/ttyUSB0]

Cuidado: ao abrir a porta o Arduino reinicia e os tres servos vao para 90 graus (1472 us) na
velocidade maxima. Ao selecionar uma junta, ela vai ao zero ja medido, se houver. O comando <motor>,<angulo> (padrao do firmware do time) leva o servo direto ao
angulo; o firmware limita a velocidade (~25 graus/s), mas nao o destino. Comece com o elo livre para nao bater, e
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


JUNTAS = ['Revolucionar_46', 'Revolucionar_49', 'Revolucionar_72']
MAX_PASSO = 50
PULSO_MIN = 500
PULSO_MAX = 2500
# Conversao do Servo.write, a mesma do comando manual do firmware.
SERVO_WRITE_MIN = 544
SERVO_WRITE_MAX = 2400
US_POR_GRAU = (SERVO_WRITE_MAX - SERVO_WRITE_MIN) / 180
PULSO_INICIAL = SERVO_WRITE_MIN + round(90 * US_POR_GRAU)  # 1472 us
GRAUS_POR_S = 25  # velocidade do firmware: 1 grau a cada 40 ms
SAIDA_PADRAO = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'garra_moveit_config', 'config', 'calibracao_servos.yaml')

AJUDA = """Comandos:
  <motor>,<angulo>  seleciona a junta e leva ao angulo de 0 a 180 (ex.: 1,90), como no
                    Monitor Serial. Salta direto: confira o caminho antes
  j <1|2|3>     seleciona a junta
  + [us] / - [us]  ajuste fino do pulso (padrao 10 us ~ 1 grau, maximo por comando: {passo} us)
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


def graus(pulso):
    return (pulso - SERVO_WRITE_MIN) / US_POR_GRAU


def descrever(pulso):
    return f'{pulso} us (~{graus(pulso):.1f} graus)'


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
    parser.add_argument('--porta', default='/dev/ttyUSB0')
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
        if ',' in linha:
            texto = linha.replace(' ', '').strip()
            motor, _, angulo = texto.partition(',')
            if motor not in ('1', '2', '3') or not angulo.isdigit():
                print('Use: <motor>,<angulo>, ex.: 1,90 (motor 1 a 3)')
                continue
            if int(angulo) > 180:
                print('Angulo invalido! Digite entre 0 e 180.')
                continue
            atual = int(motor) - 1
            antes = pulsos[atual]
            pulsos[atual] = SERVO_WRITE_MIN + round(int(angulo) * US_POR_GRAU)
            if enviar():
                t = abs(pulsos[atual] - antes) / US_POR_GRAU / GRAUS_POR_S if antes else 0
                print(f'-> Motor {motor} ({JUNTAS[atual]}) indo para {descrever(pulsos[atual])}'
                      + (f', ~{t:.1f} s' if t >= 0.5 else ''))
            else:
                pulsos[atual] = antes
            continue
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
            for i, nome in enumerate(JUNTAS):
                m = medidas[nome]
                campos = [f'{k} {descrever(m[k])}' for k in ('zero', 'min', 'max') if k in m]
                if 'sentido' in m:
                    campos.append('sentido ' + ('+' if m['sentido'] > 0 else '-'))
                marca = '>' if i == atual else ' '
                print(f'{marca} {i + 1} {nome}: ' + (', '.join(campos) or 'nada medido'))
            continue
        if cmd == 'j':
            if len(partes) != 2 or partes[1] not in ('1', '2', '3'):
                print('Use: j 1, j 2 ou j 3')
                continue
            atual = int(partes[1]) - 1
            if pulsos[atual] == 0:
                pulsos[atual] = medidas[JUNTAS[atual]].get('zero', PULSO_INICIAL)
                enviar()
            print(f'{JUNTAS[atual]}: pulso {descrever(pulsos[atual])}')
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
            print(f'pulso {descrever(novo)}')
        elif cmd in ('zero', 'min', 'max'):
            m[cmd] = pulsos[atual]
            print(f'{cmd} = {descrever(pulsos[atual])}')
        elif cmd == 'sentido' and len(partes) == 2 and partes[1] in ('+', '-'):
            m['sentido'] = 1 if partes[1] == '+' else -1
            print(f'sentido {partes[1]}')
        else:
            print(AJUDA)
    return 0


if __name__ == '__main__':
    sys.exit(main())
