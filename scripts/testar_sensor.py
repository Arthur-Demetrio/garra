#!/usr/bin/env python3
"""Testa a chave fim de curso dos dedos direto pela serial (sem ROS).

Pergunta "S" ao firmware a cada 0,1 s e mostra o estado: aperte e solte a chave para ver mudar.
Feche antes o garra_real.launch.py (so um programa usa a porta por vez).

Uso:  python3 scripts/testar_sensor.py [/dev/ttyUSB0]      Ctrl+C para sair
"""

import glob
import os
import sys
import termios
import time


def abrir(porta):
    fd = os.open(porta, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = a[1] = a[3] = 0                                      # modo cru
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[4] = a[5] = termios.B115200
    a[6][termios.VMIN] = a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    return fd


def linhas(fd, buf):
    try:
        buf += os.read(fd, 256)
    except BlockingIOError:
        pass
    *prontas, resto = buf.split(b'\n')
    return [p.strip().decode(errors='replace') for p in prontas], resto


def main():
    portas = sys.argv[1:] or sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    if not portas:
        sys.exit('Nenhuma porta serial encontrada (Arduino conectado?).')
    porta = portas[0]
    fd = abrir(porta)
    print(f'{porta}: esperando o Arduino reiniciar...', flush=True)
    time.sleep(4.0)                 # abrir a porta reinicia o Mega (2,5 s nao bastou)
    termios.tcflush(fd, termios.TCIOFLUSH)

    buf = b''
    os.write(fd, b'V\n')
    time.sleep(0.3)
    resp, buf = linhas(fd, buf)
    print('Firmware:', ', '.join(resp) or '(sem resposta)', flush=True)

    anterior, n = None, 0
    try:
        while True:
            os.write(fd, b'S\n')
            time.sleep(0.1)
            resp, buf = linhas(fd, buf)
            for r in resp:
                if r == 'E comando':
                    sys.exit('Firmware nao conhece o comando S: grave o garra_servos.ino novo.')
                if r in ('S 0', 'S 1'):
                    n += 1
                    if r != anterior:
                        estado = 'APERTADA (cubo preso)' if r == 'S 1' else 'solta (sem cubo)'
                        print(f'{time.strftime("%H:%M:%S")}  chave {estado}', flush=True)
                        anterior = r
    except KeyboardInterrupt:
        print(f'\n{n} leituras.')
    finally:
        os.close(fd)


if __name__ == '__main__':
    main()
