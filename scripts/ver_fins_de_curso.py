#!/usr/bin/env python3
"""Mostra ao vivo o estado dos fins de curso da plataforma Y (pinos 17 (esquerda) e 14 (direita) do Mega).

Precisa do firmware/teste_fins_de_curso gravado. Nao usa pyserial (so a biblioteca padrao).
Feche o driver ROS e o Monitor Serial antes: a porta so pode ter um dono.

Uso:  python3 ver_fins_de_curso.py [/dev/ttyUSB0]
"""

import os
import sys
import termios
import time

porta = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
fd = os.open(porta, os.O_RDWR | os.O_NOCTTY)
attrs = termios.tcgetattr(fd)
attrs[0] = attrs[1] = attrs[3] = 0                       # iflag, oflag, lflag: modo cru
attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # 8N1
attrs[4] = attrs[5] = termios.B115200
attrs[6][termios.VMIN], attrs[6][termios.VTIME] = 0, 2   # read volta em ate 0,2 s
termios.tcsetattr(fd, termios.TCSANOW, attrs)
termios.tcflush(fd, termios.TCIFLUSH)

print(f'Lendo {porta} (Ctrl+C para sair). Aperte uma chave de cada vez.')
buf, ultimo, ultimo_dado, recebidas = b'', None, time.time(), 0
try:
    while True:
        dados = os.read(fd, 256)
        if dados:
            ultimo_dado = time.time()
            buf += dados
        elif time.time() - ultimo_dado > 3:
            print('\r(nenhum dado ha 3 s: firmware de teste gravado? porta certa?)', end='', flush=True)
            ultimo_dado = time.time()
        while b'\n' in buf:
            linha, buf = buf.split(b'\n', 1)
            linha = linha.decode(errors='replace').strip()
            partes = linha.split()
            if len(partes) != 3 or partes[0] != 'FC':
                if linha:
                    print(f'\n[arduino] {linha}')
                continue
            estado = (partes[1], partes[2])
            recebidas += 1
            txt = lambda v: 'APERTADA' if v == '1' else 'solta   '
            if estado != ultimo and ultimo is not None:
                print()  # mudou: guarda a linha anterior no historico
            ultimo = estado
            print(f'\r{time.strftime("%H:%M:%S")}  esquerda (17): {txt(estado[0])}   direita (14): {txt(estado[1])}'
                  f'   (msgs recebidas: {recebidas})', end='', flush=True)
except KeyboardInterrupt:
    print()
finally:
    os.close(fd)
