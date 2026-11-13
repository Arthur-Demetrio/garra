#!/usr/bin/env python3
"""Calibra ao vivo a pose da webcam no robo usando os AprilTags dos cubos.

Abre a camera numa janela e, a cada imagem, usa os tags deitados no chao (tag para cima) para medir:
  - inclinacao da camera (graus abaixo da horizontal), pela normal dos tags;
  - altura da lente sobre o chao;
  - rolagem (imagem torta), so para conferir: o ideal e ficar perto de 0.
Na tela aparece o x/y/z (cm, robo_base) de cada cubo com esses valores.

A inclinacao e a altura saem dos tags; a posicao x/y da camera nao. Para acertar x/y, ponha um cubo
num ponto medido com a trena, aperte K e digite "x y" em cm (frente e esquerda do centro da base).

Teclas:  S salva em camera_montagem.json (o camera_cubo.py passa a usar)
         K corrige x/y da camera com um cubo medido     R zera as medidas     Q sai

Uso:  python3 scripts/calibrar_camera.py            (feche antes o camera_cubo.py e o Cheese)
"""

import argparse
import collections
import json
import math

import cv2
import numpy as np

import camera_cubo as cc

JANELA_LEITURAS = 150    # leituras de tag usadas na media (desliza no tempo)


class Calibracao:
    def __init__(self):
        self.montagem = dict(cc.MONTAGEM)
        self.normais = collections.deque(maxlen=JANELA_LEITURAS)
        self.tvecs = collections.deque(maxlen=JANELA_LEITURAS)
        self.mensagem = 'Deixe cubos com o tag para cima, sem nada na frente.'

    def adicionar(self, tags):
        for _, _, r, t in tags:
            self.normais.append(cv2.Rodrigues(r)[0][:, 2])
            self.tvecs.append(t)

    def estimar(self):
        """(inclinacao, rolagem, altura) pelos tags, ou None com poucas leituras."""
        if len(self.normais) < 10:
            return None
        N = np.array(self.normais)
        n = np.median(N, 0)
        n /= np.linalg.norm(n)
        bons = N @ n > math.cos(math.radians(15))     # descarta tag de lado / leitura espelhada
        n = N[bons].mean(0)
        n /= np.linalg.norm(n)
        incl = math.degrees(math.asin(-n[2]))
        rol = math.degrees(math.atan2(n[0], -n[1]))
        altura = float(np.median([cc.CUBO - t @ n for t in np.array(self.tvecs)[bons]]))
        return incl, rol, altura

    def montagem_atual(self):
        est = self.estimar()
        m = dict(self.montagem)
        if est:
            m['inclinacao'], m['z'] = round(est[0], 2), round(est[2], 4)
        return m


def desenhar(img, det, tags, cal):
    m = cal.montagem_atual()
    R, t = cc.rotacao_camera(m), np.array([m['x'], m['y'], m['z']])
    for i, c, r, tv in tags:
        p = cc.centro_do_cubo(r, tv, R, t)
        cv2.polylines(img, [c.astype(int)], True, (0, 255, 0), 3)
        cv2.drawFrameAxes(img, det.K, det.dist, r, tv, 0.03)
        org = tuple(int(v) for v in c[0] - [0, 14])
        cv2.putText(img, f'ID {i}  x={p[0] * 100:.1f} y={p[1] * 100:.1f} z={p[2] * 100:.1f} cm',
                    org, cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    est = cal.estimar()
    linhas = [f'Leituras: {len(cal.normais)}   tags agora: {len(tags)}']
    if est:
        incl, rol, alt = est
        linhas.append(f'Inclinacao {incl:.1f} graus (trena {cc.MONTAGEM["inclinacao"]:.0f})   '
                      f'Altura {alt * 100:.1f} cm (trena {cc.MONTAGEM["z"] * 100:.0f})   '
                      f'Rolagem {rol:+.1f} graus')
    else:
        linhas.append('Juntando leituras... (precisa de 10)')
    linhas.append(f'Camera em x={m["x"] * 100:.1f} y={m["y"] * 100:.1f} cm  | {cal.mensagem}')
    linhas.append('S salva   K corrige x/y com cubo medido   R zera   Q sai')
    h, w = img.shape[:2]
    cv2.rectangle(img, (0, 0), (w, 40 * len(linhas) + 10), (20, 20, 20), -1)
    for k, txt in enumerate(linhas):
        cv2.putText(img, txt, (14, 38 + 40 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (230, 230, 230), 2)


def corrigir_xy(cal, tags):
    if len(tags) != 1:
        cal.mensagem = 'Para K deixe exatamente UM cubo visivel.'
        return
    try:
        x, y = (float(v) / 100 for v in input('Posicao real do cubo "x y" em cm (frente, esquerda): ').split())
    except (ValueError, EOFError):
        cal.mensagem = 'Ignorado: digite dois numeros, ex: 46 -3'
        return
    m = cal.montagem_atual()
    _, _, r, tv = tags[0]
    p = cc.centro_do_cubo(r, tv, cc.rotacao_camera(m), np.array([m['x'], m['y'], m['z']]))
    cal.montagem['x'] += x - p[0]
    cal.montagem['y'] += y - p[1]
    cal.mensagem = f'x/y da camera corrigidos em ({(x - p[0]) * 100:+.1f}, {(y - p[1]) * 100:+.1f}) cm.'
    print(cal.mensagem, flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--camera', type=int, default=None, help='indice /dev/videoN; sem ele, procura a webcam')
    args = ap.parse_args()

    indice = cc.achar_webcam() if args.camera is None else args.camera
    cam = cc.abrir_camera(indice)
    ok, frame = cam.read()
    if not ok:
        raise SystemExit(f'Nao abriu a camera {indice} (feche o camera_cubo.py, o Cheese ou o navegador).')
    h, w = frame.shape[:2]
    print(f'Camera {indice}: {w}x{h}', flush=True)
    det = cc.DetectorTags(*cc.carregar_calibracao(w, h))
    cal = Calibracao()

    janela = 'Calibrar camera'
    cv2.namedWindow(janela, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(janela, 1280, 720)
    try:
        while True:
            ok, frame = cam.read()
            if not ok:
                print('A camera parou de mandar imagens.', flush=True)
                break
            tags = det.detectar(frame)
            cal.adicionar(tags)
            desenhar(frame, det, tags, cal)
            cv2.imshow(janela, frame)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q') or cv2.getWindowProperty(janela, cv2.WND_PROP_VISIBLE) < 1:
                break
            if k == ord('s'):
                m = cal.montagem_atual()
                cc.ARQ_MONTAGEM.write_text(json.dumps(m, indent=2))
                cal.mensagem = f'Salvo em {cc.ARQ_MONTAGEM.name}.'
                print(f'Salvo: {m}', flush=True)
            elif k == ord('r'):
                cal.normais.clear()
                cal.tvecs.clear()
                cal.mensagem = 'Medidas zeradas.'
            elif k == ord('k'):
                corrigir_xy(cal, tags)
    except KeyboardInterrupt:
        pass
    finally:
        cam.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
