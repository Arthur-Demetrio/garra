#!/usr/bin/env python3
"""Acha os cubos pelo AprilTag (36h11) com a webcam e manda a posicao para a garra.

Baseado na visao do repositorio niviton/-WORK (vision/cubes/detector.py e grasp.py):
  - camera_calib_webcam.yaml: calibracao da webcam em 1920x1080 (RMS 0,51 px), do -WORK;
  - o tag (quadrado preto de 32 mm) fica no topo do cubo de 42 mm. A posicao sai do PIXEL do centro
    do tag cruzado com a altura conhecida do topo do cubo (chao, ou --apoio para mesa). O metodo
    antigo, pela pose 3D do tag (solvePnP), fica em --pnp: a distancia dele sai ~11% maior;
  - a pose da camera no robo vem de ~/.config/garra/camera_montagem.json (calibrar_camera.py;
    recalibrada em 2026-11-16) ou, sem ele, do robot_config_webcam.yaml / MONTAGEM.
Funciona com o OpenCV 4.6 do ROS Jazzy (API antiga do aruco) e com o >= 4.7.

Frame robo_base: centro da base do robo no chao; X frente, Y esquerda, Z cima (m).
Publica geometry_msgs/Point em /pegar_cubo, que o pegar_cubo.py ja escuta (ou em --topico;
com /cubo_detectado o alcance_cubo.py decide entre a garra e mover a base).

Fluxo:
  terminal 1: ros2 launch ... garra_real.launch.py
  terminal 2: ros2 run garra_coleta pegar_cubo              (escuta /pegar_cubo)
  terminal 3: ros2 run garra_coleta camera_cubo             (aperte P para pegar)

Plataforma Y: com --tag-plataforma ID (e --altura-plataforma), esse tag nao e tratado como cubo;
a posicao dele (pelo pixel, no plano z = altura do tag) sai em /garra/plataforma_y/posicao a cada
quadro, para o plataforma_y.py fechar a malha.

Teclado (na janela): P manda o cubo escolhido   A liga/desliga envio automatico
                     0-9 escolhe o ID do cubo (o mais perto se nenhum)   Q sair

Uso:
  ros2 run garra_coleta camera_cubo --sem-ros            (so ver a camera e as coordenadas)
  ros2 run garra_coleta camera_cubo --id 3          (so o cubo com tag 3)
"""

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml



def pasta_config():
    """config/ do pacote: GARRA_CONFIG, a pasta do codigo-fonte ou o share/ instalado."""
    if os.environ.get('GARRA_CONFIG'):
        return Path(os.environ['GARRA_CONFIG'])
    fonte = Path(__file__).resolve().parents[1] / 'config'
    if fonte.is_dir():
        return fonte
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('garra_coleta')) / 'config'


CONFIG = pasta_config()
CALIB = CONFIG / 'camera_calib_webcam.yaml'
TOPICO = '/pegar_cubo'
TOPICO_PLATAFORMA = '/garra/plataforma_y/posicao'
TAG = 0.032              # m, lado do quadrado PRETO do AprilTag
CUBO = 0.042             # m, aresta do cubo
Z_CHAO = CUBO / 2        # centro do cubo apoiado no chao
PARADO_S = 1.0           # envio automatico: cubo parado por este tempo
PARADO_M = 0.005         # m de tremido aceito como parado
MOVEU_M = 0.02           # envio automatico: so manda de novo se o cubo andou mais que isso
Y_PLANO = -0.029         # m: plano da garra (o braco ainda e planar)
Y_TOLERANCIA = 0.015

# Camera no meio, 44 cm atras do centro da base, lente a 54 cm do chao, 41 graus para baixo,
# olhando para a frente do robo. (Os tags no -WORK deram 39,4 graus: se o x sair deslocado,
# confira a inclinacao primeiro.)
MONTAGEM = dict(x=-0.44, y=0.0, z=0.54, inclinacao=41.0, guinada=0.0)
# Valores calibrados pelo calibrar_camera.py (tecla S) substituem os da trena.
# Fica fora do pacote para valer sem recompilar e sobreviver ao colcon build.
ARQ_MONTAGEM = Path(os.environ.get('GARRA_MONTAGEM',
                                   Path.home() / '.config' / 'garra' / 'camera_montagem.json'))
MONTAGEM_CALIBRADA = dict(MONTAGEM, **json.loads(ARQ_MONTAGEM.read_text())) if ARQ_MONTAGEM.exists() else None
# Pose calibrada no -WORK (extrinsic_height: tags na mesa + altura 57 cm). O frame dela tem origem no
# chao LOGO ABAIXO da camera (x = para onde ela aponta); somando CAMERA_EM vira robo_base.
ARQ_GIT = CONFIG / 'robot_config_webcam.yaml'
CAMERA_EM = np.array([MONTAGEM['x'], MONTAGEM['y'], 0.0])   # ponto sob a camera, em robo_base


def pose_camera():
    """(R, t, origem) da camera em robo_base: calibrar_camera.py > -WORK > trena."""
    if MONTAGEM_CALIBRADA:
        m = MONTAGEM_CALIBRADA
        return rotacao_camera(m), np.array([m['x'], m['y'], m['z']]), 'calibrar_camera.py'
    if ARQ_GIT.exists():
        T = np.array(yaml.safe_load(ARQ_GIT.read_text())['T_base_cam'], float)
        return T[:3, :3], T[:3, 3] + CAMERA_EM, ARQ_GIT.name + ' (-WORK)'
    m = MONTAGEM
    return rotacao_camera(m), np.array([m['x'], m['y'], m['z']]), 'trena'


def rotacao_camera(m):
    """Colunas = eixos do frame optico (x direita, y baixo, z frente) escritos em robo_base."""
    p, g = math.radians(m['inclinacao']), math.radians(m['guinada'])
    frente = [math.cos(p) * math.cos(g), math.cos(p) * math.sin(g), -math.sin(p)]
    direita = [math.sin(g), -math.cos(g), 0.0]
    baixo = [-math.sin(p) * math.cos(g), -math.sin(p) * math.sin(g), -math.cos(p)]
    return np.array([direita, baixo, frente]).T


def carregar_calibracao(largura, altura):
    d = yaml.safe_load(CALIB.read_text())
    K = np.array(d['camera_matrix'], float)
    dist = np.array(d['dist_coeffs'], float)
    if largura != d['image_width']:
        K[:2] *= largura / d['image_width']   # mesma proporcao, outra resolucao
    return K, dist


class DetectorTags:
    """AprilTag 36h11 com refinamento subpixel (mesmos parametros do -WORK)."""

    def __init__(self, K, dist):
        self.K, self.dist = K, dist
        s = TAG / 2
        self.obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
        dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        novo = hasattr(cv2.aruco, 'ArucoDetector')
        par = cv2.aruco.DetectorParameters() if novo else cv2.aruco.DetectorParameters_create()
        par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        par.cornerRefinementWinSize = 5
        par.minMarkerPerimeterRate = 0.03
        par.adaptiveThreshWinSizeMin = 5
        par.adaptiveThreshWinSizeMax = 35
        par.adaptiveThreshWinSizeStep = 15
        if novo:
            det = cv2.aruco.ArucoDetector(dic, par)
            self._detectar = det.detectMarkers
        else:
            self._detectar = lambda g: cv2.aruco.detectMarkers(g, dic, parameters=par)
        # Bits 6x6 de cada ID nas 4 rotacoes, para o resgate pela etiqueta branca
        n = len(dic.bytesList)
        bits = np.array([dic.getBitsFromByteList(dic.bytesList[i:i + 1], 6) for i in range(n)])
        self.codigos = np.stack([np.rot90(bits, k, axes=(1, 2)) for k in range(4)], 1).reshape(n, 4, 36)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def resgatar(self, cinza, quad, celula=12):
        """Le um candidato rejeitado como a ETIQUETA BRANCA inteira (10x10 celulas: margem branca,
        borda preta, 6x6 bits). Acontece com o tag no topo do cubo visto de lado: as celulas ficam
        com ~6 px de altura e o detector so acha o contorno da etiqueta. Devolve (id, cantos) ou None."""
        if cv2.contourArea(quad) < 400:
            return None
        S = 10 * celula
        H = cv2.getPerspectiveTransform(quad, np.float32([[0, 0], [S, 0], [S, S], [0, S]]))
        w = cv2.warpPerspective(cinza, H, (S, S))
        _, b = cv2.threshold(w, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        m = celula // 4
        grade = np.array([[b[y * celula + m:(y + 1) * celula - m, x * celula + m:(x + 1) * celula - m].mean() > 0.5
                           for x in range(10)] for y in range(10)], np.uint8)
        anel = lambda k: np.concatenate([grade[k, k:10 - k], grade[9 - k, k:10 - k],
                                         grade[k:10 - k, k], grade[k:10 - k, 9 - k]])
        if anel(0).mean() < 0.8 or anel(1).mean() > 0.2:      # margem branca e borda preta
            return None
        erros = (self.codigos != grade[2:8, 2:8].ravel()).sum(2)
        i, k = np.unravel_index(erros.argmin(), erros.shape)
        if erros[i, k] > 1:
            return None
        # Cantos do quadrado preto (1 celula para dentro), na ordem do OpenCV para a rotacao k
        dentro = np.float32([[[celula, celula], [9 * celula, celula], [9 * celula, 9 * celula], [celula, 9 * celula]]])
        c = cv2.perspectiveTransform(dentro, np.linalg.inv(H)).reshape(4, 2)
        c = np.roll(c, k, axis=0).astype(np.float32)
        cv2.cornerSubPix(cinza, c, (3, 3), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
        return int(i), c

    def detectar(self, frame):
        """Lista de (id, cantos 4x2, rvec, tvec) no frame optico da camera."""
        cinza = self.clahe.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        cantos, ids, rejeitados = self._detectar(cinza)
        cantos = list(cantos) if ids is not None else []
        ids = list(ids.ravel()) if ids is not None else []
        for c in rejeitados:
            achado = self.resgatar(cinza, c.reshape(4, 2).astype(np.float32))
            if achado and achado[0] not in ids:
                ids.append(achado[0])
                cantos.append(achado[1])
        saida = []
        if not ids:
            return saida
        for c, i in zip(cantos, ids):
            c = c.reshape(4, 2).astype(np.float32)
            ok, rvec, tvec = cv2.solvePnP(self.obj, c, self.K, self.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                saida.append((int(i), c, rvec.ravel(), tvec.ravel()))
        return saida


def centro_pelo_pixel(cantos, K, dist, R_base_cam, t_base_cam, apoio=0.0):
    """Centro do cubo em robo_base pelo PIXEL do centro do tag e pela altura conhecida do topo do cubo.

    O tag fica no centro da face de cima: o raio da camera pelo centro do tag (cruzamento das
    diagonais, correto em perspectiva) encontra o plano z = apoio + CUBO no centro dessa face, e o
    centro do cubo fica meia aresta abaixo. So depende da pose calibrada da camera; nao usa a
    distancia do solvePnP, que em 2026-11-16 saiu ~11% maior que a real (erro de varios cm em x/y).
    apoio: altura da superficie onde o cubo esta (0 = chao; mesa: altura do tampo).
    """
    topo = ponto_no_plano(cantos, K, dist, R_base_cam, t_base_cam, apoio + CUBO)
    if topo is None:
        return None
    return np.array([topo[0], topo[1], apoio + CUBO / 2])


def ponto_no_plano(cantos, K, dist, R_base_cam, t_base_cam, z):
    """Centro do tag em robo_base, sabendo que ele esta no plano horizontal de altura z (m)."""
    p0, p1, p2, p3 = np.asarray(cantos, float).reshape(4, 2)
    d1, d2 = p2 - p0, p3 - p1                       # diagonais 0-2 e 1-3
    s = np.cross(p1 - p0, d2) / np.cross(d1, d2)
    centro = p0 + s * d1
    n = cv2.undistortPoints(centro.reshape(1, 1, 2), K, dist).ravel()
    raio = R_base_cam @ np.array([n[0], n[1], 1.0])
    if raio[2] >= 0:
        return None                                  # raio nao desce ate o plano (acima do horizonte)
    return t_base_cam + raio * (z - t_base_cam[2]) / raio[2]


def centro_do_cubo(rvec, tvec, R_base_cam, t_base_cam):
    """Centro do cubo em robo_base pela pose 3D do tag (solvePnP): meia aresta atras da face do tag.

    Metodo antigo (--pnp): mede o z, mas a distancia sai ~11% maior; prefira centro_pelo_pixel."""
    R_tag, _ = cv2.Rodrigues(rvec)
    centro_cam = tvec + R_tag @ np.array([0.0, 0.0, -CUBO / 2])
    return R_base_cam @ centro_cam + t_base_cam


class Publicador:
    def __init__(self, sem_ros, topico=TOPICO):
        self.no = None
        self.topico = topico
        if sem_ros:
            return
        import rclpy
        from geometry_msgs.msg import Point
        rclpy.init()
        self.rclpy, self.Point = rclpy, Point
        self.no = rclpy.create_node('camera_cubo')
        self.pub = self.no.create_publisher(Point, topico, 10)
        self.pub_plataforma = self.no.create_publisher(Point, TOPICO_PLATAFORMA, 10)

    def ativo(self):
        return self.no is None or self.rclpy.ok()

    def enviar(self, x, y, z):
        if self.no is not None and self.rclpy.ok():
            if not self.pub.get_subscription_count():
                print(f'AVISO: ninguem escutando {self.topico}', flush=True)
            self.pub.publish(self.Point(x=x, y=y, z=z))
            self.rclpy.spin_once(self.no, timeout_sec=0.0)

    def plataforma(self, x, y, z):
        if self.no is not None and self.rclpy.ok():
            self.pub_plataforma.publish(self.Point(x=x, y=y, z=z))
            self.rclpy.spin_once(self.no, timeout_sec=0.0)

    def fechar(self):
        if self.no is not None:
            self.no.destroy_node()
            if self.rclpy.ok():
                self.rclpy.shutdown()


def achar_webcam(nome='Web Camera'):
    """Menor /dev/videoN com esse nome (o numero muda quando a webcam e reconectada)."""
    for d in sorted(Path('/sys/class/video4linux').glob('video*'), key=lambda d: int(d.name[5:])):
        if nome in (d / 'name').read_text():
            return int(d.name[5:])
    return 0


def abrir_camera(indice):
    cam = cv2.VideoCapture(indice, cv2.CAP_V4L2)
    # 1080p a 60 Hz: a maioria das webcams so chega la em MJPG.
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cam.set(cv2.CAP_PROP_FPS, 60)
    cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cam


class App:
    def __init__(self, args):
        self.args = args
        self.pub = Publicador(args.sem_ros, args.topico)
        self.R, self.t, origem = pose_camera()
        incl = math.degrees(math.asin(-self.R[2, 2]))
        print(f'Pose da camera: {origem}; lente em {np.round(self.t * 100, 1)} cm, '
              f'{incl:.1f} graus para baixo', flush=True)
        self.id_alvo = args.id
        self.auto = args.auto
        self.ultimo_enviado = None
        self.parado = (None, 0.0)       # (posicao, desde quando)
        self.mensagem = 'P envia o cubo, A automatico, 0-9 escolhe o ID, Q sai.'

    def escolher(self, cubos):
        if not cubos:
            return None
        if self.id_alvo is not None:
            return next((c for c in cubos if c['id'] == self.id_alvo), None)
        return min(cubos, key=lambda c: c['xyz'][0])   # o mais perto da base

    def posicao_final(self, cubo):
        x, y, z = (float(v) for v in cubo['xyz'])
        # pixel: z ja e o da superficie informada; pnp: z medido so com --z-medido.
        if self.args.pnp and not self.args.z_medido:
            z = Z_CHAO
        return x, y, z

    def enviar(self, cubo):
        x, y, z = self.posicao_final(cubo)
        texto = f'Cubo {cubo["id"]}: x={x:.3f} y={y:.3f} z={z:.3f} (robo_base)'
        if abs(y - Y_PLANO) > Y_TOLERANCIA:
            texto += f'  AVISO: fora do plano da garra (y~{Y_PLANO:.3f}); y e ignorado'
        print(('Enviado ' if not self.args.sem_ros else '[sem ROS] ') + texto, flush=True)
        self.pub.enviar(x, y, z)
        self.ultimo_enviado = (x, y)
        self.mensagem = 'Enviado: ' + texto

    def automatico(self, cubo):
        if cubo is None:
            self.parado = (None, 0.0)
            return
        xy, agora = cubo['xyz'][:2], time.time()
        if self.parado[0] is None or np.linalg.norm(xy - self.parado[0]) > PARADO_M:
            self.parado = (xy, agora)
            return
        longe = self.ultimo_enviado is None or math.dist(xy, self.ultimo_enviado) > MOVEU_M
        if self.auto and agora - self.parado[1] >= PARADO_S and longe:
            self.enviar(cubo)

    def desenhar(self, img, cubos, alvo, det, fps):
        for c in cubos:
            cor = (0, 255, 0) if c is alvo else (0, 200, 255)
            cv2.polylines(img, [c['cantos'].astype(int)], True, cor, 3)
            cv2.drawFrameAxes(img, det.K, det.dist, c['rvec'], c['tvec'], 0.03)
            x, y, z = c['xyz']
            org = tuple(int(v) for v in c['cantos'][0] - [0, 14])
            xc, yc = x - CAMERA_EM[0], y - CAMERA_EM[1]
            cv2.putText(img, f'ID {c["id"]} base: x={x * 100:.1f} y={y * 100:.1f} z={z * 100:.1f}  '
                        f'| sob camera: x={xc * 100:.1f} y={yc * 100:.1f} cm',
                        org, cv2.FONT_HERSHEY_SIMPLEX, 0.9, cor, 2)
        h, w = img.shape[:2]
        cv2.rectangle(img, (0, h - 50), (w, h), (20, 20, 20), -1)
        alvo_txt = 'mais perto' if self.id_alvo is None else f'ID {self.id_alvo}'
        cv2.putText(img, f'[{"AUTO" if self.auto else "MANUAL"}] alvo: {alvo_txt}  {fps:.0f} fps  '
                    f'{"SEM ROS" if self.args.sem_ros else self.args.topico}  | {self.mensagem}',
                    (12, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (230, 230, 230), 2)

    def rodar(self):
        if self.args.camera is None:
            self.args.camera = achar_webcam()
        cam = abrir_camera(self.args.camera)
        ok, frame = cam.read()
        if not ok:
            print(f'Nao abriu a camera {self.args.camera} (ocupada por outro programa, como Cheese '
                  'ou o navegador? ou outro indice: --camera 0, 2...)', flush=True)
            return
        h, w = frame.shape[:2]
        print(f'Camera {self.args.camera}: {w}x{h} @ {cam.get(cv2.CAP_PROP_FPS):.0f} fps pedidos',
              flush=True)
        det = DetectorTags(*carregar_calibracao(w, h))
        janela = 'Camera - AprilTag'
        cv2.namedWindow(janela, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(janela, 1280, 720)
        t_ant, fps = time.time(), 0.0
        try:
            while True:
                if not self.pub.ativo():
                    break                         # Ctrl+C / ROS encerrado
                ok, frame = cam.read()
                if not ok:
                    print('A camera parou de mandar imagens.', flush=True)
                    break
                agora = time.time()
                fps = 0.9 * fps + 0.1 / max(agora - t_ant, 1e-3)
                t_ant = agora

                cubos = []
                for i, c, r, t in det.detectar(frame):
                    if i == self.args.tag_plataforma:
                        p = ponto_no_plano(c, det.K, det.dist, self.R, self.t, self.args.altura_plataforma)
                        if p is not None:
                            self.pub.plataforma(*(float(v) for v in p))
                            cv2.polylines(frame, [c.astype(int)], True, (255, 0, 255), 3)
                            cv2.putText(frame, f'PLATAFORMA y={p[1] * 100:.1f} cm',
                                        tuple(int(v) for v in c[0] - [0, 14]),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)
                        continue
                    xyz = (centro_do_cubo(r, t, self.R, self.t) if self.args.pnp else
                           centro_pelo_pixel(c, det.K, det.dist, self.R, self.t, self.args.apoio))
                    if xyz is not None:
                        cubos.append(dict(id=i, cantos=c, rvec=r, tvec=t, xyz=xyz))
                alvo = self.escolher(cubos)
                self.automatico(alvo)
                self.desenhar(frame, cubos, alvo, det, fps)
                cv2.imshow(janela, frame)

                k = cv2.waitKey(1) & 0xFF
                if k == ord('q') or cv2.getWindowProperty(janela, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if k == ord('p'):
                    if alvo is None:
                        self.mensagem = 'Nenhum cubo alvo visivel.'
                    else:
                        self.enviar(alvo)
                elif k == ord('a'):
                    self.auto, self.ultimo_enviado = not self.auto, None
                    self.mensagem = 'Automatico LIGADO.' if self.auto else 'Automatico desligado.'
                elif ord('0') <= k <= ord('9'):
                    n = k - ord('0')
                    self.id_alvo = None if self.id_alvo == n else n
                    self.mensagem = f'Alvo: ID {n}.' if self.id_alvo is not None else 'Alvo: mais perto.'
        except KeyboardInterrupt:
            pass
        finally:
            cam.release()
            cv2.destroyAllWindows()
            self.pub.fechar()



def argumentos_sem_ros():
    """sys.argv sem os --ros-args que o ros2 launch/run acrescentam."""
    try:
        from rclpy.utilities import remove_ros_args
        return remove_ros_args(sys.argv)[1:]
    except ImportError:
        return sys.argv[1:]

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--camera', type=int, default=None,
                    help='indice /dev/videoN; sem ele, procura a "Web Camera"')
    ap.add_argument('--id', type=int, default=None, help='ID do AprilTag do cubo; sem ele, o mais perto')
    ap.add_argument('--apoio', type=float, default=0.0,
                    help='altura (m) da superficie onde estao os cubos: 0 = chao (padrao); mesa = tampo')
    ap.add_argument('--pnp', action='store_true',
                    help='metodo antigo: posicao pela pose 3D do tag (distancia ~11%% maior)')
    ap.add_argument('--z-medido', action='store_true',
                    help=f'com --pnp: usa o z medido em vez do cubo no chao (z={Z_CHAO})')
    ap.add_argument('--auto', action='store_true',
                    help='ja comeca enviando sozinho (cubo parado 1 s e que andou > 2 cm)')
    ap.add_argument('--topico', default=TOPICO,
                    help=f'onde publicar (padrao {TOPICO}; use /cubo_detectado com o alcance_cubo.py)')
    ap.add_argument('--tag-plataforma', type=int, default=None,
                    help=f'ID do AprilTag preso na plataforma Y; publica a posicao em {TOPICO_PLATAFORMA}')
    ap.add_argument('--altura-plataforma', type=float, default=0.20,
                    help='altura (m) do tag da plataforma em relacao ao chao (medir com a trena)')
    ap.add_argument('--sem-ros', action='store_true', help='nao publica no ROS, so mostra/imprime')
    App(ap.parse_args(argumentos_sem_ros())).rodar()


if __name__ == '__main__':
    main()
