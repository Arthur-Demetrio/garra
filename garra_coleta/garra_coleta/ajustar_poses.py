#!/usr/bin/env python3
"""Ajusta as poses dos slots movendo a garra pelo terminal e grava em config/slots.yaml.

Os servos nao dao para mover a mao: cada junta anda por comando, em graus, e a pose atual
(/joint_states) e gravada no yaml quando estiver boa.

Precisa do MoveIt da garra rodando (garra_real.launch.py). Para mostrar o tag, rode tambem o
camera_cubo (o comando 't' le /camera/tags_vistos).

Uso:
  python3 -m garra_coleta.ajustar_poses [--velocidade 0.1]

Comandos:
  1 +5 / 2 -3 / 3 10    move a junta 1, 2 ou 3 esse tanto de graus (relativo)
  q 7 -54 50            vai para as juntas em graus (absoluto)
  ir <nome>             vai para uma pose do yaml: pronta, mostrar_apriltag, acima_slots, slot0..slot2
  a / f                 abre / fecha os dedos
  y <cm>                plataforma Y para y em cm (+ = esquerda)
  t                     IDs que a camera ve agora (um so = leitura boa)
  p                     mostra as juntas atuais
  g <nome>              grava as juntas atuais em <nome> (pronta, mostrar_apriltag, acima_slots,
                        slot0, slot1, slot2)
  x                     sai
"""

import argparse
import math
import re
import sys
import time
from pathlib import Path

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from std_msgs.msg import Float64, Int32MultiArray

from garra_coleta.cliente_garra import LIMITES, SUCESSO, ClienteGarra
from garra_coleta.slots import CAPACIDADE

ABERTA = 1.0471975511965976
FECHADA = 1.5707963267948966
PONTOS = ("pronta", "mostrar_apriltag", "acima_slots")
CONFIG = Path(__file__).resolve().parent.parent / "config" / "slots.yaml"


def gravar_no_yaml(texto, nome, q):
    """Troca a pose 'nome' no texto do slots.yaml (mantendo os comentarios). Devolve o texto novo."""
    valor = "[" + ", ".join(f"{v:.3f}" for v in q) + "]"
    if nome in PONTOS:
        padrao = re.compile(rf"^(  {nome}:)[^\n#]*", re.M)
        novo, n = padrao.subn(rf"\1 {valor}", texto)
    else:
        m = re.fullmatch(r"slot(\d+)", nome)
        if not m or int(m.group(1)) >= CAPACIDADE:
            raise ValueError(f"nome invalido: {nome}")
        padrao = re.compile(rf"^(  {m.group(1)}: \{{juntas: )[^,]*", re.M)
        novo, n = padrao.subn(rf"\g<1>{valor}", texto)
    if n != 1:
        raise ValueError(f"nao achei '{nome}' em {CONFIG.name}")
    return novo


def ler_do_yaml(nome):
    import yaml
    dados = yaml.safe_load(CONFIG.read_text(encoding="utf-8")) or {}
    if nome in PONTOS:
        return (dados.get("pontos") or {}).get(nome)
    m = re.fullmatch(r"slot(\d+)", nome)
    if m:
        return ((dados.get("slots") or {}).get(int(m.group(1))) or {}).get("juntas")
    raise ValueError(f"nome invalido: {nome}")


def graus(q):
    return "[" + ", ".join(f"{math.degrees(v):+.1f}" for v in q) + "] graus"


class Ajuste:
    def __init__(self, c, velocidade):
        self.c, self.v = c, velocidade
        self.dedos = ActionClient(c, GripperCommand, "/garra_dedos_controller/gripper_cmd")
        self.pub_y = c.create_publisher(Float64, "/garra/plataforma_y/alvo", 10)
        self.tags, self.t_tags = [], 0.0
        c.create_subscription(Int32MultiArray, "/camera/tags_vistos", self._tags, 10)

    def _tags(self, msg):
        self.tags, self.t_tags = list(msg.data), time.time()

    def mover(self, alvo):
        for i, (v, (lo, hi)) in enumerate(zip(alvo, LIMITES)):
            if not lo <= v <= hi:
                raise ValueError(f"junta {i + 1} = {math.degrees(v):.1f} graus fora do limite "
                                 f"[{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]")
        traj, codigo = self.c.planejar_juntas(self.c.juntas_atuais(), alvo, self.v)
        if codigo != SUCESSO:
            raise RuntimeError(f"planejamento falhou (codigo {codigo}): colisao ou fora do alcance?")
        codigo = self.c.executar(traj)
        if codigo != SUCESSO:
            raise RuntimeError(f"execucao falhou (codigo {codigo})")

    def mover_dedos(self, posicao):
        if not self.dedos.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("acao dos dedos indisponivel")
        objetivo = GripperCommand.Goal()
        objetivo.command.position = posicao
        envio = self.dedos.send_goal_async(objetivo)
        rclpy.spin_until_future_complete(self.c, envio)
        rclpy.spin_until_future_complete(self.c, envio.result().get_result_async())

    def ver_tags(self, janela=1.0):
        fim = time.time() + janela
        while time.time() < fim:
            rclpy.spin_once(self.c, timeout_sec=0.05)
        if time.time() - self.t_tags > janela:
            return "nada em /camera/tags_vistos (camera_cubo rodando?)"
        ids = sorted(set(self.tags))
        if len(ids) == 1:
            return f"le o tag {ids[0]}  OK"
        return f"ve {ids or 'nenhum tag'}"

    def comando(self, linha):
        p = linha.split()
        if not p:
            return True
        cmd = p[0].lower()
        if cmd == "x":
            return False
        if cmd in ("1", "2", "3"):
            q = self.c.juntas_atuais()
            q[int(cmd) - 1] += math.radians(float(p[1]))
            self.mover(q)
            print("  ->", graus(self.c.juntas_atuais()))
        elif cmd == "q":
            self.mover([math.radians(float(v)) for v in p[1:4]])
            print("  ->", graus(self.c.juntas_atuais()))
        elif cmd == "ir":
            q = ler_do_yaml(p[1])
            if q is None:
                raise ValueError(f"{p[1]} ainda e null no yaml")
            self.mover([float(v) for v in q])
            print("  ->", graus(self.c.juntas_atuais()))
        elif cmd == "a":
            self.mover_dedos(ABERTA)
        elif cmd == "f":
            self.mover_dedos(FECHADA)
        elif cmd == "y":
            self.pub_y.publish(Float64(data=float(p[1]) / 100.0))
            print("  alvo enviado; resultado em /garra/plataforma_y/resultado")
        elif cmd == "t":
            print(" ", self.ver_tags())
        elif cmd == "p":
            q = self.c.juntas_atuais()
            print(f"  {graus(q)}  =  [{', '.join(f'{v:.3f}' for v in q)}] rad")
        elif cmd == "g":
            q = self.c.juntas_atuais()
            CONFIG.write_text(gravar_no_yaml(CONFIG.read_text(encoding="utf-8"), p[1], q),
                              encoding="utf-8")
            print(f"  gravado {p[1]} = {graus(q)} em {CONFIG}")
        else:
            print("  comando desconhecido (veja o topo do arquivo)")
        return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--velocidade", type=float, default=0.1)
    args = ap.parse_args()
    rclpy.init()
    c = ClienteGarra("ajustar_poses")
    try:
        c.aguardar()
        aj = Ajuste(c, args.velocidade)
        print(__doc__.split("Comandos:")[1])
        print("atual:", graus(c.juntas_atuais()))
        while True:
            try:
                linha = input("pose> ")
            except EOFError:
                break
            try:
                if not aj.comando(linha):
                    break
            except Exception as exc:
                print(f"  ERRO: {exc}")
    except KeyboardInterrupt:
        pass
    finally:
        c.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
