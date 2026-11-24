#!/usr/bin/env python3
"""Interface ROS 2 para o estoque de tres slots da garra.

Comandos em std_msgs/String em /garra/slots/comando:
  {"acao":"status"}
  {"acao":"reset"}
  {"acao":"guardar","cubo_id":"12"}
  {"acao":"retirar","cubo_id":"12"}
  {"acao":"retirar","slot":2}
"""

import argparse
import json
import signal
import sys
from pathlib import Path

import rclpy
import yaml
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from garra_coleta.cliente_garra import ClienteGarra, SUCESSO
from garra_coleta.slots import GerenciadorSlots


TOPICO_COMANDO = "/garra/slots/comando"
TOPICO_ESTADO = "/garra/slots/estado"
TOPICO_EVENTO = "/garra/slots/evento"
ABERTA = 1.0471975511965976
FECHADA = 1.5707963267948966


def carregar_config():
    caminho = Path(__file__).resolve().parent.parent / "config" / "slots.yaml"
    with caminho.open("r", encoding="utf-8") as arquivo:
        return yaml.safe_load(arquivo) or {}


class GerenciadorSlotsNode:
    def __init__(self, no, cliente, velocidade=0.2):
        self.no = no
        self.cliente = cliente
        self.estoque = GerenciadorSlots()
        self.velocidade = velocidade
        self.config = carregar_config()
        self.dedos = ActionClient(
            no, GripperCommand, "/garra_dedos_controller/gripper_cmd"
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_estado = no.create_publisher(String, TOPICO_ESTADO, qos)
        self.pub_evento = no.create_publisher(String, TOPICO_EVENTO, 10)
        no.create_subscription(String, TOPICO_COMANDO, self._comando, 10)

    def publicar_estado(self, extra=None):
        dados = self.estoque.estado().para_dict()
        if extra:
            dados.update(extra)
        self.pub_estado.publish(String(data=json.dumps(dados, sort_keys=True)))

    def publicar_evento(self, tipo, **dados):
        dados["tipo"] = tipo
        self.pub_evento.publish(String(data=json.dumps(dados, sort_keys=True)))

    def pose(self, nome):
        pontos = self.config.get("pontos", {})
        if nome not in pontos:
            raise RuntimeError(f"ponto {nome} nao configurado")
        valor = pontos[nome]
        if valor is None:
            raise RuntimeError(
                f"ponto {nome} ainda nao foi calibrado em config/slots.yaml"
            )
        if not isinstance(valor, list) or len(valor) != 3:
            raise RuntimeError(f"ponto {nome} deve ser uma lista com tres juntas")
        return [float(v) for v in valor]

    def ir_para(self, nome):
        alvo = self.pose(nome)
        traj, codigo = self.cliente.planejar_juntas(
            self.cliente.juntas_atuais(), alvo, self.velocidade
        )
        if codigo != SUCESSO:
            raise RuntimeError(f"falha ao planejar {nome} (codigo {codigo})")
        codigo = self.cliente.executar(traj)
        if codigo != SUCESSO:
            raise RuntimeError(f"falha ao executar {nome} (codigo {codigo})")

    def mover_dedos(self, posicao):
        if not self.dedos.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("acao dos dedos indisponivel")
        objetivo = GripperCommand.Goal()
        objetivo.command.position = posicao
        envio = self.dedos.send_goal_async(objetivo)
        rclpy.spin_until_future_complete(self.no, envio)
        resultado = envio.result().get_result_async()
        rclpy.spin_until_future_complete(self.no, resultado)
        return resultado.result().result

    def _comando(self, msg):
        try:
            dados = json.loads(msg.data)
            acao = dados.get("acao")
            if acao == "status":
                self.publicar_estado()
                return
            if acao == "reset":
                estado = self.estoque.reset()
                self.publicar_estado()
                self.publicar_evento("reset", slots=estado.para_dict()["slots"])
                return
            if acao == "guardar":
                cubo_id = str(dados["cubo_id"])
                slot = self.estoque.guardar(cubo_id)
                try:
                    self.ir_para("mostrar_apriltag")
                    self.ir_para("acima_slots")
                    self.ir_para(f"slot_{slot}")
                    self.mover_dedos(ABERTA)
                    self.ir_para("pronta")
                except Exception:
                    self.estoque.retirar_por_slot(slot)
                    raise
                self.publicar_estado()
                self.publicar_evento("guardado", cubo_id=cubo_id, slot=slot)
                return
            if acao == "retirar":
                if "slot" in dados:
                    slot = int(dados["slot"])
                    cubo_id = self.estoque.retirar_por_slot(slot)
                else:
                    cubo_id = str(dados["cubo_id"])
                    slot = self.estoque.retirar_por_cubo(cubo_id)
                try:
                    self.ir_para("mostrar_apriltag")
                    self.ir_para("acima_slots")
                    self.ir_para(f"slot_{slot}")
                    self.mover_dedos(FECHADA)
                    self.ir_para("pronta")
                except Exception:
                    self.estoque.slots[slot] = cubo_id
                    raise
                self.publicar_estado()
                self.publicar_evento("retirado", cubo_id=cubo_id, slot=slot)
                return
            raise ValueError(f"acao invalida: {acao}")
        except Exception as exc:
            self.publicar_estado({"erro": str(exc)})
            self.publicar_evento("erro", mensagem=str(exc))


def argumentos_sem_ros():
    try:
        from rclpy.utilities import remove_ros_args
        return remove_ros_args(sys.argv)[1:]
    except ImportError:
        return sys.argv[1:]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--velocidade", type=float, default=0.2)
    args = ap.parse_args(argumentos_sem_ros())

    rclpy.init()
    c = ClienteGarra("gerenciador_slots")
    try:
        c.aguardar()
        GerenciadorSlotsNode(c, c, args.velocidade)
        print(
            f"Gerenciador ativo: {TOPICO_COMANDO}; estado em {TOPICO_ESTADO}",
            flush=True,
        )
        while rclpy.ok():
            rclpy.spin_once(c, timeout_sec=0.1)
    except KeyboardInterrupt:
        return 0
    except RuntimeError as exc:
        print(f"ERRO: {exc}", file=sys.stderr)
        return 1
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        c.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
