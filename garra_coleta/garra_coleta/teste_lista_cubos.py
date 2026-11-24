#!/usr/bin/env python3
"""Teste: a câmera registra os cubos (IDs de AprilTag) numa lista dinâmica.

Escuta /camera/tags_vistos (publicado pelo camera_cubo). Cada ID novo entra na lista
e é guardado no GerenciadorSlots (menor slot livre). Não move a garra.

  Terminal 1: ros2 run garra_coleta camera_cubo
  Terminal 2: python3 teste_lista_cubos.py
"""

import rclpy
from std_msgs.msg import Int32MultiArray

from garra_coleta.slots import GerenciadorSlots


def main():
    rclpy.init()
    no = rclpy.create_node("teste_lista_cubos")
    vistos = []                       # lista dinâmica, na ordem em que apareceram
    estoque = GerenciadorSlots(4)

    def ao_receber(msg):
        for cubo_id in msg.data:
            if cubo_id in vistos:
                continue
            vistos.append(cubo_id)
            try:
                slot = estoque.guardar(cubo_id)
                no.get_logger().info(f"novo cubo {cubo_id} -> slot {slot}")
            except Exception as erro:  # slots cheios etc.
                no.get_logger().warn(f"novo cubo {cubo_id}, sem slot: {erro}")
            no.get_logger().info(f"lista={vistos}  slots={estoque.estado().slots}")

    no.create_subscription(Int32MultiArray, "/camera/tags_vistos", ao_receber, 10)
    no.get_logger().info("aguardando tags em /camera/tags_vistos ...")
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\nFinal: lista={vistos}  slots={estoque.estado().slots}")
        no.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
