"""Calcula uma reta do TCP no plano do braco (X e Z), mantendo a inclinacao, sem executar.

Uso, com o MoveIt rodando:
    python3 scripts/teste_cartesiano.py [dx dy dz]   (metros; padrao: -0.005 0 0)

Deslocamento em Y nao conclui: o braco nao sai do plano.
"""

import sys

import rclpy

from cliente_garra import ClienteGarra


def main():
    deslocamento = tuple(float(v) for v in sys.argv[1:4]) or (-0.005, 0.0, 0.0)
    rclpy.init()
    c = ClienteGarra('teste_cartesiano')
    try:
        c.aguardar()
        q = c.juntas_atuais()
        inicio = c.fk(q).position
        print(f'Juntas atuais: {[round(v, 4) for v in q]}')
        print(f'TCP atual: X={inicio.x:.4f} Y={inicio.y:.4f} Z={inicio.z:.4f}')
        print(f'Deslocamento pedido: {deslocamento}')

        res = c.linha_cartesiana(q, deslocamento)
        print(f'Fracao calculada: {res.fracao * 100:.1f}% {res.mensagem}')
        if res.trajetoria is None:
            return 1
        print(f'Desvio maximo da reta: {res.desvio_max * 1000:.3f} mm')
        print(f'Maior salto de junta: {res.salto_max:.3f} rad')
        pontos = res.trajetoria.joint_trajectory.points
        print(f'Pontos: {len(pontos)}, ultimo: {[round(v, 4) for v in pontos[-1].positions]}')
        if res.fracao < 0.999:
            print('A reta nao foi concluida.')
            return 1
        print('SUCESSO: reta calculada.')
        return 0
    finally:
        c.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
