
import sys

import rclpy
from rclpy.node import Node

from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState


class VerificarPontoFinal(Node):

    def __init__(self):
        super().__init__('verificar_ponto_final')

        self.client = self.create_client(
            GetPositionFK,
            '/compute_fk'
        )

    def executar(self):
        self.get_logger().info('Aguardando serviço /compute_fk...')

        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'Serviço /compute_fk não encontrado.'
            )
            return

        request = GetPositionFK.Request()

        # Referencial no qual queremos calcular a posição
        request.header.frame_id = 'base_link'

        # Link cuja posição queremos descobrir
        request.fk_link_names = ['tool0']

        # Estado das juntas da última trajetória cartesiana
        estado = RobotState()

        estado.joint_state = JointState()

        estado.joint_state.name = [
            'Revolucionar_46',
            'Revolucionar_49',
            'Revolucionar_52'
        ]

        estado.joint_state.position = [
            1.9909577071743478e-05,
            -8.587206625383718e-05,
            -2.8971467539639844e-05
        ]

        request.robot_state = estado

        self.get_logger().info(
            'Calculando posição final do tool0...'
        )

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error(
                'Falha ao calcular a cinemática direta.'
            )
            return

        resposta = future.result()

        if resposta.error_code.val != 1:
            self.get_logger().error(
                f'Erro no MoveIt: {resposta.error_code.val}'
            )
            return

        if not resposta.pose_stamped:
            self.get_logger().error(
                'Nenhuma pose foi retornada.'
            )
            return

        pose = resposta.pose_stamped[0].pose

        print('\n========== RESULTADO FK ==========')

        print('\nPosição do tool0:')
        print(f'X = {pose.position.x:.6f} m')
        print(f'Y = {pose.position.y:.6f} m')
        print(f'Z = {pose.position.z:.6f} m')

        print('\nOrientação (quaternion):')
        print(f'x = {pose.orientation.x:.6f}')
        print(f'y = {pose.orientation.y:.6f}')
        print(f'z = {pose.orientation.z:.6f}')
        print(f'w = {pose.orientation.w:.6f}')

        print('\n==================================\n')


def main(args=None):
    rclpy.init(args=args)

    node = VerificarPontoFinal()

    try:
        node.executar()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

