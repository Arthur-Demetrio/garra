import rclpy

from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK


class ConsultarIK(Node):

    def __init__(self):
        super().__init__('consultar_ik')

        self.client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )

    def executar(self):

        self.get_logger().info('Aguardando serviço IK...')

        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Serviço IK não encontrado!')
            return

        request = GetPositionIK.Request()
        ik = request.ik_request

        # Grupo e elo que queremos resolver
        ik.group_name = 'garra'
        ik.ik_link_name = 'tool0'

        # Referencial do destino
        ik.pose_stamped.header.frame_id = 'base_link'
        ik.pose_stamped.header.stamp = self.get_clock().now().to_msg()

        # Destino do teste: deslocamento de +5 mm em X
        ik.pose_stamped.pose.position.x = 0.195966
        ik.pose_stamped.pose.position.y = 0.005295
        ik.pose_stamped.pose.position.z = 0.184070

        # Manter orientação inicial
        ik.pose_stamped.pose.orientation.x = -0.349906
        ik.pose_stamped.pose.orientation.y = 0.635908
        ik.pose_stamped.pose.orientation.z = 0.563035
        ik.pose_stamped.pose.orientation.w = 0.395194

        # Configuração inicial das juntas
        ik.robot_state.is_diff = True

        ik.robot_state.joint_state.name = [
            'Revolucionar_46',
            'Revolucionar_49',
            'Revolucionar_52'
        ]

        ik.robot_state.joint_state.position = [
            -0.021693668639288486,
            -0.03172569624184287,
            0.000000007020088
        ]

        # Mesmo critério de colisão usado no teste cartesiano
        ik.avoid_collisions = False

        # Tempo disponível para o solver
        ik.timeout.sec = 2
        ik.timeout.nanosec = 0

        self.get_logger().info(
            'Solicitando solução IK para X=0.198366...'
        )

        future = self.client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('Falha na chamada IK.')
            return

        response = future.result()

        self.get_logger().info(
            f'Código de erro: {response.error_code.val}'
        )

        self.get_logger().info(
            f'Mensagem: {response.error_code.message}'
        )

        self.get_logger().info(
            f'Origem: {response.error_code.source}'
        )

        if response.error_code.val == 1:

            joints = response.solution.joint_state

            self.get_logger().info('SOLUÇÃO IK ENCONTRADA!')

            for nome, posicao in zip(
                joints.name,
                joints.position
            ):
                self.get_logger().info(
                    f'{nome}: {posicao:.9f} rad'
                )

        else:
            self.get_logger().warning(
                'O solver não encontrou uma solução IK.'
            )


def main():

    rclpy.init()

    node = ConsultarIK()

    try:
        node.executar()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
