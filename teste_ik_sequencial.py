import rclpy

from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK


class TesteIKSequencial(Node):

    def __init__(self):
        super().__init__('teste_ik_sequencial')

        self.client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )

        self.joint_names = [
            'Revolucionar_46',
            'Revolucionar_49',
            'Revolucionar_52'
        ]

        # Configuração onde o cartesiano parou
        self.seed = [
            -0.021693668639288486,
            -0.03172569624184337,
            0.000000007020088
        ]

        self.x_inicial = 0.195866
        self.incremento = 0.0001
        self.quantidade = 25

    def executar(self):

        self.get_logger().info('Aguardando serviço IK...')

        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Serviço IK não encontrado!')
            return

        for i in range(1, self.quantidade + 1):

            x_destino = self.x_inicial + i * self.incremento

            request = GetPositionIK.Request()
            ik = request.ik_request

            ik.group_name = 'garra'
            ik.ik_link_name = 'tool0'

            ik.pose_stamped.header.frame_id = 'base_link'
            ik.pose_stamped.header.stamp = (
                self.get_clock().now().to_msg()
            )

            ik.pose_stamped.pose.position.x = x_destino
            ik.pose_stamped.pose.position.y = 0.005295
            ik.pose_stamped.pose.position.z = 0.184070

            # Orientação inicial
            ik.pose_stamped.pose.orientation.x = -0.349906
            ik.pose_stamped.pose.orientation.y = 0.635908
            ik.pose_stamped.pose.orientation.z = 0.563035
            ik.pose_stamped.pose.orientation.w = 0.395194

            ik.robot_state.is_diff = True

            ik.robot_state.joint_state.name = self.joint_names
            ik.robot_state.joint_state.position = self.seed

            ik.avoid_collisions = False

            ik.timeout.sec = 2

            future = self.client.call_async(request)

            rclpy.spin_until_future_complete(self, future)

            if future.result() is None:
                self.get_logger().error(
                    f'Falha na chamada para X={x_destino:.6f}'
                )
                return

            response = future.result()

            if response.error_code.val != 1:
                self.get_logger().warning(
                    f'IK falhou em X={x_destino:.6f}, '
                    f'erro={response.error_code.val}'
                )
                return

            solucao = response.solution.joint_state

            mapa = dict(zip(solucao.name, solucao.position))

            try:
                self.seed = [
                    mapa[nome] for nome in self.joint_names
                ]
            except KeyError as erro:
                self.get_logger().error(
                    f'Junta ausente na solução: {erro}'
                )
                return

            self.get_logger().info(
                f'X={x_destino:.6f} | '
                f'Juntas={self.seed}'
            )

        self.get_logger().info(
            'SUCESSO: todas as 25 posições foram resolvidas!'
        )


def main():

    rclpy.init()

    node = TesteIKSequencial()

    try:
        node.executar()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

