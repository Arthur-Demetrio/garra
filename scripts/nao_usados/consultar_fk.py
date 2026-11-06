import rclpy

from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState


class ConsultarFK(Node):

    def __init__(self):
        super().__init__('consultar_fk')

        self.client = self.create_client(
            GetPositionFK,
            '/compute_fk'
        )

    def executar(self):

        self.get_logger().info('Aguardando serviço FK...')

        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Serviço não encontrado!')
            return

        request = GetPositionFK.Request()

        request.header.frame_id = 'base_link'
        request.fk_link_names = ['tool0']

        # Configuração final do teste de +5 mm
        joint_state = JointState()

        joint_state.name = [
            'Revolucionar_46',
            'Revolucionar_49',
            'Revolucionar_52'
        ]

        joint_state.position = [
            -0.04372419296170685,
            -0.06428771595118739,
            -0.000005718891711
        ]

        request.robot_state.joint_state = joint_state

        self.get_logger().info('Calculando FK...')

        future = self.client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('Falha na consulta FK.')
            return

        response = future.result()

        self.get_logger().info(
            f'Código de erro: {response.error_code.val}'
        )

        if response.error_code.val != 1:
            self.get_logger().error(
                f'Falha: {response.error_code.message}'
            )
            return

        pose = response.pose_stamped[0].pose

        self.get_logger().info(
            f'Posição do tool0: '
            f'X={pose.position.x:.6f}, '
            f'Y={pose.position.y:.6f}, '
            f'Z={pose.position.z:.6f}'
        )

        self.get_logger().info(
            f'Orientação: '
            f'X={pose.orientation.x:.6f}, '
            f'Y={pose.orientation.y:.6f}, '
            f'Z={pose.orientation.z:.6f}, '
            f'W={pose.orientation.w:.6f}'
        )


def main():

    rclpy.init()

    node = ConsultarFK()

    try:
        node.executar()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
