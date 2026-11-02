
import rclpy

from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState


class TesteFK(Node):

    def __init__(self):
        super().__init__('teste_cinematica_direta')

        self.client = self.create_client(
            GetPositionFK,
            '/compute_fk'
        )

        self.juntas = [
            'Revolucionar_46',
            'Revolucionar_49',
            'Revolucionar_52'
        ]

    def calcular_fk(self, posicoes):

        request = GetPositionFK.Request()

        request.header.frame_id = 'base_link'
        request.fk_link_names = ['tool0']

        estado = RobotState()
        estado.is_diff = True

        estado.joint_state = JointState()
        estado.joint_state.name = self.juntas
        estado.joint_state.position = posicoes

        request.robot_state = estado

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            return None

        response = future.result()

        if response.error_code.val != 1:
            self.get_logger().error(
                f'FK falhou: código {response.error_code.val}'
            )
            return None

        return response.pose_stamped[0].pose.position

    def executar_teste(self):

        self.get_logger().info('Aguardando serviço FK...')

        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Serviço /compute_fk não encontrado.')
            return

        angulos = [0.0, 0.05, -0.05, 0.10, -0.10]

        origem = self.calcular_fk([0.0, 0.0, 0.0])

        if origem is None:
            return

        self.get_logger().info(
            f'HOME: X={origem.x:.6f}, '
            f'Y={origem.y:.6f}, Z={origem.z:.6f}'
        )

        for indice, nome in enumerate(self.juntas):

            self.get_logger().info(f'--- Testando {nome} ---')

            for angulo in angulos[1:]:

                posicoes = [0.0, 0.0, 0.0]
                posicoes[indice] = angulo

                resultado = self.calcular_fk(posicoes)

                if resultado is None:
                    continue

                dx = resultado.x - origem.x
                dy = resultado.y - origem.y
                dz = resultado.z - origem.z

                distancia = (dx**2 + dy**2 + dz**2)**0.5

                self.get_logger().info(
                    f'Ângulo: {angulo:+.3f} rad | '
                    f'X={resultado.x:.6f}, '
                    f'Y={resultado.y:.6f}, '
                    f'Z={resultado.z:.6f} | '
                    f'dX={dx*1000:+.3f} mm, '
                    f'dY={dy*1000:+.3f} mm, '
                    f'dZ={dz*1000:+.3f} mm | '
                    f'distância={distancia*1000:.3f} mm'
                )


def main():

    rclpy.init()
    node = TesteFK()

    try:
        node.executar_teste()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
