
import rclpy

from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath


class TesteCartesiano(Node):

    def __init__(self):
        super().__init__('teste_cartesiano')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        self.client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )

    def executar_teste(self):

        self.get_logger().info('Aguardando serviço cartesiano...')

        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Serviço não encontrado!')
            return

        self.get_logger().info('Obtendo pose atual do tool0...')
        self.get_logger().info('Aguardando transformações TF...')

        tempo_limite = self.get_clock().now() + Duration(
            seconds=10.0
        )

        while rclpy.ok():

            if self.tf_buffer.can_transform(
                'base_link',
                'tool0',
                rclpy.time.Time()
            ):
                break

            if self.get_clock().now() >= tempo_limite:
                self.get_logger().error(
                    'Não foi possível encontrar base_link -> tool0.'
                )
                return

            rclpy.spin_once(self, timeout_sec=0.1)

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'tool0',
                rclpy.time.Time()
            )

        except Exception as erro:
            self.get_logger().error(f'Erro no TF: {erro}')
            return

        # Obtém a pose atual do TCP
        pose = Pose()

        pose.position.x = transform.transform.translation.x + 0.003

        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z

        pose.orientation = transform.transform.rotation

        # Monta a solicitação cartesiana
        request = GetCartesianPath.Request()

        request.header.frame_id = 'base_link'
        request.header.stamp = self.get_clock().now().to_msg()

        # Utiliza o estado atual do robô
        request.start_state.is_diff = True

	    # Mantém a posição atual do TCP
        # Obtém a pose atual do TCP e adiciona 10 mm em X
        pose = Pose()

        pose.position.x = transform.transform.translation.x + 0.005
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z

        pose.orientation = transform.transform.rotation

        # Monta a solicitação cartesiana
        request = GetCartesianPath.Request()

        request.header.frame_id = 'base_link'
        request.header.stamp = self.get_clock().now().to_msg()

        # Utiliza o estado atual do robô
        request.start_state.is_diff = True

        request.group_name = 'garra'
        request.link_name = 'tool0'

        request.waypoints = [pose]

        request.max_step = 0.0001
        request.jump_threshold = 0.0

        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = 0.0

        # Desativado temporariamente para diagnóstico
        request.avoid_collisions = False

        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        self.get_logger().info(
            f'Destino: X={pose.position.x:.4f}, '
            f'Y={pose.position.y:.4f}, '
            f'Z={pose.position.z:.4f}'
        )

        self.get_logger().info('Calculando trajetória...')

        future = self.client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error(
                'Falha ao chamar o serviço.'
            )
            return

        response = future.result()

        self.get_logger().info(
            f'Fração calculada: {response.fraction * 100:.1f}%'
        )

        self.get_logger().info(
            f'Código de erro: {response.error_code.val}'
        )

        self.get_logger().info(
            f'Mensagem do MoveIt: {response.error_code.message}'
        )

        self.get_logger().info(
            f'Origem do erro: {response.error_code.source}'
        )

        trajectory = response.solution.joint_trajectory
        self.get_logger().info(
            f'Quantidade de pontos cartesianos: {len(trajectory.points)}'
        )

        if trajectory.points:
            self.get_logger().info(
                f'Duração total: {trajectory.points[-1].time_from_start.sec}s '
                f'{trajectory.points[-1].time_from_start.nanosec}ns'
            )

        self.get_logger().info(
            f'Pontos da trajetória: {len(trajectory.points)}'
        )

        self.get_logger().info(
            f'Juntas na trajetória: {trajectory.joint_names}'
        )

        if trajectory.points:
            self.get_logger().info(
                f'Primeiro ponto: {trajectory.points[0].positions}'
            )

            self.get_logger().info(
                f'Último ponto: {trajectory.points[-1].positions}'
            )

        if response.fraction >= 0.999 and response.error_code.val == 1:
            self.get_logger().info(
                'SUCESSO: trajetória cartesiana calculada!'
            )
        else:
            self.get_logger().warning(
                'A trajetória não foi calculada completamente.'
            )


def main():

    rclpy.init()

    node = TesteCartesiano()

    try:
        node.executar_teste()

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
