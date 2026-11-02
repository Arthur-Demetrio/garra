import rclpy

from rclpy.node import Node
from moveit_msgs.srv import GetCartesianPath
from geometry_msgs.msg import Pose


class TesteCartesianoContinuacao(Node):

    def __init__(self):
        super().__init__('teste_cartesiano_continuacao')

        self.client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )

    def executar(self):

        self.get_logger().info('Aguardando serviço cartesiano...')

        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Serviço não encontrado!')
            return

        request = GetCartesianPath.Request()

        request.header.frame_id = 'base_link'
        request.header.stamp = self.get_clock().now().to_msg()

        # Configuração exata onde o planejamento anterior parou
        request.start_state.is_diff = True

        request.start_state.joint_state.name = [
            'Revolucionar_46',
            'Revolucionar_49',
            'Revolucionar_52'
        ]

        
        request.start_state.joint_state.position = [
            0.0,
            0.0,
            0.0
        ]
        

        request.group_name = 'garra'
        request.link_name = 'tool0'

        # Pose inicial conhecida
        x_inicial = 0.193366

        y = 0.005295
        z = 0.184070

        orientacao = [
            -0.349906,
             0.635908,
             0.563035,
             0.395194
        ]

        waypoints = []

        # 25 passos de 0,1 mm
        for i in range(1, 51):

            pose = Pose()

            pose.position.x = x_inicial + i * 0.0001
            pose.position.y = y
            pose.position.z = z

            pose.orientation.x = orientacao[0]
            pose.orientation.y = orientacao[1]
            pose.orientation.z = orientacao[2]
            pose.orientation.w = orientacao[3]

            waypoints.append(pose)

        request.waypoints = waypoints

        request.max_step = 0.0001
        request.jump_threshold = 0.0

        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = 0.0

        request.avoid_collisions = False

        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        self.get_logger().info(
            'Calculando continuação com 25 waypoints...'
        )

        future = self.client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('Falha na chamada.')
            return

        response = future.result()

        self.get_logger().info(
            f'Fração calculada: {response.fraction * 100:.1f}%'
        )

        self.get_logger().info(
            f'Código de erro: {response.error_code.val}'
        )

        trajectory = response.solution.joint_trajectory

        self.get_logger().info(
            f'Pontos gerados: {len(trajectory.points)}'
        )

        if trajectory.points:
            self.get_logger().info(
                f'Primeiro ponto: {trajectory.points[0].positions}'
            )

            self.get_logger().info(
                f'Último ponto: {trajectory.points[-1].positions}'
            )

        if response.fraction >= 0.999:
            self.get_logger().info(
                'SUCESSO: continuação cartesiana completa!'
            )
        else:
            self.get_logger().warning(
                'A continuação cartesiana foi parcial.'
            )


def main():

    rclpy.init()

    node = TesteCartesianoContinuacao()

    try:
        node.executar()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
