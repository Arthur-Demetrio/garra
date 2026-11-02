
import rclpy
import numpy as np

from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState


class TesteJacobiana(Node):

    def __init__(self):
        super().__init__('teste_jacobiana')

        self.client = self.create_client(
            GetPositionFK,
            '/compute_fk'
        )

        self.joint_names = [
            'Revolucionar_46',
            'Revolucionar_49',
            'Revolucionar_52'
        ]

        # Altere esta configuração para testar outras posições
        self.q = [0.0, 0.0, 0.0]

        self.delta = 0.0001
        self.frame_id = 'base_link'
        self.link_name = 'tool0'

    def calcular_fk(self, q):

        request = GetPositionFK.Request()

        request.header.frame_id = self.frame_id
        request.fk_link_names = [self.link_name]

        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = q

        request.robot_state.joint_state = joint_state

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            raise RuntimeError('Falha ao chamar /compute_fk')

        response = future.result()

        if response.error_code.val != 1:
            raise RuntimeError(
                f'Erro no FK: {response.error_code.val}'
            )

        return response.pose_stamped[0].pose

    def quaternion_to_matrix(self, q):

        x, y, z, w = q.x, q.y, q.z, q.w

        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])

    def calcular_jacobiana(self):

        pose_central = self.calcular_fk(self.q)

        posicao = np.array([
            pose_central.position.x,
            pose_central.position.y,
            pose_central.position.z
        ])

        # Orientação atual do tool0 em relação ao base_link
        R0 = self.quaternion_to_matrix(
            pose_central.orientation
        )

        J = np.zeros((6, 3))
        for i in range(3):

            q_menos = self.q.copy()
            q_mais = self.q.copy()

            q_menos[i] -= self.delta
            q_mais[i] += self.delta

            pose_menos = self.calcular_fk(q_menos)
            pose_mais = self.calcular_fk(q_mais)

            p_menos = np.array([
                pose_menos.position.x,
                pose_menos.position.y,
                pose_menos.position.z
            ])

            p_mais = np.array([
                pose_mais.position.x,
                pose_mais.position.y,
                pose_mais.position.z
            ])

            J[:3, i] = (
                p_mais - p_menos
            ) / (2 * self.delta)

            R_menos = self.quaternion_to_matrix(
                pose_menos.orientation
            )

            R_mais = self.quaternion_to_matrix(
                pose_mais.orientation
            )

            R_relativa = R_mais @ R_menos.T

            w = np.array([
                R_relativa[2, 1] - R_relativa[1, 2],
                R_relativa[0, 2] - R_relativa[2, 0],
                R_relativa[1, 0] - R_relativa[0, 1]
            ]) / (4 * self.delta)

            J[3:, i] = w

        R_local = R0.T

        J_local = np.zeros((6, 3))

        J_local[:3, :] = R_local @ J[:3, :]
        J_local[3:, :] = R_local @ J[3:, :]

        return posicao, J, J_local

    def analisar(self):

        self.get_logger().info(
            'Aguardando serviço /compute_fk...'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Aguardando /compute_fk...'
            )

        self.get_logger().info(
            'Calculando Jacobiana e espaço nulo...'
        )

        posicao, J, J_local = self.calcular_jacobiana()

        J_pos = J[:3, :]
        J_ang = J[3:, :]

        # Decomposição em valores singulares
        U, S, Vt = np.linalg.svd(J_pos)

        tolerancia = max(J_pos.shape) * np.finfo(float).eps * S[0]
        posto = np.sum(S > tolerancia)

        print('\n========== ANÁLISE DO ESPAÇO NULO ==========')

        print(f'\nConfiguração das juntas: {self.q}')

        print('\nPosição do tool0:')
        print(f'  X = {posicao[0]:.6f} m')
        print(f'  Y = {posicao[1]:.6f} m')
        print(f'  Z = {posicao[2]:.6f} m')

        print('\nPosto da Jacobiana translacional:')
        print(f'  {posto}')

        print('\nValores singulares:')
        for valor in S:
            print(f'  {valor:.10f}')

        # Espaço nulo: J_pos * q_dot = 0
        espaco_nulo = Vt[posto:, :].T

        print('\nDimensão do espaço nulo:')
        print(f'  {espaco_nulo.shape[1]}')

        if espaco_nulo.shape[1] > 0:

            print('\nCombinações de juntas que preservam a posição:')

            for i in range(espaco_nulo.shape[1]):

                vetor = espaco_nulo[:, i]
                velocidade_angular = J_ang @ vetor

                print(f'\n  Solução {i + 1}:')
                print(f'  Velocidades das juntas: {vetor}')
                print(
                    '  Velocidade angular do TCP (rad/s): '
                    f'{velocidade_angular}'
                )

        else:

            print(
                '\nNão existe combinação não nula de velocidades '
                'das juntas que mantenha a posição do TCP parada '
                'instantaneamente nesta configuração.'
            )

        # Direção de junta que produz o menor deslocamento
        # translacional para uma velocidade articular unitária.
        vetor_fraco = Vt[-1, :]
        velocidade_linear = J_pos @ vetor_fraco
        velocidade_angular = J_ang @ vetor_fraco

        print('\n========== MOVIMENTO MENOS TRANSLACIONAL ==========')

        print(
            'Direção articular de menor efeito translacional:'
        )
        print(f'  {vetor_fraco}')

        print('\nMovimento linear resultante:')
        print(f'  {velocidade_linear} m/s')

        print('\nMovimento angular resultante:')
        print(f'  {velocidade_angular} rad/s')

        print(
            '\nMagnitude da velocidade linear: '
            f'{np.linalg.norm(velocidade_linear):.8f} m/s'
        )

        print(
            'Magnitude da velocidade angular: '
            f'{np.linalg.norm(velocidade_angular):.8f} rad/s'
        )

        print('\n===============================================\n')
       
        print('\n========== JACOBIANA LOCAL (tool0) ==========')

        print('\nJacobiana translacional local (m/rad):')

        for linha in J_local[:3]:
            print(
                '  [' +
                ', '.join(f'{v: .6f}' for v in linha) +
                ' ]'
            )

        print('\nJacobiana angular local (rad/rad):')

        for linha in J_local[3:]:
            print(
                '  [' +
                ', '.join(f'{v: .6f}' for v in linha) +
                ' ]'
            )

        print('\nEfeito da terceira junta no referencial local:')

        print(f'  X: {J_local[0, 2]:.6f} m/rad')
        print(f'  Y: {J_local[1, 2]:.6f} m/rad')
        print(f'  Z: {J_local[2, 2]:.6f} m/rad')

        print('\n============================================')


def main(args=None):

    rclpy.init(args=args)

    node = TesteJacobiana()

    try:
        node.analisar()
    except Exception as e:
        node.get_logger().error(str(e))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
