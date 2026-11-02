
import rclpy

from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK


class TesteIK(Node):

    def __init__(self):
        super().__init__('teste_ik_direcoes')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )

    def executar_teste(self):

        self.get_logger().info('Aguardando serviço IK...')

        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Serviço /compute_ik não encontrado!')
            return

        limite = self.get_clock().now() + Duration(seconds=10.0)

        while rclpy.ok():
            if self.tf_buffer.can_transform(
                'base_link', 'tool0', rclpy.time.Time()
            ):
                break

            if self.get_clock().now() >= limite:
                self.get_logger().error('Transformação TF não encontrada.')
                return

            rclpy.spin_once(self, timeout_sec=0.1)

        transform = self.tf_buffer.lookup_transform(
            'base_link', 'tool0', rclpy.time.Time()
        )

        origem = transform.transform

        deslocamentos = [
            ('X +1 cm',  0.01,  0.00,  0.00),
            ('X -1 cm', -0.01,  0.00,  0.00),
            ('Y +1 cm',  0.00,  0.01,  0.00),
            ('Y -1 cm',  0.00, -0.01,  0.00),
            ('Z +1 cm',  0.00,  0.00,  0.01),
            ('Z -1 cm',  0.00,  0.00, -0.01),
        ]

        for nome, dx, dy, dz in deslocamentos:

            pose = PoseStamped()
            pose.header.frame_id = 'base_link'
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = origem.translation.x + dx
            pose.pose.position.y = origem.translation.y + dy
            pose.pose.position.z = origem.translation.z + dz

            pose.pose.orientation = origem.rotation

            request = GetPositionIK.Request()
            request.ik_request.group_name = 'garra'
            request.ik_request.ik_link_name = 'tool0'
            request.ik_request.pose_stamped = pose
            request.ik_request.timeout.sec = 2
            request.ik_request.timeout.nanosec = 0
            request.ik_request.avoid_collisions = False
            request.ik_request.robot_state.is_diff = True

            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is None:
                self.get_logger().error(f'{nome}: falha na chamada')
                continue

            response = future.result()
            codigo = response.error_code.val

            if codigo == 1:
                self.get_logger().info(f'{nome}: ALCANÇÁVEL (SUCCESS)')
            else:
                self.get_logger().warning(
                    f'{nome}: NÃO ALCANÇÁVEL (código {codigo})'
                )


def main():

    rclpy.init()
    node = TesteIK()

    try:
        node.executar_teste()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
