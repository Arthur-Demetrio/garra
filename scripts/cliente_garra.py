"""Cliente dos servicos do MoveIt para a garra (grupo 'garra', TCP 'tool0').

O braco e planar, com tres eixos paralelos: controla X, Z e a inclinacao da garra. O Y do TCP e
fixo. Toda pose pedida precisa manter esse Y e girar apenas em torno do eixo das juntas.
"""

from dataclasses import dataclass
import math

from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
)
from moveit_msgs.srv import (
    GetCartesianPath,
    GetMotionPlan,
    GetPositionFK,
    GetPositionIK,
    GetStateValidity,
)
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


JUNTAS = ['Revolucionar_46', 'Revolucionar_49', 'Revolucionar_72']
# Iguais aos do URDF (scripts/ajustar_urdf.py), que vem da calibracao dos servos.
LIMITES = [(-0.79, 1.50), (-1.47, 1.35), (-1.25, 1.30)]
GRUPO = 'garra'
TCP = 'tool0'
BASE = 'base_link'

HOME = [0.0, 0.0, 0.0]
# Pose de espera: TCP ~25 cm do chao, 20 cm a frente da base da garra, inclinado 45 graus para baixo,
# no meio da regiao de coleta e com folga nas juntas. A coleta planeja pelas juntas daqui ate a
# pre-pegada; as retas a partir dela sao curtas (2 a 5 cm).
PRONTA = [0.12, -0.95, 0.87]

SUCESSO = MoveItErrorCodes.SUCCESS


@dataclass
class ResultadoLinha:
    trajetoria: object
    fracao: float
    desvio_max: float
    salto_max: float
    mensagem: str = ''


class ClienteGarra(Node):

    def __init__(self, nome='cliente_garra'):
        super().__init__(nome)
        self._fk = self.create_client(GetPositionFK, '/compute_fk')
        self._ik = self.create_client(GetPositionIK, '/compute_ik')
        self._valid = self.create_client(GetStateValidity, '/check_state_validity')
        self._cart = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._plan = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self._exec = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self._estado = None
        self.create_subscription(JointState, '/joint_states', self._ao_estado, 10)

    def _ao_estado(self, msg):
        self._estado = dict(zip(msg.name, msg.position))

    def aguardar(self, timeout=30.0):
        """Espera os servicos do move_group; levanta RuntimeError se nao aparecerem."""
        for cliente in (self._fk, self._ik, self._valid, self._cart, self._plan):
            if not cliente.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f'Servico {cliente.srv_name} indisponivel. O MoveIt esta rodando?')
        if not self._exec.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('Acao /execute_trajectory indisponivel.')

    def _chamar(self, cliente, request):
        future = cliente.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            raise RuntimeError(f'Falha ao chamar {cliente.srv_name}')
        return future.result()

    def _estado_robo(self, q, msg):
        msg.joint_state.name = JUNTAS
        msg.joint_state.position = [float(v) for v in q]

    def juntas_atuais(self, timeout=5.0):
        self._estado = None
        fim = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while self._estado is None and self.get_clock().now().nanoseconds < fim:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._estado is None:
            raise RuntimeError('Nada publicado em /joint_states.')
        return [self._estado[j] for j in JUNTAS]

    def fk(self, q):
        """Pose do TCP em base_link para a configuracao q."""
        req = GetPositionFK.Request()
        req.header.frame_id = BASE
        req.fk_link_names = [TCP]
        self._estado_robo(q, req.robot_state)
        res = self._chamar(self._fk, req)
        if res.error_code.val != SUCESSO:
            raise RuntimeError(f'FK falhou (codigo {res.error_code.val})')
        return res.pose_stamped[0].pose

    def ik(self, pose, semente, evitar_colisao=True, timeout=0.5):
        """Juntas que levam o TCP a pose, ou None se nao houver solucao."""
        req = GetPositionIK.Request()
        ik = req.ik_request
        ik.group_name = GRUPO
        ik.ik_link_name = TCP
        ik.avoid_collisions = evitar_colisao
        ik.timeout.sec = int(timeout)
        ik.timeout.nanosec = int((timeout % 1) * 1e9)
        self._estado_robo(semente, ik.robot_state)
        alvo = PoseStamped()
        alvo.header.frame_id = BASE
        alvo.pose = pose
        ik.pose_stamped = alvo
        res = self._chamar(self._ik, req)
        if res.error_code.val != SUCESSO:
            return None
        valores = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [valores[j] for j in JUNTAS]

    def valido(self, q):
        """(sem colisao?, pares em contato) para a configuracao q."""
        req = GetStateValidity.Request()
        req.group_name = GRUPO
        self._estado_robo(q, req.robot_state)
        res = self._chamar(self._valid, req)
        return res.valid, sorted({
            tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in res.contacts
        })

    def linha_cartesiana(self, q_ini, deslocamento, passo=0.001, evitar_colisao=True,
                         velocidade=0.1):
        """Reta do TCP a partir de q_ini, mantendo a inclinacao da garra.

        fracao < 1.0 indica que a reta nao foi concluida (limite, colisao ou alcance).
        Um deslocamento em Y nunca conclui: o braco nao sai do plano.
        """
        inicio = self.fk(q_ini)
        origem = (inicio.position.x, inicio.position.y, inicio.position.z)
        alvo = Pose()
        alvo.position = Point(
            x=origem[0] + deslocamento[0],
            y=origem[1] + deslocamento[1],
            z=origem[2] + deslocamento[2])
        alvo.orientation = inicio.orientation

        req = GetCartesianPath.Request()
        req.header.frame_id = BASE
        req.group_name = GRUPO
        req.link_name = TCP
        self._estado_robo(q_ini, req.start_state)
        req.waypoints = [alvo]
        req.max_step = passo
        req.avoid_collisions = evitar_colisao
        req.max_velocity_scaling_factor = velocidade
        req.max_acceleration_scaling_factor = velocidade
        res = self._chamar(self._cart, req)

        traj = res.solution
        pontos = traj.joint_trajectory.points
        if not pontos:
            return ResultadoLinha(None, 0.0, float('inf'), 0.0, 'sem trajetoria')
        salto = 0.0
        for a, b in zip(pontos, pontos[1:]):
            salto = max(salto, max(abs(x - y) for x, y in zip(a.positions, b.positions)))
        mensagem = '' if res.error_code.val == SUCESSO else f'codigo {res.error_code.val}'
        return ResultadoLinha(
            traj, res.fraction, self._desvio_da_reta(traj, origem, deslocamento), salto, mensagem)

    def _desvio_da_reta(self, traj, origem, deslocamento):
        """Maior distancia (m) entre o TCP na trajetoria e a reta pedida."""
        norma = math.sqrt(sum(d * d for d in deslocamento))
        u = [d / norma for d in deslocamento]
        pior = 0.0
        pontos = traj.joint_trajectory.points
        passo = max(1, len(pontos) // 40)
        for ponto in pontos[::passo] + pontos[-1:]:
            p = self.fk(list(ponto.positions)).position
            v = [p.x - origem[0], p.y - origem[1], p.z - origem[2]]
            t = sum(a * b for a, b in zip(v, u))
            pior = max(pior, math.sqrt(sum((a - t * b) ** 2 for a, b in zip(v, u))))
        return pior

    def _requisicao_plano(self, q_ini, restricoes, velocidade, tempo):
        req = GetMotionPlan.Request()
        m = MotionPlanRequest()
        m.group_name = GRUPO
        m.pipeline_id = 'ompl'
        m.num_planning_attempts = 5
        m.allowed_planning_time = tempo
        m.max_velocity_scaling_factor = velocidade
        m.max_acceleration_scaling_factor = velocidade
        self._estado_robo(q_ini, m.start_state)
        m.goal_constraints = [restricoes]
        req.motion_plan_request = m
        return req

    def planejar_juntas(self, q_ini, q_alvo, velocidade=0.1, tempo=5.0):
        """Planeja de q_ini ate q_alvo. Retorna (trajetoria, codigo)."""
        goal = Constraints()
        for nome, valor in zip(JUNTAS, q_alvo):
            c = JointConstraint()
            c.joint_name = nome
            c.position = float(valor)
            c.tolerance_above = c.tolerance_below = 0.005
            c.weight = 1.0
            goal.joint_constraints.append(c)
        res = self._chamar(self._plan, self._requisicao_plano(q_ini, goal, velocidade, tempo))
        return res.motion_plan_response.trajectory, res.motion_plan_response.error_code.val

    def planejar_pose(self, q_ini, pose, velocidade=0.1, tempo=5.0):
        """Planeja ate uma pose do TCP, resolvendo a IK antes. Retorna (trajetoria, codigo)."""
        q_alvo = self.ik(pose, q_ini)
        if q_alvo is None:
            return None, MoveItErrorCodes.NO_IK_SOLUTION
        return self.planejar_juntas(q_ini, q_alvo, velocidade, tempo)

    def executar(self, trajetoria):
        """Executa a trajetoria pelo move_group. Retorna o codigo de erro do MoveIt."""
        objetivo = ExecuteTrajectory.Goal()
        objetivo.trajectory = trajetoria
        envio = self._exec.send_goal_async(objetivo)
        rclpy.spin_until_future_complete(self, envio)
        handle = envio.result()
        if not handle.accepted:
            return MoveItErrorCodes.FAILURE
        resultado = handle.get_result_async()
        rclpy.spin_until_future_complete(self, resultado)
        return resultado.result().result.error_code.val
