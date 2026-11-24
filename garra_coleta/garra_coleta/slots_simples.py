#!/usr/bin/env python3
"""Slots da garra, versão SIMPLES para editar à mão.

Tudo que você precisa mexer está no topo deste arquivo:
  1. SIMULAR       -> True: só imprime os movimentos. False: move a garra de verdade (ROS).
  2. POSES         -> ângulos (graus) de cada pose: [junta1, junta2, junta3]
  3. SLOT_POSES    -> ângulos (graus) de cada slot
  4. slots         -> o vetor: slots[2] = 1 quer dizer "o cubo ID 1 está no slot 2"

Funções para usar:
  mover_garra(j1, j2, j3)   move a garra por ângulos (graus)
  deixar_cubo(id)           guarda o cubo que está na garra no primeiro slot livre
  pegar_cubo_do_slot(id)    tira o cubo do slot e mostra o AprilTag para a câmera
  onde_esta(id)             número do slot do cubo (ou None)
  mostrar_slots()           imprime o vetor

Teste rápido (sem robô):  python3 slots_simples.py
"""

import math
import time

# ============================ 1. MODO ============================================================
SIMULAR = True          # mude para False para mover a garra de verdade
VELOCIDADE = 0.2        # 0 a 1, fração da velocidade máxima

# ============================ 2. POSES (graus) ===================================================
# [junta1, junta2, junta3]. Ajuste depois de medir na garra. None = ainda não medido.
POSES = {
    "pronta":           [6.9, -54.4, 49.8],   # = PRONTA do cliente_garra (0.12, -0.95, 0.87 rad)
    "mostrar_apriltag": None,                 # cubo na frente da câmera, tag virada para ela
    "acima_slots":      None,                 # acima dos slots, sem bater nos cubos guardados
}

# ============================ 3. POSES DOS SLOTS (graus) =========================================
# Pose com o cubo encaixado no slot.
SLOT_POSES = [
    None,   # slot 0
    None,   # slot 1
    None,   # slot 2
    None,   # slot 3
]

# Dedos (graus do J4), iguais ao pegar_cubo.py
DEDOS_ABERTOS = 60.0
DEDOS_FECHADOS = 90.0

# Leitura do AprilTag (ler_apriltag)
TEMPO_LEITURA_S = 2.0   # quanto tempo espera a câmera ler
QUADROS_IGUAIS = 5      # quadros seguidos com o mesmo ID para aceitar

# ============================ 4. O VETOR DOS SLOTS ===============================================
# slots[n] = ID do cubo que está no slot n, ou None se o slot estiver vazio.
# Exemplo: [3, None, 1, None] -> cubo 3 no slot 0, cubo 1 no slot 2.
slots = [None, None, None, None]


# =================================================================================================
# Movimentos básicos
# =================================================================================================

def mover_garra(j1, j2, j3):
    """Move a garra para os ângulos dados (graus)."""
    print(f"  mover_garra({j1:.1f}, {j2:.1f}, {j3:.1f})")
    if SIMULAR:
        return
    alvo = [math.radians(j1), math.radians(j2), math.radians(j3)]
    garra = _ros()
    from garra_coleta.cliente_garra import SUCESSO
    traj, codigo = garra.planejar_juntas(garra.juntas_atuais(), alvo, VELOCIDADE)
    if codigo != SUCESSO:
        raise RuntimeError(f"não consegui planejar até {j1, j2, j3} (código {codigo})")
    codigo = garra.executar(traj)
    if codigo != SUCESSO:
        raise RuntimeError(f"movimento falhou (código {codigo})")


def ir_para(nome):
    """Vai para uma pose do dicionário POSES pelo nome."""
    angulos = POSES[nome]
    if angulos is None:
        raise RuntimeError(f'a pose "{nome}" ainda não foi medida (POSES no topo do arquivo)')
    print(f"-> {nome}")
    mover_garra(*angulos)


def ir_para_slot(numero):
    angulos = SLOT_POSES[numero]
    if angulos is None:
        raise RuntimeError(f"o slot {numero} ainda não foi medido (SLOT_POSES no topo do arquivo)")
    print(f"-> slot {numero}")
    mover_garra(*angulos)


def abrir_dedos():
    print("  abrir dedos")
    if not SIMULAR:
        _dedos(DEDOS_ABERTOS)


def fechar_dedos():
    print("  fechar dedos")
    if not SIMULAR:
        _dedos(DEDOS_FECHADOS)


def ler_apriltag():
    """ID do AprilTag mais perto da câmera (o cubo que a garra mostra), ou None.

    Lê /camera/tags_vistos (a camera_cubo precisa estar rodando). Só aceita o ID se ele for
    o maior tag da imagem em QUADROS_IGUAIS quadros seguidos, dentro de TEMPO_LEITURA_S.
    Com SIMULAR = True devolve None (as funções usam o ID que você passou).
    """
    if SIMULAR:
        return None
    import rclpy
    from std_msgs.msg import Int32MultiArray
    garra = _ros()
    quadros = []
    sub = garra.create_subscription(
        Int32MultiArray, "/camera/tags_vistos", lambda m: quadros.append(list(m.data)), 10)
    try:
        fim = time.time() + TEMPO_LEITURA_S
        while time.time() < fim:
            rclpy.spin_once(garra, timeout_sec=0.05)
            ultimos = [q[0] if q else None for q in quadros[-QUADROS_IGUAIS:]]
            if len(ultimos) == QUADROS_IGUAIS and ultimos[0] is not None \
                    and all(i == ultimos[0] for i in ultimos):
                print(f"  câmera leu o cubo {ultimos[0]}")
                return ultimos[0]
        if not quadros:
            print("  AVISO: nada em /camera/tags_vistos (a camera_cubo está rodando?)")
        else:
            print("  AVISO: a câmera não leu um tag estável")
        return None
    finally:
        garra.destroy_subscription(sub)


# =================================================================================================
# Slots
# =================================================================================================

def slot_livre():
    """Primeiro slot vazio (o de menor número), ou None se todos estiverem cheios."""
    for numero, cubo in enumerate(slots):
        if cubo is None:
            return numero
    return None


def onde_esta(cubo_id):
    """Número do slot onde está o cubo, ou None."""
    for numero, cubo in enumerate(slots):
        if cubo == cubo_id:
            return numero
    return None


def mostrar_slots():
    for numero, cubo in enumerate(slots):
        print(f"  slot {numero}: {'vazio' if cubo is None else f'cubo {cubo}'}")


def deixar_cubo(cubo_id):
    """Guarda o cubo que está na garra. Devolve o slot usado.

    Chamar logo depois de pegar o cubo do chão (a garra já está com ele).
    """
    if onde_esta(cubo_id) is not None:
        raise RuntimeError(f"o cubo {cubo_id} já está no slot {onde_esta(cubo_id)}")
    numero = slot_livre()
    if numero is None:
        raise RuntimeError("todos os slots estão cheios")

    print(f"deixar_cubo({cubo_id}) -> slot {numero}")
    ir_para("mostrar_apriltag")      # mostra o tag para conferir qual cubo é
    lido = ler_apriltag()
    if lido is not None and lido != cubo_id:
        print(f"  AVISO: a câmera leu o cubo {lido}, não o {cubo_id}. Usando {lido}.")
        cubo_id = lido

    ir_para("acima_slots")
    ir_para_slot(numero)
    abrir_dedos()
    slots[numero] = cubo_id          # anota só depois de soltar
    ir_para("acima_slots")
    ir_para("pronta")
    return numero


def pegar_cubo_do_slot(cubo_id):
    """Tira o cubo do slot e termina mostrando o tag para a câmera. Devolve o ID.

    Depois disso a garra está com o cubo: a missão decide o que fazer com ele.
    """
    numero = onde_esta(cubo_id)
    if numero is None:
        raise RuntimeError(f"o cubo {cubo_id} não está em nenhum slot")

    print(f"pegar_cubo_do_slot({cubo_id}) <- slot {numero}")
    abrir_dedos()
    ir_para("acima_slots")
    ir_para_slot(numero)
    fechar_dedos()
    slots[numero] = None             # o cubo saiu do slot
    ir_para("acima_slots")
    ir_para("mostrar_apriltag")
    lido = ler_apriltag()
    if lido is not None and lido != cubo_id:
        print(f"  AVISO: o slot {numero} tinha o cubo {lido}, não o {cubo_id}.")
        return lido
    return cubo_id


# =================================================================================================
# Parte do ROS (não precisa mexer). Só é usada com SIMULAR = False.
# =================================================================================================
_garra = None


def _ros():
    global _garra
    if _garra is None:
        import rclpy
        from garra_coleta.cliente_garra import ClienteGarra
        if not rclpy.ok():
            rclpy.init()
        _garra = ClienteGarra("slots_simples")
        _garra.aguardar()
    return _garra


def _dedos(graus):
    import rclpy
    from control_msgs.action import GripperCommand
    from rclpy.action import ActionClient
    garra = _ros()
    cliente = ActionClient(garra, GripperCommand, "/garra_dedos_controller/gripper_cmd")
    if not cliente.wait_for_server(timeout_sec=5.0):
        raise RuntimeError("ação dos dedos indisponível")
    objetivo = GripperCommand.Goal()
    objetivo.command.position = math.radians(graus)
    envio = cliente.send_goal_async(objetivo)
    rclpy.spin_until_future_complete(garra, envio)
    resultado = envio.result().get_result_async()
    rclpy.spin_until_future_complete(garra, resultado)
    time.sleep(0.3)


# =================================================================================================
# Exemplo: rode "python3 slots_simples.py" (com SIMULAR = True não precisa do robô)
# =================================================================================================
if __name__ == "__main__":
    if SIMULAR:
        # Poses de mentira só para o exemplo rodar; apague quando medir as de verdade.
        POSES["mostrar_apriltag"] = [0.0, -30.0, 30.0]
        POSES["acima_slots"] = [90.0, -20.0, 10.0]
        SLOT_POSES[:] = [[80.0, 10.0, 20.0], [90.0, 10.0, 20.0],
                         [100.0, 10.0, 20.0], [110.0, 10.0, 20.0]]

    deixar_cubo(1)          # peguei o cubo 1 -> vai para o slot 0
    deixar_cubo(3)          # cubo 3 -> slot 1
    mostrar_slots()
    pegar_cubo_do_slot(1)   # tira o cubo 1, slot 0 fica livre
    deixar_cubo(5)          # cubo 5 -> slot 0 (o menor livre)
    mostrar_slots()
    print("vetor:", slots)
