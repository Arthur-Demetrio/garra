


import math


def normalizar(q):
    norma = math.sqrt(sum(v * v for v in q))
    return tuple(v / norma for v in q)


def comparar(q_inicial, q_final):

    q1 = normalizar(q_inicial)
    q2 = normalizar(q_final)

    produto = sum(a * b for a, b in zip(q1, q2))

    # q e -q representam a mesma orientação
    produto = abs(produto)

    produto = max(-1.0, min(1.0, produto))

    angulo = 2.0 * math.acos(produto)

    return math.degrees(angulo)


# Orientação inicial obtida pelo TF
orientacao_inicial = (
    -0.349906,
    0.635908,
    0.563035,
    0.395194
)

# Orientação final obtida pelo compute_fk
orientacao_final = (
    -0.347122,
    0.637848,
    0.564768,
    0.392038
)

angulo = comparar(
    orientacao_inicial,
    orientacao_final
)

print('========== COMPARAÇÃO DE ORIENTAÇÕES ==========')
print(f'Rotação efetivamente calculada: {angulo:.4f} graus')
print('Rotação solicitada: 5.0000 graus')
print(f'Diferença: {abs(5.0 - angulo):.4f} graus')
print('================================================')
