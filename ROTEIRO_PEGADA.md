# Roteiro de teste da pegada (bancada)

Objetivo: chegar ao treino com o robô sabendo que a garra pega o cubo de forma repetível, primeiro
com posição medida na trena e depois com a posição vinda da câmera. Siga em ordem: cada etapa só
vale se a anterior passou. Anote os resultados na tabela do fim.

**Antes de tudo, em todo terminal:**

```bash
source /opt/ros/jazzy/setup.bash && source ~/work/garra/install/setup.bash
cd ~/work/garra
```

**Segurança:** sempre alguém com a **mão na fonte dos servos** enquanto a garra se mexe. Nada no
caminho do braço. Na dúvida, desliga a fonte.

**Medidas:** x medido da **borda dianteira** da base; y da linha central (+ para a esquerda).
Para o código: `x = x_da_borda + 0,275`. Cubo no chão: `z = 0.021`.

---

## Etapa 0 — Preparação (5 min)

- [ ] Firmware `firmware/garra_servos` gravado no Mega (não o `teste_fins_de_curso`).
- [ ] Fonte dos servos ligada e com carga; Arduino no USB.
- [ ] Monitor Serial e scripts de calibração **fechados** (a porta serial só tem um dono).
- [ ] Compilado: `colcon build --symlink-install` e `source install/setup.bash` de novo.

## Etapa 1 — Conferir a calibração dos servos (15 min)

```bash
ros2 launch garra_moveit_config garra_real.launch.py      # terminal 1 (deixe aberto)
python3 scripts/mover_junta.py home                        # terminal 2
python3 scripts/mover_junta.py J1 0.5
python3 scripts/mover_junta.py home
python3 scripts/mover_junta.py J2 -0.5
python3 scripts/mover_junta.py home
python3 scripts/mover_junta.py J3 0.5
python3 scripts/mover_junta.py home
```

Em cada pose, compare o elo real com o RViz (celular/nível). No 1º elo, meça **na parte reta perto
do J2** (o elo é curvo).

- **Passa:** todas as juntas a ±3° do RViz e indo para o mesmo lado.
- **Falha:** veja o item 1.4 do `CALIBRACAO.md` (zero, escala ou sentido) e corrija antes de seguir.

## Etapa 2 — Pegada com posição medida na trena (30 min)

Com o `garra_real.launch.py` ainda aberto. Cubo no chão, **y = −3 cm** (plano da garra).

1. Cubo a **18,5 cm** da borda. Planeje sem mexer:

   ```bash
   ros2 run garra_coleta pegar_cubo 0.46 0.021 --so-planejar
   ```

2. Se planejou, execute de verdade:

   ```bash
   ros2 run garra_coleta pegar_cubo 0.46 0.021
   ```

3. Devolva o cubo (ou recoloque à mão no mesmo lugar):

   ```bash
   ros2 run garra_coleta pegar_cubo 0.46 0.021 --soltar
   ```

4. Repita **5 vezes** em 18,5 cm. Depois **3 vezes** em cada um dos outros pontos:

   | Da borda | Comando |
   |---|---|
   | 13 cm | `pegar_cubo 0.405 0.021` |
   | 17 cm | `pegar_cubo 0.445 0.021` |
   | 21 cm | `pegar_cubo 0.485 0.021` |

**Passa:** ≥ 4 de 5 em 18,5 cm e pelo menos 2 de 3 em cada outro ponto, com o cubo saindo do chão
e voltando para `PRONTA` sem cair.

**Se falhar, olhe como falhou:**

| O que acontece | Provável causa | O que fazer |
|---|---|---|
| Ponta fica **acima** do cubo / fecha no ar | descida curta | aumente `--descer` (ex.: `--descer 10`) |
| Ponta **bate no chão** / empurra o cubo para baixo | descida demais | diminua `--descer` |
| Ponta para **antes** ou **depois** do cubo, sempre igual | zero do J1/J2 | volte à Etapa 1 |
| Erro cresce de 13 → 21 cm | escala (`us_por_rad`) | `scripts/medir_escala.py` (CALIBRACAO 1.5) |
| Fecha mas o cubo escorrega | fechamento fraco | aumente `--fechamento` (máx. 100°) |
| Cubo empurrado para o lado | cubo fora de y = −3 cm | reposicione; o braço é planar |
| "falhou ao planejar" | ponto fora do alcance | confira x entre 0.40 e 0.49 |

Anote o valor de `--descer`/`--fechamento` que funcionou: ele precisa ir para o padrão do
`pegar_cubo.py` antes do treino com o robô.

## Etapa 3 — Pegada com a posição da câmera (30 min)

Feche Cheese/navegador (a webcam só abre em um programa). `garra_real.launch.py` aberto.

1. Confira a câmera sem ROS. Ponha o cubo em 13, 17 e 21 cm (y = −3) e veja o texto
   `da borda: x=… y=…` ao lado do tag. Tem que bater com a trena em **~1 cm**:

   ```bash
   ros2 run garra_coleta camera_cubo --sem-ros
   ```

   Se não bater, recalibre (CALIBRACAO.md, seção 2) antes de seguir.

2. Pegada mandada pela câmera:

   ```bash
   ros2 run garra_coleta pegar_cubo --soltar      # terminal 2: pega e devolve no mesmo lugar
   ros2 run garra_coleta camera_cubo              # terminal 3
   ```

   Na janela da câmera aperte **P** para mandar o cubo mais perto. Repita 5 vezes, mudando o cubo
   de lugar dentro da faixa 13–21 cm.

**Passa:** ≥ 4 de 5. Se a Etapa 2 passou e esta falha, o problema é a câmera, não a garra.

## Etapa 4 — Plataforma Y (10 min, opcional)

O homing está suspenso até chegar a peça nova da chave. Só confira que ela **não desliza sozinha**:
mande parar e observe por 30 s. Se andar, meça o neutro de novo (`Y <pulso>`, ver
`firmware/garra_servos`). Hoje o neutro é 1528 µs.

## Etapa 5 — Coleta completa como o robô vai pedir (20 min)

Simula o executor do robô, com a base **parada** (`base:=false`) para não mexer o robô na bancada:

```bash
ros2 launch garra_coleta coleta.launch.py garra:=real base:=false     # terminal 1 (feche os outros antes)
ros2 topic echo /coleta/resultado                                    # terminal 2
ros2 topic pub --once /coleta/pedido std_msgs/msg/String "{data: 'teste'}"   # terminal 3
```

**Passa:** a garra pega o cubo e `/coleta/resultado` mostra `ok`. Com o cubo fora da faixa, o
terminal 1 mostra `fora do alcance -> /robo/mover` (ninguém atende com `base:=false`) e, depois de
4 tentativas (~20 s cada), `/coleta/resultado` mostra `falhou: …`, sem travar.

---

## Resultados

| Etapa | Ponto / caso | Tentativas | Acertos | Parâmetros usados | Observações |
|---|---|---|---|---|---|
| 1 | J1 / J2 / J3 | – | – | – | |
| 2 | 18,5 cm | 5 | | | |
| 2 | 13 cm | 3 | | | |
| 2 | 17 cm | 3 | | | |
| 2 | 21 cm | 3 | | | |
| 3 | câmera | 5 | | | |
| 4 | plataforma parada | – | – | | |
| 5 | coleta completa | | | | |

**Pronto para o robô amanhã se:** Etapas 1, 2 e 3 passaram. A Etapa 5 dá para fazer já no robô,
de manhã. Se a 3 não passou, treine a navegação com `executor_prova.py --sem-garra` enquanto
corrige a câmera.
