# Garra Robótica — ROS  2 Jazzy

Projeto de uma garra robótica planar de **três juntas rotativas**, modelada a partir de uma montagem CAD do Fusion 360, descrita em URDF/Xacro e integrada ao **MoveIt 2**. O repositório reúne a descrição do robô, a configuração de planejamento, uma interface de hardware baseada em `ros2_control`, o firmware para Arduino Mega e scripts Python de calibração e validação.

O sistema foi organizado para permitir desenvolvimento e testes em duas etapas:

- **Simulação:** visualizar o modelo no RViz 2, calcular cinemática, verificar colisões e planejar/executar trajetórias com controladores simulados.
- **Hardware físico:** enviar comandos aos servos do braço (J1–J3), dos dedos (J4) e da plataforma do eixo Y por meio de um Arduino Mega conectado por USB serial.

> **Atenção:** os testes automatizados de `scripts/validar_garra.py` são destinados à simulação. Não os execute com a garra física conectada/acionada. Antes de operar o hardware, faça a calibração individual dos servos e confira os limites mecânicos e a alimentação.

## 1. Tecnologias e ambiente

O ambiente de desenvolvimento considerado neste projeto é:

- Linux Mint 22.x (base Ubuntu 24.04 / Noble);
- ROS 2 Jazzy;
- MoveIt 2;
- `ros2_control` e `ros2_controllers`;
- `colcon` para compilação;
- Python 3 e `rclpy`;
- Arduino Mega;
- Arduino IDE e biblioteca `Servo` para gravar o firmware.

O ROS 2 Jazzy tem como plataforma binária oficial o Ubuntu 24.04. Em Linux Mint, que é derivado do Ubuntu, a disponibilidade de pacotes pode variar. O procedimento pressupõe que o ROS 2 Jazzy já esteja instalado e que `source /opt/ros/jazzy/setup.bash` funcione no terminal.

## 2. Estrutura do repositório

```text
garra/
├── firmware/
│   ├── garra_servos/
│   │   └── garra_servos.ino
│   ├── ino atual/
│   │   └── ...                  # versão anterior do firmware
│   ├── plataforma/              # teste avulso do servo da plataforma Y (ajuste de pulso)
│   └── plataforma_sem_sinal/    # teste avulso da plataforma Y (E/D/P, para cortando o sinal)
├── garra_final_description/
│   ├── urdf/
│   │   └── garra_final.urdf.xacro
│   ├── meshes/                  # malhas exportadas do CAD
│   ├── launch/
│   │   └── display.launch.py
│   └── ...
├── garra_hardware/
│   ├── include/
│   ├── src/
│   │   └── garra_servo_system.cpp
│   ├── CMakeLists.txt
│   └── package.xml
├── garra_moveit_config/
│   ├── config/
│   │   ├── garra_final.urdf.xacro   # garra + estrutura do robô (frame robo_base)
│   │   ├── garra_final.srdf
│   │   ├── kinematics.yaml
│   │   ├── joint_limits.yaml
│   │   ├── ros2_controllers.yaml
│   │   ├── moveit_controllers.yaml
│   │   └── calibracao_servos.yaml
│   ├── launch/
│   │   ├── garra_real.launch.py
│   │   └── ...
│   └── ...
├── garra_coleta/                # pacote da coleta (nós + launchers), o que o robô inclui
│   ├── garra_coleta/
│   │   ├── cliente_garra.py         # biblioteca cliente do MoveIt
│   │   ├── pegar_cubo.py
│   │   ├── alcance_cubo.py
│   │   ├── ajuste_base.py
│   │   ├── camera_cubo.py
│   │   └── plataforma_y.py
│   ├── launch/
│   │   ├── coleta.launch.py         # atalho: sobe a coleta inteira
│   │   ├── pegar_cubo.launch.py
│   │   ├── ajuste_base.launch.py
│   │   ├── alcance_cubo.launch.py
│   │   ├── camera_cubo.launch.py
│   │   ├── tf_camera.launch.py
│   │   └── coleta_comum.py          # funções comuns dos launchers
│   ├── config/                      # calibração da webcam e pose da câmera do -WORK
│   ├── package.xml
│   └── setup.py
├── scripts/                         # ferramentas (validação, calibração, testes)
│   ├── validar_garra.py
│   ├── calibrar_servos.py
│   ├── calibrar_camera.py
│   ├── medir_escala.py
│   ├── testar_sensor.py
│   ├── ajustar_urdf.py
│   ├── mover_junta.py
│   └── nao_usados/              # scripts históricos, fora dos procedimentos
├── integracao_robo/             # patch do executor_prova.py do robô (Robo-WORK)
├── servo_imgs/                  # imagens e vídeo de apoio à calibração
├── dimensoes.md                 # medidas do robô móvel (base, andares, montagem da garra)
├── planejamento.md
├── .gitignore
├── README.md
└── GUIA_TESTES.md
```

Os diretórios `build/`, `install/` e `log/` são gerados pelo `colcon` e não devem ser versionados. A pasta `scripts/nao_usados/` contém material histórico e não faz parte dos procedimentos documentados neste README ou no guia de testes.

### 2.1 Pacotes ROS 2

| Pacote | Responsabilidade |
|---|---|
| `garra_final_description` | Descrição do robô: URDF/Xacro, malhas, juntas, links, publicação de TF e launch de visualização. |
| `garra_moveit_config` | Configuração do MoveIt 2: grupo de planejamento, cinemática, limites, controladores, RViz e launch para simulação/hardware. |
| `garra_hardware` | Implementação da interface de hardware `ros2_control` para comunicação serial com o Arduino Mega. |
| `garra_coleta` | Coleta de cubos: nós `pegar_cubo`, `alcance_cubo`, `ajuste_base`, `camera_cubo` e `plataforma_y` (`ros2 run garra_coleta <nó>`), TF da câmera e launchers. |

### 2.2 Firmware

O firmware principal está em `firmware/garra_servos/garra_servos.ino`. Ele recebe comandos pela serial a **115200 baud**.

- Arduino Mega;
- pinos 9, 10 e 11 para os servos do braço (J1–J3, DS3218MG) e pino 12 para o servo dos dedos (J4, DS-S006MP);
- pino 8: servo de rotação contínua da plataforma do eixo Y;
- pino 3: chave fim de curso dos dedos (normalmente aberta, para o GND, com pull-up interno);
- pulsos de comando entre 500 e 2500 µs.

| Comando | Resposta | Uso |
|---|---|---|
| `V` | `GARRA 1` | Identificação (handshake do driver). |
| `M <j1> <j2> <j3> [<j4>]*HH` | `K` ou `E <motivo>` | Pulsos em µs (0 = não altera); `HH` = XOR em hexadecimal dos bytes antes do `*`. |
| `S` | `S 1` (cubo preso) / `S 0` | Lê a chave dos dedos; o driver publica em `/garra/cubo_preso`. |
| `Y <E\|D\|P\|?>` | `Y ESQUERDA` / `Y DIREITA` / `Y PARADO` | Plataforma Y. Para sozinha após 500 ms sem novo comando (`Y TIMEOUT`). |
| `<motor>,<ângulo>` | — | Teste manual pelo Monitor Serial (ex.: `1,90`); ignora a calibração. |

A pasta `firmware/ino atual/` contém uma versão anterior e deve ser tratada como referência histórica; para a montagem descrita no projeto, utilize o firmware em `firmware/garra_servos/`.

### 2.3 Scripts Python

| Arquivo | Finalidade |
|---|---|
| `garra_coleta/garra_coleta/cliente_garra.py` | Biblioteca cliente ROS 2 para chamar serviços/ações do MoveIt: FK, IK, verificação de colisão, planejamento e execução de trajetórias. É importada pelos demais scripts; não é um teste executável isoladamente. |
| `scripts/validar_garra.py` | Validação automatizada da descrição e do planejamento em simulação. Verifica TCP, autocolisão, IK, limites, trajetórias cartesianas e planejamento. |
| `scripts/calibrar_servos.py` | Ferramenta interativa que comunica diretamente com o Arduino para medir zero, sentido e limites de pulso dos três servos e gerar `calibracao_servos.yaml`. |
| `garra_coleta/garra_coleta/pegar_cubo.py` | Pega o cubo numa posição de `robo_base` (argumentos ou tópico `/pegar_cubo`) e entrega o cubo que já está nos dedos (`--entregar` ou tópico `/entregar_cubo`); publica o resultado em `/pegar_cubo/resultado` e `/entregar_cubo/resultado`. |
| `garra_coleta/garra_coleta/alcance_cubo.py` | Durante uma coleta (`/coleta/pedido`), decide entre chamar a garra (`/pegar_cubo`) e pedir um ajuste da base (`/robo/mover`); responde em `/coleta/resultado`. |
| `garra_coleta/garra_coleta/ajuste_base.py` | Executa o ajuste curto da base pedido em `/robo/mover`, pelo canal `especial` do árbitro do robô (até 15 cm, 0,05 m/s). |
| `garra_coleta/garra_coleta/camera_cubo.py` | Webcam + AprilTag: acha os cubos e publica a posição em `robo_base`. A posição sai do pixel do centro do tag cruzado com a altura conhecida do cubo (`--pnp` usa o método antigo, ~11% mais longe). Na tela, o x aparece medido da borda dianteira (ver [seção 7.6](#76-câmera-medidas-e-calibração)). |
| `garra_coleta/garra_coleta/plataforma_y.py` | Leva a plataforma do eixo Y até um `y` pedido em `/garra/plataforma_y/alvo`, em malha fechada pela câmera (`camera_cubo --tag-plataforma N`); comanda `/garra/plataforma_y` (Int8 −1/0/+1) e responde em `/garra/plataforma_y/resultado`. `--testar-sentido` confere o sentido. |
| `scripts/calibrar_camera.py` | Calibra ao vivo a pose da webcam (inclinação e altura pelos tags; x/y com um cubo medido na trena, tecla K) e salva em `~/.config/garra/camera_montagem.json`. Usa o mesmo método de posição do `camera_cubo.py`. |
| `scripts/mover_junta.py` | Move juntas pelo MoveIt sem o RViz (`J1 0.5`, `home`, `pronta`; `--graus`). |
| `scripts/medir_escala.py` | Mede a escala real de cada servo com um inclinômetro, direto pela serial (sem ROS). |
| `scripts/testar_sensor.py` | Mostra o estado da chave dos dedos pela serial (sem ROS; feche o `garra_real.launch.py`). |
| `scripts/ajustar_urdf.py` | Ajusta a URDF depois de uma nova exportação do CAD (limites, eixos, `garra_base`). |

Os scripts da coleta têm launchers próprios; veja a [seção 7.5](#75-coleta-automática-de-cubos).

Os comandos completos e a sequência recomendada estão em [`GUIA_TESTES.md`](GUIA_TESTES.md).

## 3. Convenções cinemáticas

A configuração usada pelos scripts e pelo MoveIt é:

| Elemento | Nome |
|---|---|
| Grupo de planejamento | `garra` |
| Link de referência da base | `garra_base` (no robô, `base_link` é o chassi) |
| TCP (ponta/ferramenta) | `tool0` |
| Junta 1 | `Revolucionar_46` |
| Junta 2 | `Revolucionar_49` |
| Junta 3 | `Revolucionar_72` |

A garra é planar: os três eixos de junta são paralelos. O movimento do TCP ocorre no plano X–Z e a coordenada Y permanece fixa. Portanto, um deslocamento cartesiano em Y não é esperado ser executável.

As posições articulares são expressas em radianos e as posições cartesianas em metros.

Frame `robo_base` (raiz da árvore, usado pela coleta): centro da base do robô no chão; X para a frente, Y para a esquerda, Z para cima. A base tem 0,55 m em X, então a **borda dianteira fica em X = 0,275 m**. O `garra_base` fica em X = 0,225 m, girado 180° em Z. O plano da garra (Y do TCP) é Y ≈ −0,029 m.

## 4. Preparação do ambiente

```bash
source /opt/ros/jazzy/setup.bash
cd ~/work/garra

colcon build --symlink-install
source install/setup.bash
```

Se a compilação terminar corretamente, o workspace estará disponível no terminal atual. Em cada novo terminal, carregue os ambientes:

```bash
source /opt/ros/jazzy/setup.bash
source ~/work/garra/install/setup.bash
```

O repositório é o próprio workspace (`build/`, `install/` e `log/` ficam na raiz). Se ele estiver em outro caminho, ajuste o segundo comando. Os dois `source` são necessários em **todo terminal novo** antes de qualquer `ros2` ou script; para não esquecer, dá para colocá-los no `~/.bashrc`:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/work/garra/install/setup.bash' >> ~/.bashrc
```

Depois de um `colcon build`, rode `source ~/work/garra/install/setup.bash` de novo nos terminais já abertos.

## 5. Executar a visualização

Para abrir a descrição da garra no RViz 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/work/garra/install/setup.bash

ros2 launch garra_final_description display.launch.py
```

Esse launch inicia a publicação do modelo e os recursos de visualização/juntas previstos pelo pacote. Use essa etapa para conferir se links, malhas, juntas e TFs estão coerentes antes de testar planejamento.

## 6. Executar o MoveIt 2 em simulação

Em outro terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/work/garra/install/setup.bash

ros2 launch garra_moveit_config demo.launch.py
```

O launch inicia o MoveIt 2 com controladores simulados. Aguarde o RViz e o `move_group` terminarem de inicializar antes de executar scripts clientes.

Para validar a simulação, consulte [`GUIA_TESTES.md`](GUIA_TESTES.md).

## 7. Operação com hardware físico

### 7.1 Gravar o firmware

1. Abra `firmware/garra_servos/garra_servos.ino` na Arduino IDE.
2. Selecione **Arduino Mega** e a porta correta.
3. Verifique a instalação da biblioteca `Servo`.
4. Grave o sketch.
5. Confirme que a alimentação dos servos é adequada e que o GND da fonte e do Arduino é comum.

Não alimente os servos diretamente pelo pino de 5 V do Arduino. Dimensione a fonte para a corrente exigida pelos servos e mantenha um meio de interromper a alimentação em caso de movimento inesperado.

### 7.2 Permissão da porta serial no Linux Mint

Identifique a porta:

```bash
ls /dev/ttyACM*
ls /dev/ttyUSB*
```

Se houver erro de permissão, verifique o grupo do dispositivo. Em instalações usuais, o acesso serial é concedido ao grupo `dialout`:

```bash
sudo usermod -aG dialout "$USER"
```

Depois, encerre a sessão e entre novamente (ou reinicie o computador) para aplicar a alteração. Não use `sudo` para executar os scripts ROS como solução permanente.

### 7.3 Calibrar os servos

A calibração deve ser feita antes de iniciar o hardware com MoveIt. Exemplo:

```bash
source /opt/ros/jazzy/setup.bash
source ~/work/garra/install/setup.bash

cd ~/work/garra
python3 scripts/calibrar_servos.py --versao 180 --porta /dev/ttyUSB0
```

Use `--versao 270` se os servos utilizados forem da faixa mecânica de 270 graus. O script grava os dados em `garra_moveit_config/config/calibracao_servos.yaml` por padrão.

**Não execute o ROS 2/driver de hardware ao mesmo tempo que o script de calibração estiver usando a mesma porta serial.** Consulte o guia de testes para os comandos interativos e os cuidados de segurança.

O arquivo de calibração atualmente versionado contém valores provisórios, limitados a 1400–1600 µs. Não considere esses valores uma calibração mecânica real.

### 7.4 Iniciar o MoveIt com a garra física

Depois de gravar o firmware, calibrar os servos (J1–J3 e dedos) e conferir a montagem:

```bash
source /opt/ros/jazzy/setup.bash
source ~/work/garra/install/setup.bash

export GARRA_PORTA=/dev/ttyUSB0
ros2 launch garra_moveit_config garra_real.launch.py
```

Para iniciar sem RViz:

```bash
source /opt/ros/jazzy/setup.bash
source ~/work/garra/install/setup.bash

ros2 launch garra_moveit_config garra_real.launch.py use_rviz:=false
```

Para selecionar outro arquivo de calibração:

```bash
export GARRA_CALIBRACAO=/caminho/absoluto/calibracao_servos.yaml
```

O launch configura o plugin `garra_hardware/GarraServoSystem`, inicia o `ros2_control_node`, os controladores e o `move_group`. O driver realiza um handshake com o firmware antes de aceitar comandos.

### 7.5 Coleta automática de cubos

Fluxo: o executor do robô (`executor_prova.py`, com o patch de `integracao_robo/`) chega na mesa e publica `/coleta/pedido`. O `alcance_cubo.py` olha a câmera: se o cubo está fora do alcance, pede ao `ajuste_base.py` um ajuste curto da base e mede de novo; se está ao alcance, chama o `pegar_cubo.py`. No fim responde `ok` ou `falhou: motivo` em `/coleta/resultado`. Fora de uma coleta, a câmera é ignorada e a base não se mexe sozinha.

Com o MoveIt da garra já rodando, tudo de uma vez:

```bash
source /opt/ros/jazzy/setup.bash
source ~/work/garra/install/setup.bash

ros2 launch garra_coleta coleta.launch.py
ros2 launch garra_coleta coleta.launch.py garra:=real            # inclui o garra_real.launch.py
ros2 launch garra_coleta coleta.launch.py camera:=vision         # cube_vision do -WORK no lugar do camera_cubo.py
```

Ou cada parte em um terminal:

| Launcher | O que roda | Argumentos principais |
|---|---|---|
| `coleta.launch.py` | Inclui os launchers abaixo | `garra:=nenhuma\|real\|demo`, `camera:=camera_cubo\|vision\|nenhuma`, `base`, `girar`, `sempre_ativo`, `id`, `velocidade`, `apoio` |
| `pegar_cubo.launch.py` | `pegar_cubo.py` escutando `/pegar_cubo` | `velocidade`, `fechamento`, `descer`, `so_planejar`, `sem_sensor` |
| `ajuste_base.launch.py` | `ajuste_base.py` | `vel`, `vel_giro`, `max_desloc`, `max_giro`, `timeout`, `topico_cmd`, `sem_arbitro` |
| `alcance_cubo.launch.py` | `alcance_cubo.py` | `vision`, `id`, `girar`, `sempre_ativo`, `max_ajustes`, `max_pegadas` |
| `camera_cubo.launch.py` | `camera_cubo.py` publicando em `/cubo_detectado` (abre uma janela) | `topico`, `auto`, `indice` (webcam), `id`, `apoio` (altura da superfície dos cubos) |
| `tf_camera.launch.py` | TF estática `robo_base` → `camera_optical_frame` | — |

Use `ros2 launch garra_coleta <launcher> --show-args` para ver todos os argumentos e padrões.

Observações:

- O `tf_camera.launch.py` calcula a pose com o mesmo código do `camera_cubo.py` (`calibrar_camera.py` > `robot_config_webcam.yaml` do -WORK > trena); ele é necessário para o `cube_vision`, que publica no frame da câmera.
- Com `camera:=vision`, suba o `cube_vision` em outro terminal, com o ambiente virtual da câmera (OpenCV ≥ 4.8) ativo.
- Os nós e launchers são do pacote `garra_coleta`: depois de mudar o código, rode `colcon build` de novo. A montagem calibrada da câmera (`calibrar_camera.py`) fica em `~/.config/garra/camera_montagem.json` e vale sem recompilar.
- `sempre_ativo:=true` faz o `alcance_cubo.py` agir sem `/coleta/pedido`: use só em testes, porque a base pode se mexer a qualquer momento.
- Para testar sem o robô: `ros2 topic pub --once /coleta/pedido std_msgs/msg/String "{data: 'teste'}"` com a câmera vendo um cubo.
- O `pegar_cubo.py` usa só x e z. Um cubo fora do plano da garra (Y ≈ −2,9 cm ± 1,5 cm) é recusado pelo `alcance_cubo.py`, mas não por quem publica direto em `/pegar_cubo`.

### 7.6 Câmera, medidas e calibração

**Convenção de medida:** toda medida feita com a trena parte da **borda dianteira da base**. O código converte para `robo_base` somando a distância do centro do robô à borda dianteira:

```
x_robo_base = x_medido_da_borda + 0,275 m        (constante BORDA_DIANTEIRA em camera_cubo.py)
y = esquerda (+) / direita (−) da linha central do robô
```

O `calibrar_camera.py` (tecla K) pede o x a partir da borda, e as janelas do `calibrar_camera.py` e do `camera_cubo.py` mostram o x a partir da borda (o `camera_cubo.py` mostra também o x de `robo_base`). Se a base mudar de tamanho, atualize só `BORDA_DIANTEIRA`.

**Janela de coleta** (calibração de 2026-11-16), no chão:

| | `robo_base` | Medido da borda dianteira |
|---|---|---|
| X | 0,40 – 0,49 m | 12,5 – 21,5 cm |
| Y | −0,029 ± 0,015 m | −2,9 ± 1,5 cm |

**Calibrar a câmera** (feche antes o `camera_cubo.py` e qualquer programa usando a webcam):

```bash
source /opt/ros/jazzy/setup.bash
source ~/work/garra/install/setup.bash

cd ~/work/garra
python3 scripts/calibrar_camera.py
```

1. Deixe alguns cubos no chão, com o tag para cima, até a inclinação e a altura estabilizarem.
2. Deixe **um** cubo visível num ponto medido (bom ponto: 19 cm à frente da borda, y = −3 cm), aperte **K** e digite `19 -3`.
3. Aperte **S** para salvar em `~/.config/garra/camera_montagem.json`.
4. Confira com `ros2 run garra_coleta camera_cubo --sem-ros` em dois ou três outros pontos medidos: o x/y da tela deve bater com a trena.

**Plataforma do eixo Y:** com `camera_cubo --tag-plataforma N --altura-plataforma <m>`, o tag `N` (preso na plataforma) não é tratado como cubo; a posição dele sai em `/garra/plataforma_y/posicao` e o `plataforma_y.py` usa essa leitura para levar o plano da garra até o `y` pedido. Esses argumentos ainda não estão no `camera_cubo.launch.py`: rode pelo `ros2 run`.

Prioridade da pose da câmera: `camera_montagem.json` > `config/robot_config_webcam.yaml` (-WORK) > valores da trena em `MONTAGEM` (`camera_cubo.py`).

> **Atenção:** calibrações salvas antes de 2026-11-19 usavam o solvePnP na tecla K (distância ~11% maior), enquanto o `camera_cubo.py` mede pelo pixel. Isso desloca todas as posições (a pose RViz da pegada fica longe do cubo real). Refaça a calibração. O caminho `camera:=vision` (`cube_vision` do -WORK) ainda calcula a posição pelo solvePnP e pode ter o mesmo desvio.

## 8. Diagnóstico rápido

```bash
source /opt/ros/jazzy/setup.bash
source ~/work/garra/install/setup.bash

# Nós ativos
ros2 node list

# Tópicos ativos
ros2 topic list

# Estado das juntas
ros2 topic echo /joint_states

# Controladores carregados/ativos
ros2 control list_controllers

# Serviços relacionados ao MoveIt
ros2 service list | grep -E 'compute_fk|compute_ik|check_state_validity|compute_cartesian_path'
```

Problemas comuns:

- **Serviços do MoveIt indisponíveis:** confirme que `demo.launch.py` ou `garra_real.launch.py` está ativo e terminou de inicializar.
- **Nenhum `/joint_states`:** confira `joint_state_broadcaster`, controladores e publicação do estado das juntas.
- **Porta serial inexistente:** confira `ls /dev/ttyACM*` e `ls /dev/ttyUSB*`; o nome pode mudar após reconectar o Arduino.
- **Permissão negada:** confira se o usuário pertence a `dialout` e refaça o login.
- **Porta ocupada:** encerre qualquer monitor serial, script de calibração ou nó que já esteja utilizando a porta.
- **Movimento incorreto:** interrompa o sistema, confira montagem, sentido, zero e limites dos servos; não tente compensar erro mecânico apenas alterando limites de software.

## 9. Arquivos importantes

- `garra_final_description/urdf/garra_final.urdf.xacro` — descrição principal do robô.
- `garra_final_description/launch/display.launch.py` — visualização no RViz 2.
- `garra_moveit_config/config/kinematics.yaml` — solver cinemático.
- `garra_moveit_config/config/joint_limits.yaml` — limites articulares para planejamento.
- `garra_moveit_config/config/ros2_controllers.yaml` — configuração dos controladores ROS 2.
- `garra_moveit_config/launch/garra_real.launch.py` — inicialização com hardware físico.
- `garra_coleta/launch/coleta.launch.py` — coleta automática de cubos (inclui os launchers individuais da seção 7.5).
- `garra_moveit_config/config/calibracao_servos.yaml` — conversão entre ângulo de junta e pulso do servo.
- `garra_hardware/src/garra_servo_system.cpp` — interface de hardware `ros2_control`.
- `firmware/garra_servos/garra_servos.ino` — firmware do Arduino.
- `scripts/validar_garra.py` — validação da simulação.
- `scripts/calibrar_servos.py` — calibração física.
- `scripts/calibrar_camera.py` e `~/.config/garra/camera_montagem.json` — pose calibrada da webcam.
- `dimensoes.md` — medidas do robô móvel usadas no URDF e na conversão da borda dianteira.
- `garra_coleta/garra_coleta/pegar_cubo.py`, `garra_coleta/garra_coleta/alcance_cubo.py`, `garra_coleta/garra_coleta/ajuste_base.py`, `garra_coleta/garra_coleta/camera_cubo.py` — coleta de cubos.
- `integracao_robo/` — patch e instruções para o executor do robô chamar a garra.

## 10. Observações de manutenção

- Mantenha `build/`, `install/` e `log/` fora do Git.
- Ao alterar URDF/Xacro, SRDF, limites ou controladores, execute novamente a visualização e a validação em simulação.
- Após alterar arquivos de configuração ou código, recompile o workspace. Com `--symlink-install`, alterações em arquivos não compilados podem ser refletidas sem uma nova cópia, mas alterações em código C++ exigem recompilação.
- Não use os arquivos em `scripts/nao_usados/` como procedimento operacional.
- Faça primeiro os testes em simulação. A validação de planejamento não substitui a calibração, a inspeção mecânica nem os testes controlados de hardware.

## Referências

- [Repositório da garra](https://github.com/Arthur-Demetrio/garra)
- [Documentação ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Documentação MoveIt 2](https://moveit.picknik.ai/main/index.html)
- [Documentação ros2_control](https://control.ros.org/)
