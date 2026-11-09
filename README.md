# Garra Robótica — ROS  2 Jazzy

Projeto de uma garra robótica planar de **três juntas rotativas**, modelada a partir de uma montagem CAD do Fusion 360, descrita em URDF/Xacro e integrada ao **MoveIt 2**. O repositório reúne a descrição do robô, a configuração de planejamento, uma interface de hardware baseada em `ros2_control`, o firmware para Arduino Mega e scripts Python de calibração e validação.

O sistema foi organizado para permitir desenvolvimento e testes em duas etapas:

- **Simulação:** visualizar o modelo no RViz 2, calcular cinemática, verificar colisões e planejar/executar trajetórias com controladores simulados.
- **Hardware físico:** enviar comandos aos três servos por meio de um Arduino Mega conectado por USB serial.

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
│   └── ino atual/
│       └── ...                  # versão anterior do firmware
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
│   │   ├── kinematics.yaml
│   │   ├── joint_limits.yaml
│   │   ├── ros2_controllers.yaml
│   │   ├── moveit_controllers.yaml
│   │   └── calibracao_servos.yaml
│   ├── launch/
│   │   ├── garra_real.launch.py
│   │   └── ...
│   └── ...
├── scripts/
│   ├── cliente_garra.py
│   ├── validar_garra.py
│   ├── calibrar_servos.py
│   └── nao_usados/              # scripts históricos, fora dos procedimentos
├── servo_imgs/                  # imagens e vídeo de apoio à calibração
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

### 2.2 Firmware

O firmware principal está em `firmware/garra_servos/garra_servos.ino`. Ele controla três servos e recebe comandos pela serial a **115200 baud**.

- Arduino Mega;
- pinos 9, 10 e 11 para os três servos;
- pulsos de comando entre 500 e 2500 µs;
- protocolo de texto com verificação XOR;
- resposta `GARRA 1` ao comando de identificação `V`;
- resposta `K` quando um comando de movimento é aceito.

A pasta `firmware/ino atual/` contém uma versão anterior e deve ser tratada como referência histórica; para a montagem descrita no projeto, utilize o firmware em `firmware/garra_servos/`.

### 2.3 Scripts Python

| Arquivo | Finalidade |
|---|---|
| `scripts/cliente_garra.py` | Biblioteca cliente ROS 2 para chamar serviços/ações do MoveIt: FK, IK, verificação de colisão, planejamento e execução de trajetórias. É importada pelos demais scripts; não é um teste executável isoladamente. |
| `scripts/validar_garra.py` | Validação automatizada da descrição e do planejamento em simulação. Verifica TCP, autocolisão, IK, limites, trajetórias cartesianas e planejamento. |
| `scripts/calibrar_servos.py` | Ferramenta interativa que comunica diretamente com o Arduino para medir zero, sentido e limites de pulso dos três servos e gerar `calibracao_servos.yaml`. |

Os comandos completos e a sequência recomendada estão em [`GUIA_TESTES.md`](GUIA_TESTES.md).

## 3. Convenções cinemáticas

A configuração usada pelos scripts e pelo MoveIt é:

| Elemento | Nome |
|---|---|
| Grupo de planejamento | `garra` |
| Link de referência da base | `base_link` |
| TCP (ponta/ferramenta) | `tool0` |
| Junta 1 | `Revolucionar_46` |
| Junta 2 | `Revolucionar_49` |
| Junta 3 | `Revolucionar_52` |

A garra é planar: os três eixos de junta são paralelos. O movimento do TCP ocorre no plano X–Z e a coordenada Y permanece fixa. Portanto, um deslocamento cartesiano em Y não é esperado ser executável.

As posições articulares são expressas em radianos e as posições cartesianas em metros.

## 4. Preparação do ambiente

### 4.1 Dependências

Com ROS 2 Jazzy instalado, instale as ferramentas de compilação e dependências usuais:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  git
```

Instale também os pacotes ROS utilizados pelo projeto. Se o `rosdep` ainda não estiver inicializado na máquina:

```bash
sudo rosdep init
rosdep update
```

`sudo rosdep init` só precisa ser executado uma vez por instalação. Se informar que a inicialização já foi feita, prossiga.

### 4.2 Clonar no workspace

O repositório contém os pacotes ROS diretamente na raiz. Recomenda-se colocá-lo dentro de `src/` de um workspace:

```bash
mkdir -p ~/braco_ws/src
cd ~/braco_ws/src
git clone https://github.com/Arthur-Demetrio/garra.git
```

A estrutura resultante será `~/braco_ws/src/garra/`.

### 4.3 Resolver dependências e compilar

```bash
source /opt/ros/jazzy/setup.bash
cd ~/braco_ws

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

Se a compilação terminar corretamente, o workspace estará disponível no terminal atual. Em cada novo terminal, carregue os ambientes:

```bash
source /opt/ros/jazzy/setup.bash
source ~/braco_ws/install/setup.bash
```

Se o workspace estiver em outro caminho, ajuste o segundo comando.

## 5. Executar a visualização

Para abrir a descrição da garra no RViz 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/braco_ws/install/setup.bash

ros2 launch garra_final_description display.launch.py
```

Esse launch inicia a publicação do modelo e os recursos de visualização/juntas previstos pelo pacote. Use essa etapa para conferir se links, malhas, juntas e TFs estão coerentes antes de testar planejamento.

## 6. Executar o MoveIt 2 em simulação

Em outro terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/braco_ws/install/setup.bash

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
cd ~/braco_ws/src/garra
python3 scripts/calibrar_servos.py --versao 180 --porta /dev/ttyACM0
```

Use `--versao 270` se os servos utilizados forem da faixa mecânica de 270 graus. O script grava os dados em `garra_moveit_config/config/calibracao_servos.yaml` por padrão.

**Não execute o ROS 2/driver de hardware ao mesmo tempo que o script de calibração estiver usando a mesma porta serial.** Consulte o guia de testes para os comandos interativos e os cuidados de segurança.

O arquivo de calibração atualmente versionado contém valores provisórios, limitados a 1400–1600 µs. Não considere esses valores uma calibração mecânica real.

### 7.4 Iniciar o MoveIt com a garra física

Depois de gravar o firmware, calibrar os três servos e conferir a montagem:

```bash
source /opt/ros/jazzy/setup.bash
source ~/braco_ws/install/setup.bash

export GARRA_PORTA=/dev/ttyACM0
ros2 launch garra_moveit_config garra_real.launch.py
```

Para iniciar sem RViz:

```bash
ros2 launch garra_moveit_config garra_real.launch.py use_rviz:=false
```

Para selecionar outro arquivo de calibração:

```bash
export GARRA_CALIBRACAO=/caminho/absoluto/calibracao_servos.yaml
```

O launch configura o plugin `garra_hardware/GarraServoSystem`, inicia o `ros2_control_node`, os controladores e o `move_group`. O driver realiza um handshake com o firmware antes de aceitar comandos.

## 8. Diagnóstico rápido

```bash
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
- `garra_moveit_config/config/calibracao_servos.yaml` — conversão entre ângulo de junta e pulso do servo.
- `garra_hardware/src/garra_servo_system.cpp` — interface de hardware `ros2_control`.
- `firmware/garra_servos/garra_servos.ino` — firmware do Arduino.
- `scripts/validar_garra.py` — validação da simulação.
- `scripts/calibrar_servos.py` — calibração física.

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
