# Garra robótica - ROS 2 Jazzy

Projeto de uma garra robótica planar de três juntas, modelada em URDF/Xacro e
integrada ao MoveIt 2. O projeto pode ser executado em simulação ou conectado
a três servos controlados por um Arduino Mega via USB serial.

## Requisitos

- Ubuntu com [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- `colcon`
- MoveIt 2
- `ros2_control` e `ros2_controllers`
- Arduino IDE (apenas para gravar o firmware)
- Arduino Mega conectado aos servos, para operação física

## Estrutura do projeto

| Diretório | Conteúdo |
|---|---|
| `garra_final_description/` | Modelo do robô, URDF/Xacro, meshes, TFs e launch de visualização |
| `garra_moveit_config/` | Configuração do MoveIt 2, cinemática, limites, RViz e controladores |
| `garra_hardware/` | Plugin `ros2_control` que comunica com o Arduino Mega pela serial |
| `firmware/garra_servos/` | Firmware principal para os três servos |
| `firmware/ino atual/` | Versão anterior do firmware |
| `scripts/` | Cliente MoveIt, calibração dos servos e validação da simulação |
| `servo_imgs/` | Imagens e vídeo de apoio à calibração |

Os diretórios `build/`, `install/` e `log/` são gerados pelo ROS 2 e não
precisam ser versionados. Os scripts em `scripts/nao_usados/` são históricos e
não fazem parte dos procedimentos deste README.

## Inicialização do projeto

No terminal, a partir da raiz deste repositório:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Para carregar automaticamente o ambiente em novos terminais:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source $PWD/install/setup.bash" >> ~/.bashrc
```

Se o workspace for movido, prefira executar os dois comandos `source`
manualmente ou atualize o caminho no `~/.bashrc`.

## Simulação

### Visualização do modelo

Executa o `robot_state_publisher`, os controles de juntas e o RViz 2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch garra_final_description display.launch.py
```

### MoveIt 2

Executa o MoveIt com hardware simulado:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch garra_moveit_config demo.launch.py
```

Em outro terminal, com o MoveIt em execução, a validação automatizada pode ser
executada com:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/validar_garra.py
```

O script primeiro exibe que está aguardando os serviços do MoveIt. Se essa
mensagem não aparecer, confira se o comando foi executado corretamente. Se ele
ficar aguardando nessa etapa, inicie o `demo.launch.py` e verifique se os
serviços e a ação do MoveIt estão disponíveis.

O teste verifica FK, IK, limites, autocolisão, planejamento e trajetórias
cartesianas. Ele deve ser executado somente com a simulação.

## Operação com a garra física

### 1. Gravar o firmware

Abra `firmware/garra_servos/garra_servos.ino` na Arduino IDE, selecione a placa
Arduino Mega e a porta correspondente, instale a biblioteca `Servo` caso
necessário e grave o sketch.

O firmware usa:

- baud rate `115200`;
- pinos 9, 10 e 11 para os três servos;
- protocolo serial com comandos de pulso em microssegundos;
- faixa elétrica de 500 a 2500 us.

Garanta alimentação adequada e uma conexão comum de GND entre o Arduino e os
servos.

### 2. Calibrar os servos

Não execute o ROS 2 enquanto o script de calibração estiver usando a mesma
porta serial. Para calibrar:

```bash
python3 scripts/calibrar_servos.py --versao 180 --porta /dev/ttyUSB0
```

Use `--versao 270` para servos com faixa mecânica de 270 graus. O programa
grava os valores em
`garra_moveit_config/config/calibracao_servos.yaml`. A calibração só é salva
quando todas as juntas possuem zero, mínimo, máximo e sentido medidos.

O arquivo atualmente versionado contém valores provisórios e limitados a
1400-1600 us. Faça a calibração real antes de movimentar a estrutura.

### 3. Iniciar o hardware e o MoveIt

Conecte o Arduino e confira a porta:

```bash
ls /dev/ttyUSB*
```

Depois de compilar e carregar o ambiente:

```bash
export GARRA_PORTA=/dev/ttyUSB0
ros2 launch garra_moveit_config garra_real.launch.py
```

Para iniciar sem RViz:

```bash
ros2 launch garra_moveit_config garra_real.launch.py use_rviz:=false
```

O launch inicia o `ros2_control_node`, os controladores, o `move_group` e,
por padrão, o RViz 2. O plugin de hardware faz o handshake `GARRA 1` antes de
aceitar comandos. Também é possível informar outro arquivo de calibração:

```bash
export GARRA_CALIBRACAO=/caminho/para/calibracao_servos.yaml
```

## Pacotes e arquivos importantes

- `garra_final_description/urdf/garra_final.urdf.xacro`: entrada principal do
  modelo.
- `garra_final_description/launch/display.launch.py`: visualização do modelo.
- `garra_moveit_config/config/kinematics.yaml`: solver cinemático.
- `garra_moveit_config/config/joint_limits.yaml`: limites de planejamento.
- `garra_moveit_config/config/ros2_controllers.yaml`: controlador de trajetória.
- `garra_moveit_config/launch/garra_real.launch.py`: execução com o Arduino.
- `garra_hardware/src/garra_servo_system.cpp`: implementação da interface
  `ros2_control`.
- `scripts/cliente_garra.py`: cliente Python para FK, IK, planejamento e
  execução.

O grupo do MoveIt é `garra`, o TCP é `tool0`, a base é `base_link` e as juntas
controladas são `Revolucionar_46`, `Revolucionar_49` e `Revolucionar_52`.

## Diagnóstico rápido

```bash
# Juntas publicadas
ros2 topic echo /joint_states

# Controladores ativos
ros2 control list_controllers

# Nós ativos
ros2 node list
```

Se o hardware não iniciar, verifique a porta serial, as permissões do usuário
(`dialout`), a alimentação dos servos, o firmware gravado e a presença do
arquivo de calibração.
