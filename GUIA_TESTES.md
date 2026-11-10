# Guia de testes e scripts — Garra Robótica

Este documento descreve como executar os scripts de teste e calibração existentes em `scripts/`, considerando **Linux Mint + ROS 2 Jazzy + MoveIt 2**.

> Os procedimentos abaixo cobrem somente os scripts ativos documentados neste guia. O conteúdo de `scripts/nao_usados/` foi deliberadamente excluído.

## 1. Scripts considerados

| Script | Tipo | Precisa do MoveIt ativo? | Atua no hardware físico? |
|---|---|---:|---:|
| `cliente_garra.py` | Biblioteca cliente usada pelos testes | Sim, quando importada e utilizada | Não por conta própria |
| `validar_garra.py` | Teste automatizado da descrição, cinemática e planejamento | Sim | **Não. Execute apenas com a simulação.** |
| `calibrar_servos.py` | Calibração interativa dos três servos via serial | Não | **Sim. Atua diretamente no Arduino/servos.** |

`cliente_garra.py` não é um programa de teste independente. Ele implementa a classe `ClienteGarra`, que `validar_garra.py` importa para acessar os serviços e a ação do MoveIt.

## 2. Preparar o terminal

Abra um terminal e carregue o ROS 2 e o workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/braco_ws/install/setup.bash
```

Se o repositório/workspace estiver em outro local, ajuste o caminho do `install/setup.bash`.

Confirme a distribuição:

```bash
echo "$ROS_DISTRO"
```

O resultado esperado é:

```text
jazzy
```

Confirme também que o pacote está disponível:

```bash
ros2 pkg list | grep -E 'garra_final_description|garra_moveit_config|garra_hardware'
```

Se os pacotes não aparecerem, volte à raiz do workspace e compile:

```bash
cd ~/braco_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 2.1 Depois de uma nova exportação do CAD

O exportador gera as juntas como `continuous`, sem limites, com os dedos livres e com um bloco `<ros2_control>` próprio. Antes de compilar, ajuste o URDF exportado:

```bash
python3 scripts/ajustar_urdf.py
```

O script torna J1–J3 `revolute` com limites, prende os dedos (até o J4 ser integrado) e remove o `<ros2_control>` exportado. Se os nomes das juntas mudarem, ele avisa. Depois, refaça a matriz de colisão do SRDF e confira o `tool0`.

## 3. Teste 1 — Visualização do modelo

Esse teste não usa um script Python de `scripts/`; é a primeira verificação recomendada antes de executar os testes automatizados.

```bash
ros2 launch garra_final_description display.launch.py
```

Confira no RViz 2:

- se o modelo aparece completo;
- se as malhas estão carregadas;
- se os elos estão conectados pelas juntas corretas;
- se os eixos e movimentos das juntas fazem sentido;
- se não há erros de TF ou de descrição do robô.

Se a descrição estiver incompleta ou com juntas incorretas, corrija o URDF/Xacro antes de prosseguir.

## 4. Teste 2 — Inicializar o MoveIt em simulação

Em um segundo terminal, carregue os ambientes e inicie o MoveIt:

```bash
source /opt/ros/jazzy/setup.bash
source ~/braco_ws/install/setup.bash

ros2 launch garra_moveit_config demo.launch.py
```

Aguarde o RViz e o `move_group` inicializarem. O script de validação depende dos serviços e da ação disponibilizados pelo MoveIt.

Em um terceiro terminal, confira os serviços:

```bash
source /opt/ros/jazzy/setup.bash
source ~/braco_ws/install/setup.bash

ros2 service list | grep -E \
  'compute_fk|compute_ik|check_state_validity|compute_cartesian_path|plan_kinematic_path'
```

Também é possível conferir a ação de execução:

```bash
ros2 action list | grep execute_trajectory
```

O cliente utiliza estes recursos:

| Recurso | Uso |
|---|---|
| `/compute_fk` | Cinemática direta: juntas → pose do TCP. |
| `/compute_ik` | Cinemática inversa: pose do TCP → juntas. |
| `/check_state_validity` | Verificação de validade/colisão de uma configuração. |
| `/compute_cartesian_path` | Cálculo de trajetória cartesiana. |
| `/plan_kinematic_path` | Planejamento de trajetória articular. |
| `/execute_trajectory` | Execução da trajetória planejada. |
| `/joint_states` | Leitura do estado atual das juntas. |

Se os serviços não aparecerem, não execute o script ainda. Verifique a inicialização do `demo.launch.py`, os logs do `move_group` e se os terminais estão usando o mesmo domínio ROS 2 e configuração DDS.

## 5. Teste 3 — Validação automatizada (`validar_garra.py`)

### 5.1 Objetivo

O script executa uma sequência de verificações da garra **em simulação**:

1. **TCP:** verifica se `tool0` está na posição esperada para a configuração `HOME` e se a terceira junta produz o sentido de movimento esperado.
2. **Autocolisão:** verifica configurações que devem estar livres e configurações que devem ser detectadas como colisão.
3. **IK:** gera configurações sem colisão, calcula a FK e tenta recuperar a posição por IK, avaliando o erro.
4. **Limites de trajetória:** confere se os pontos planejados respeitam os limites das juntas.
5. **Retas cartesianas:** testa deslocamentos em X e Z, verifica desvio da reta e confirma que deslocamento em Y não é compatível com o mecanismo planar.
6. **Execução de trajetória:** executa trajetórias na simulação e mede o erro final.
7. **Planejamento até uma pose:** testa planejamento e execução de uma pose do TCP.

### 5.2 Executar

Com `demo.launch.py` ainda ativo em outro terminal:

```bash
cd ~/braco_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 src/garra/scripts/validar_garra.py
```

Se você estiver executando a partir da raiz do próprio repositório (e não do workspace), o comando equivalente é:

```bash
python3 scripts/validar_garra.py
```

O script imprime as seções de teste e os resultados `[PASSOU]` ou `[FALHOU]`. Ao final:

- `TUDO PASSOU`: todas as verificações contabilizadas passaram; o processo retorna código 0.
- `N VERIFICACAO(OES) FALHARAM`: uma ou mais verificações falharam; o processo retorna código 1.

Para consultar o código de saída imediatamente após a execução:

```bash
echo $?
```

### 5.3 Critérios importantes usados pelo teste

Os limites abaixo são parâmetros internos do script, não garantias de precisão física:

| Verificação | Critério implementado |
|---|---|
| Posição do TCP em `HOME` | Erro menor que 1 mm em relação à pose de referência do código. |
| IK | Pelo menos 95% das 100 amostras válidas devem ser resolvidas dentro de 1 mm de erro de posição. |
| Reta cartesiana | Fração planejada de pelo menos 99,9% e desvio máximo menor que 1 mm. |
| Execução de juntas | Erro final máximo menor que 0,03 rad. |
| Execução cartesiana | Erro final do TCP menor que 2 mm nos movimentos de ida/volta; o teste de pose usa tolerância de 3 mm. |

A busca de IK usa uma semente próxima da configuração original. O solver KDL pode falhar em algumas amostras, especialmente próximo dos limites; o teste admite uma taxa mínima de sucesso de 95%.

### 5.4 O que observar no RViz

Durante a execução, acompanhe a garra no RViz:

- o modelo deve se mover de acordo com as trajetórias;
- não deve atravessar superfícies ou volumes configurados como colisão;
- os movimentos cartesianos devem ocorrer no plano X–Z;
- o teste pode retornar a garra a posições de referência, como `HOME` e `PRONTA`.

O script **executa movimentos**, não apenas calcula trajetórias. Por isso, mantenha o teste restrito ao ambiente simulado.

### 5.5 Se ficar aguardando os serviços

A mensagem inicial do script é semelhante a:

```text
Aguardando os servicos e a acao do MoveIt...
```

Se permanecer nessa etapa:

1. confirme que `ros2 launch garra_moveit_config demo.launch.py` continua ativo;
2. confira `ros2 service list` e `ros2 action list`;
3. verifique se `/compute_fk`, `/compute_ik`, `/check_state_validity`, `/compute_cartesian_path`, `/plan_kinematic_path` e `/execute_trajectory` estão disponíveis;
4. confira os logs do `move_group`;
5. confirme que o terminal do script carregou o mesmo workspace e ROS 2 Jazzy.

Se o script falhar em uma verificação específica, examine o nome da seção e os valores impressos. Uma falha de IK, por exemplo, pode estar relacionada ao alcance, limites, colisões ou tolerância; não significa automaticamente que o modelo inteiro está incorreto.

## 6. Biblioteca cliente (`cliente_garra.py`)

Esse arquivo não deve ser executado diretamente. Ele fornece a classe `ClienteGarra`, utilizada pelo validador e por outros programas Python que precisem interagir com o MoveIt.

A classe utiliza:

- grupo de planejamento: `garra`;
- base: `base_link`;
- TCP: `tool0`;
- juntas: `Revolucionar_46`, `Revolucionar_49` e `Revolucionar_72`.

Métodos principais:

| Método | Função |
|---|---|
| `aguardar()` | Aguarda serviços e ação do MoveIt. |
| `juntas_atuais()` | Lê as posições de `/joint_states`. |
| `fk(q)` | Calcula a pose do TCP para uma configuração articular. |
| `ik(pose, semente)` | Calcula uma configuração articular para uma pose. |
| `valido(q)` | Verifica se a configuração está livre de colisões. |
| `linha_cartesiana(q_ini, deslocamento)` | Calcula uma reta cartesiana mantendo a orientação inicial do TCP. |
| `planejar_juntas(q_ini, q_alvo)` | Planeja uma trajetória entre configurações articulares. |
| `planejar_pose(q_ini, pose)` | Resolve IK e planeja até uma pose. |
| `executar(trajetoria)` | Envia uma trajetória para execução pelo MoveIt. |

A função `linha_cartesiana()` considera o mecanismo planar. Um deslocamento em Y não deve completar a trajetória. O passo cartesiano padrão é 0,001 m e a escala padrão de velocidade/aceleração é 0,1.

Ao criar novos testes, importe a biblioteca a partir de um script localizado no mesmo diretório, como ocorre em `validar_garra.py`. Evite duplicar a lógica de comunicação com os serviços do MoveIt.

## 7. Teste 4 — Calibração física (`calibrar_servos.py`)

<badge color="warning">Hardware físico — cuidado</badge>

### 7.1 Objetivo

O script conversa diretamente com o firmware do Arduino Mega para medir, em cada junta:

- `zero`: pulso correspondente à posição zero definida no CAD;
- `min`: limite inferior seguro medido;
- `max`: limite superior seguro medido;
- `sentido`: relação entre aumento do pulso e aumento do ângulo articular.

Ao final, gera `garra_moveit_config/config/calibracao_servos.yaml`.

### 7.2 Cuidados obrigatórios antes de executar

1. **Não inicie `garra_real.launch.py` nem qualquer nó que esteja usando a porta serial.**
2. Deixe o elo que será calibrado livre para se mover, sem atingir batentes, cabos ou outros elos.
3. Tenha acesso imediato ao interruptor/fonte de alimentação dos servos.
4. Confira a alimentação externa dos servos e o GND comum com o Arduino.
5. Identifique corretamente a porta serial.
6. Faça movimentos pequenos e observe o sentido mecânico. Pare se houver ruído, esforço ou travamento.

Abrir a porta serial pode reiniciar o Arduino Mega. O script aguarda o boot e procura a resposta `GARRA 1` antes de enviar comandos.

### 7.3 Identificar a porta

```bash
ls /dev/ttyACM*
ls /dev/ttyUSB*
```

Exemplo de porta: `/dev/ttyUSB0` (Mega com conversor CH340). Um Mega original aparece como `/dev/ttyACM0`; confira com `ls /dev/serial/by-id`.

Se houver erro de permissão, confira:

```bash
groups
```

Se `dialout` não aparecer:

```bash
sudo usermod -aG dialout "$USER"
```

Encerre a sessão e entre novamente para aplicar a alteração.

### 7.4 Executar

Para servos de 180 graus:

```bash
cd ~/braco_ws/src/garra
python3 scripts/calibrar_servos.py --versao 180 --porta /dev/ttyUSB0
```

Para servos de 270 graus:

```bash
python3 scripts/calibrar_servos.py --versao 270 --porta /dev/ttyUSB0
```

Parâmetros:

| Parâmetro | Obrigatório | Descrição |
|---|---:|---|
| `--versao` | Sim | `180` ou `270`, conforme a faixa do servo considerada na conversão. |
| `--porta` | Não | Porta serial do Arduino. Padrão do script: `/dev/ttyUSB0`. |
| `--saida` | Não | Caminho do YAML gerado. Por padrão, aponta para `garra_moveit_config/config/calibracao_servos.yaml`. |

Para ver a ajuda:

```bash
python3 scripts/calibrar_servos.py --help
```

### 7.5 Comandos interativos

Depois que o script reconhecer o firmware, ele apresenta os comandos:

| Comando | Ação |
|---|---|
| `1,90` | Formato `<motor>,<angulo>` (igual ao Monitor Serial): seleciona o motor e leva ao ângulo de 0 a 180°. Salta direto ao destino (a ~25°/s); confira o caminho antes. |
| `j 1` | Seleciona a junta 1 (`Revolucionar_46`). |
| `j 2` | Seleciona a junta 2 (`Revolucionar_49`). |
| `j 3` | Seleciona a junta 3 (`Revolucionar_72`). |
| `+` | Aumenta o pulso em 10 µs (padrão). |
| `+ 5` | Aumenta o pulso em 5 µs. |
| `-` | Diminui o pulso em 10 µs (padrão). |
| `- 5` | Diminui o pulso em 5 µs. |
| `zero` | Registra o pulso atual como zero articular. |
| `min` | Registra o limite inferior medido. |
| `max` | Registra o limite superior medido. |
| `sentido +` | Indica que o ângulo do modelo aumenta quando o pulso aumenta. |
| `sentido -` | Indica que o ângulo do modelo diminui quando o pulso aumenta. |
| `estado` | Mostra as medições feitas até o momento. |
| `salvar` | Grava o YAML se todas as medições forem válidas. |
| `sair` | Encerra a calibração. |

O incremento por comando é limitado a 50 µs. O script também limita o pulso absoluto à faixa de 500 a 2500 µs, mas esses limites elétricos **não substituem os limites mecânicos reais** do conjunto.

### 7.6 Sequência de calibração recomendada

Repita o procedimento para cada uma das três juntas:

1. Selecione a junta (`j 1`, `j 2` ou `j 3`).
2. Use `<motor>,<angulo>` para chegar perto da referência (ex.: `1,90`) e incrementos pequenos (`+ 5`, `- 5`) para aproximar o elo da posição zero definida no CAD.
3. Digite `zero` quando a junta estiver alinhada à referência mecânica.
4. Determine o sentido: se o ângulo positivo do modelo aumenta quando o pulso aumenta, registre `sentido +`; caso contrário, `sentido -`.
5. Meça o limite inferior seguro sem forçar a estrutura e registre `min`.
6. Meça o limite superior seguro sem forçar a estrutura e registre `max`.
7. Use `estado` para revisar as quatro medições.
8. Repita nas juntas restantes.
9. Digite `salvar` somente quando as três juntas estiverem completas e os valores forem coerentes.

O script só grava o arquivo se todas as juntas tiverem `zero`, `min`, `max` e `sentido`, com `min < max` e zero dentro da faixa.

**Não use o limite de pulso do código como autorização para levar o servo até o batente.** A faixa segura depende da montagem, dos cabos, dos suportes e da geometria real.

### 7.7 Depois de salvar

Confira o arquivo:

```bash
cat garra_moveit_config/config/calibracao_servos.yaml
```

O formato esperado é semelhante a:

```yaml
juntas:
  Revolucionar_46: {zero_us: 1472, us_por_rad: -590.6, min_us: 1400, max_us: 1600}
  Revolucionar_49: {zero_us: 1472, us_por_rad: -590.6, min_us: 1400, max_us: 1600}
  Revolucionar_72: {zero_us: 1472, us_por_rad: -590.6, min_us: 1400, max_us: 1600}
```

Os números acima são apenas ilustrativos e **não devem ser copiados como calibração real**. O arquivo atualmente versionado no repositório está explicitamente marcado como provisório.

Se a calibração foi salva no caminho padrão, o launch físico poderá utilizá-la. Se foi salva em outro caminho:

```bash
export GARRA_CALIBRACAO=/caminho/absoluto/calibracao_servos.yaml
```

Se não estiver usando `--symlink-install`, recompile o workspace para atualizar a cópia instalada do arquivo de configuração.

## 8. Inicialização do hardware após a calibração

Somente depois de concluir a calibração, conferir o firmware e verificar a mecânica:

```bash
source /opt/ros/jazzy/setup.bash
source ~/braco_ws/install/setup.bash

export GARRA_PORTA=/dev/ttyUSB0
ros2 launch garra_moveit_config garra_real.launch.py
```

Sem RViz:

```bash
ros2 launch garra_moveit_config garra_real.launch.py use_rviz:=false
```

O launch inicia o `ros2_control_node`, os controladores e o `move_group`, usando o plugin `garra_hardware/GarraServoSystem`.

Não execute o calibrador e o launch de hardware simultaneamente: ambos precisam da mesma porta serial.

## 9. Diagnóstico de falhas

### 9.1 `Aguardando os servicos e a acao do MoveIt...`

O `validar_garra.py` não encontrou todos os serviços/ações necessários.

- confira se `demo.launch.py` está ativo;
- aguarde a inicialização completa;
- confira `ros2 service list` e `ros2 action list`;
- verifique os logs do `move_group`;
- confirme o `source` do Jazzy e do workspace em todos os terminais.

### 9.2 `Nada publicado em /joint_states`

O cliente não recebeu o estado das juntas dentro do tempo limite.

```bash
ros2 topic echo /joint_states
ros2 control list_controllers
```

Confira se o `joint_state_broadcaster` está ativo e se os controladores foram inicializados corretamente.

### 9.3 `Permission denied` na serial

```bash
groups
ls -l /dev/ttyUSB0
```

Confirme o grupo `dialout`, adicione o usuário se necessário e faça logout/login.

### 9.4 `Device or resource busy`

Outro programa está utilizando a porta serial. Feche Arduino Serial Monitor, `screen`, `minicom`, outro calibrador ou o launch ROS que controla o Arduino.

### 9.5 `O firmware nao respondeu`

- confira a porta serial;
- confirme que o firmware correto foi gravado;
- confira a velocidade serial de 115200 baud;
- aguarde o boot do Arduino;
- confirme alimentação e conexão USB.

### 9.6 Falha de planejamento ou IK

Confira:

- se a pose está dentro do alcance;
- se a configuração não viola limites;
- se o estado não está em colisão;
- se o TCP e os nomes de juntas coincidem com URDF/SRDF;
- se o solver cinemático está carregado.

Uma fração cartesiana menor que 100% pode indicar limite articular, colisão, alcance insuficiente ou uma solicitação fora do plano de movimento.

## 10. Sequência recomendada de validação

Use esta ordem para reduzir o risco de testar várias camadas ao mesmo tempo:

1. Compilar o workspace sem erros.
2. Abrir `display.launch.py` e validar a descrição no RViz.
3. Abrir `demo.launch.py` e conferir os serviços do MoveIt.
4. Executar `validar_garra.py` somente na simulação.
5. Resolver falhas de cinemática, colisão, limites ou planejamento.
6. Gravar o firmware no Arduino Mega.
7. Calibrar individualmente os três servos com `calibrar_servos.py`.
8. Revisar o YAML gerado e os limites físicos.
9. Iniciar `garra_real.launch.py` com a estrutura em condição segura e velocidade/escala reduzida quando aplicável.
10. Fazer movimentos físicos pequenos e supervisionados antes de qualquer operação completa.

A validação em simulação confirma aspectos do modelo e do planejamento. Ela não comprova, por si só, que os servos estão calibrados, que a estrutura suporta os movimentos ou que a alimentação elétrica é adequada.
