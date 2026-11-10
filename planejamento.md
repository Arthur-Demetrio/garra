# Planejamento de Desenvolvimento e Integração da Garra Robótica

## Observações:
este arquivo é só um guia e não necessáriamente será seguido 100%

**Projeto:** Garra Robótica — RoboCup@Work  
**Ambiente de desenvolvimento:** Linux Mint 22.3  
**Middleware:** ROS 2 Jazzy  
**Plataforma de controle:** Arduino Mega  
**Planejamento de movimentos:** MoveIt 2  
**Controle de hardware:** ros2_control  
**Status:** em desenvolvimento — Fase 1, teste físico de J1–J3

---

# 0. Situação atual e próximos passos (atualizado em 10/11/2026)

**Concluído:**

- Novo URDF exportado do CAD e ajustado com `scripts/ajustar_urdf.py` (J3 paralelo a J1/J2; dedos fixos até o J4 ser integrado).
- Simulação validada com `scripts/validar_garra.py`, incluindo a estrutura do robô, a placa da base (1 cm) e o chão.
- Firmware `firmware/garra_servos/garra_servos.ino` gravado no Arduino Mega e testado pelo Monitor Serial (115200 baud).
- Calibração de J1–J3 com `scripts/calibrar_servos.py --versao 180`, gravada em `garra_moveit_config/config/calibracao_servos.yaml`.
- Limites das juntas no URDF ajustados para a faixa calibrada: J1 −0,85 a 1,55 rad; J2 −1,55 a 1,48 rad; J3 −1,52 a 1,55 rad.

**Próximos passos, em ordem:**

1. Rodar `validar_garra.py` na simulação com os limites novos.
2. Primeiro movimento real com `garra_real.launch.py`, só pelo RViz, com velocidade 0.1 e alvos próximos da pose inicial (vertical). Conferir se o braço real acompanha o RViz em sentido e amplitude.
3. Medir o Y dos suportes do 1º andar e o volume do chassi sob a placa da base, para completar o modelo de colisão.
4. Depois: conectar e integrar o J4 (dedos).

Os testes físicos iniciais usam só J1, J2 e J3. Firmware, driver e URDF ainda não cobrem o J4.

---

# 1. Objetivo do projeto

Desenvolver uma garra robótica capaz de executar movimentos de maneira independente, incluindo calibração, acionamento dos servomotores, planejamento de trajetórias e validação mecânica.

Após a validação individual, a garra deverá ser integrada ao robô completo da equipe.

A integração deverá permitir que o computador central do robô solicite operações de manipulação por meio do ROS 2, sem precisar conhecer os detalhes internos do acionamento dos servos.

O objetivo final é permitir que o robô:

1. Identifique cubos por meio de uma câmera e seus QR Codes.
2. Obtenha a identificação e a posição espacial dos cubos.
3. Encaminhe essas informações ao sistema de manipulação.
4. Planeje e execute a aproximação e a coleta dos cubos.
5. Transporte os objetos até posições de destino definidas pelo computador central.
6. Informe o resultado das operações ao sistema principal.

O desenvolvimento será dividido em três fases:

- **Fase 1:** desenvolvimento e validação da garra independente.
- **Fase 2:** integração da garra à arquitetura ROS 2.
- **Fase 3:** manipulação autônoma, integrando percepção, coleta e transporte de objetos.

---

# 2. Arquitetura mecânica

A garra possui quatro eixos de movimentação, controlados por quatro servomotores.

| Junta | Identificação | Função | Situação |
|---|---|---|---|
| J1 | `Revolucionar_46` | Movimento do primeiro segmento do braço | Em desenvolvimento |
| J2 | `Revolucionar_49` | Movimento do segundo segmento do braço | Em desenvolvimento |
| J3 | `Revolucionar_72` | Movimento do pulso, permitindo inclinar a garra para cima e para baixo | Em desenvolvimento |
| J4 | `dedos` | Abertura e fechamento da garra | Integrado ao ROS; falta teste na garra real |

J1, J2 e J3 usam servos DS3218MG de 180° nos pinos 9, 10 e 11 do Arduino Mega. Os três eixos são paralelos; com os servos em 90° (q = 0) a garra fica na vertical.

O J4 usa um micro servo DS-S006MP (engrenagem metálica, 4,8 a 6 V, ~90° de curso útil) no pino 12. Medido pelo Monitor Serial: 0° = aberta, 95° = presa no cubo, ~90–100° = fechada. No ROS ele é a junta `dedos` (valor = ângulo do comando em rad, não o ângulo real dos dedos), com o controlador `garra_dedos_controller` (GripperCommand) e o grupo `dedos` do MoveIt (estados `aberta` e `fechada`). Os dedos da malha continuam fixos no modelo até o CAD dos dedos ser finalizado.

O J4 utilizará um modelo de servo diferente dos três primeiros. Sua integração será realizada separadamente, após a identificação de suas características elétricas e mecânicas.

**Observação:** a montagem física atual ainda não corresponde integralmente ao CAD e ao URDF existentes. Os modelos digitais deverão ser atualizados após a finalização da mecânica.

---

# 3. Fase 1 — Desenvolvimento da garra independente

O objetivo desta fase é fazer a garra funcionar corretamente sem depender do computador central do robô.

## 3.1. Levantamento e documentação do hardware

- [ ] Identificar os modelos exatos dos quatro servomotores.
- [ ] Verificar tensão e corrente de alimentação.
- [ ] Confirmar a capacidade da fonte externa.
- [ ] Documentar as conexões entre Arduino, servos e alimentação.
- [x] Confirmar os pinos utilizados por cada servo (J1–J3: 9, 10 e 11; J4: 12).
- [ ] Verificar o aterramento comum entre Arduino e fonte.
- [ ] Identificar limitações mecânicas de cada eixo.

**Resultado esperado:** documentação confiável do hardware e das conexões elétricas.

## 3.2. Testes individuais dos servomotores

Inicialmente, serão testados J1, J2 e J3, que utilizam o mesmo modelo de servo.

- [x] Verificar a comunicação serial entre computador e Arduino.
- [x] Testar individualmente cada servo.
- [x] Identificar o sentido de movimentação.
- [x] Determinar limites seguros de pulso.
- [ ] Verificar vibrações, aquecimento e comportamento sob carga.
- [x] Registrar parâmetros provisórios de calibração.

Posteriormente, repetir o procedimento para J4, considerando suas especificações próprias.

**Resultado esperado:** todos os servos respondendo corretamente aos comandos, sem ultrapassar limites elétricos ou mecânicos seguros.

## 3.3. Desenvolvimento e validação do firmware

O Arduino será responsável pelo acionamento direto dos servomotores.

- [x] Revisar a implementação atual do firmware.
- [x] Organizar o protocolo de comunicação serial.
- [x] Implementar ou revisar os comandos individuais de cada servo.
- [ ] Validar o recebimento e a interpretação dos comandos.
- [x] Verificar mecanismos de proteção contra comandos inválidos (checksum e faixa de pulso).
- [ ] Implementar tratamento de falhas de comunicação.
- [x] Avaliar o comportamento seguro na perda de comunicação (os servos mantêm a última posição).
- [x] Integrar o J4 ao firmware.

**Resultado esperado:** firmware estável, capaz de controlar os quatro servos de forma independente e previsível.

## 3.4. Calibração dos servomotores

A calibração deverá separar duas questões:

- **Calibração do servo:** relação entre os pulsos enviados e a posição angular do eixo.
- **Calibração mecânica da junta:** relação entre a posição do servo e o movimento real da articulação.

Atividades:

- [x] Determinar limites seguros de cada servo.
- [x] Identificar a posição de referência de cada junta.
- [x] Determinar o sentido positivo de movimentação.
- [ ] Medir a relação entre pulso e ângulo.
- [x] Registrar parâmetros em arquivo de configuração.
- [ ] Validar os resultados com a montagem mecânica definitiva.

Resultados obtidos antes da finalização mecânica deverão ser identificados como provisórios.

## 3.5. Finalização do modelo mecânico

- [ ] Finalizar as articulações no Fusion 360.
- [ ] Conferir os eixos de rotação.
- [ ] Conferir a posição dos servomotores.
- [ ] Finalizar o mecanismo de abertura e fechamento.
- [ ] Verificar interferências entre componentes.
- [ ] Definir o sistema de referência da ferramenta.
- [ ] Atualizar o conjunto CAD conforme a montagem física.

**Resultado esperado:** montagem física e modelo CAD representando a mesma configuração mecânica.

## 3.6. Atualização do URDF

Após a finalização do CAD:

- [x] Exportar novamente a descrição da garra.
- [ ] Revisar links e joints.
- [ ] Configurar corretamente os quatro eixos.
- [x] Definir limites de posição e velocidade (J1–J3).
- [ ] Revisar os sistemas de coordenadas.
- [x] Configurar corretamente o `tool0`.
- [ ] Representar o mecanismo de abertura e fechamento do J4.
- [ ] Validar a descrição no RViz.

Quando a mecânica dos dedos permitir, avaliar o uso de `mimic joint` para representar movimentos acoplados.

## 3.7. Configuração e validação do MoveIt 2

- [ ] Atualizar o grupo de planejamento `garra`.
- [ ] Configurar os quatro eixos.
- [x] Revisar os limites das juntas (J1–J3, conforme a calibração).
- [ ] Validar a cinemática direta.
- [ ] Validar a cinemática inversa.
- [ ] Testar movimentos individuais.
- [ ] Testar trajetórias envolvendo múltiplas juntas.
- [ ] Validar movimentos cartesianos.
- [ ] Testar abertura e fechamento da garra.
- [ ] Validar a execução com o hardware real.

**Resultado esperado:** capacidade de planejar e executar movimentos da garra utilizando MoveIt 2, respeitando os limites mecânicos.

---

# 4. Fase 2 — Integração da garra com o ROS 2

O objetivo desta fase é transformar a garra em um subsistema ROS 2 que possa ser utilizado por outros componentes do robô.

A garra não deverá depender de comandos manuais ou de conhecimento dos detalhes internos do Arduino.

## 4.1. Arquitetura proposta

O computador central será responsável por coordenar as tarefas do robô. A garra disponibilizará uma interface ROS 2 para receber solicitações de manipulação e informar seus estados.

Arquitetura inicial:

```text
              ROBÔ COMPLETO
                    |
                    v
           COMPUTADOR CENTRAL
                    |
             Gerenciador de tarefas
                    |
                    v
           Coordenador da garra
                    |
              MoveIt 2
                    |
              ros2_control
                    |
           Interface de hardware
                    |
                Arduino
                    |
          +---------+---------+
          |         |         |
          J1        J2        J3        J4
```

O coordenador da garra deverá abstrair os detalhes de planejamento, acionamento e comunicação com o hardware.

## 4.2. Desenvolvimento da interface ROS 2

A interface deverá permitir:

- [ ] Receber solicitações de movimentação.
- [ ] Receber posições de destino.
- [ ] Solicitar abertura e fechamento da garra.
- [ ] Solicitar movimentos de aproximação e coleta.
- [ ] Informar o estado atual da operação.
- [ ] Informar falhas de planejamento ou execução.
- [ ] Retornar o resultado das operações.

A interface deverá ser projetada para que o computador central solicite operações de alto nível, sem enviar diretamente comandos para cada servo.

## 4.3. Definição das interfaces de comunicação

Avaliar a utilização dos seguintes recursos ROS 2:

**Topics**
- Publicação dos estados das juntas.
- Publicação de informações de diagnóstico.
- Recebimento de dados de percepção, quando necessário.

**Services**
- Operações simples e rápidas.
- Solicitações de configuração ou habilitação.

**Actions**
- Movimentos que possuem duração.
- Operações de coleta e transporte.
- Acompanhamento do progresso.
- Retorno de sucesso ou falha.
- Possibilidade de cancelamento.

A escolha final deverá considerar as necessidades do computador central e evitar interfaces redundantes.

## 4.4. Integração com ros2_control

- [ ] Integrar os quatro servos ao sistema de hardware.
- [ ] Configurar os controladores necessários.
- [ ] Validar a publicação de `/joint_states`.
- [ ] Validar a comunicação com MoveIt 2.
- [ ] Testar comandos utilizando interfaces ROS 2.
- [ ] Verificar o comportamento em caso de desconexão.
- [ ] Validar inicialização e encerramento dos nós.

**Atenção:** caso o hardware não possua sensores de posição, os valores publicados em `/joint_states` poderão representar posições estimadas a partir dos comandos enviados, e não medições reais dos eixos.

---

# 5. Fase 3 — Manipulação autônoma e integração com a percepção

Esta fase representa o principal objetivo da integração: permitir que o robô identifique cubos, alcance suas posições, realize a coleta e transporte os objetos até os locais determinados pelo computador central.

## 5.1. Integração com o sistema de visão

O robô já possui um sistema de identificação de cubos por QR Codes, utilizando uma câmera.

Informações confirmadas:

- A identificação dos cubos já é realizada.
- O sistema utiliza QR Codes.
- A câmera fornece coordenadas espaciais X, Y e Z.
- A comunicação entre o computador central e os demais sistemas ocorre por ROS 2.

Ainda será necessário investigar como essas coordenadas são representadas e publicadas.

Atividades:

- [ ] Identificar os nós ROS 2 responsáveis pela câmera e leitura dos QR Codes.
- [ ] Identificar os tópicos publicados pelo sistema de visão.
- [ ] Verificar qual mensagem ROS 2 transporta as informações.
- [ ] Identificar como o ID do QR Code é associado às coordenadas.
- [ ] Identificar o referencial em que X, Y e Z são calculados.
- [ ] Verificar se existe timestamp associado à detecção.
- [ ] Verificar se a câmera fornece orientação do cubo.
- [ ] Documentar a frequência de atualização das posições.

**Ponto pendente:** determinar se as coordenadas fornecidas pela câmera estão no referencial da própria câmera, da base do robô ou de outro sistema de referência.

## 5.2. Transformação entre referenciais

As coordenadas da câmera precisam ser convertidas para um referencial compatível com o planejamento da garra.

Será utilizado TF2 para realizar as transformações necessárias.

Exemplo conceitual:

```text
camera_link
    |
    | Transformação TF2
    v
base_link
    |
    v
Referencial de planejamento da garra
    |
    v
tool0
```

Atividades:

- [ ] Identificar os referenciais existentes no robô.
- [ ] Verificar a posição e orientação da câmera em relação ao robô.
- [ ] Validar a transformação entre câmera e base do robô.
- [ ] Verificar a calibração extrínseca da câmera.
- [ ] Testar a conversão de coordenadas.
- [ ] Comparar posições calculadas com posições físicas conhecidas.

**Resultado esperado:** a posição de um cubo identificada pela câmera deverá ser convertida corretamente para o referencial utilizado pelo MoveIt 2.

## 5.3. Desenvolvimento do sistema de coleta

O sistema deverá receber a identificação e a posição de um cubo e executar uma sequência de manipulação.

Fluxo proposto:

1. Receber a identificação e a posição do cubo.
2. Validar se os dados recebidos são utilizáveis.
3. Converter a posição para o referencial do robô.
4. Determinar a orientação adequada para a coleta.
5. Calcular uma posição de aproximação.
6. Planejar a trajetória até a posição de aproximação.
7. Aproximar a garra do cubo.
8. Posicionar J3 conforme a orientação necessária.
9. Acionar J4 para fechar a garra.
10. Verificar, quando houver sensores adequados, se o cubo foi realmente agarrado.
11. Retornar o resultado da coleta.

Atividades:

- [ ] Definir a estratégia de aproximação.
- [ ] Definir a orientação de coleta.
- [ ] Configurar o planejamento cartesiano.
- [ ] Definir a abertura inicial da garra.
- [ ] Definir a posição de fechamento do J4.
- [ ] Avaliar a necessidade de sensores para confirmar a coleta.
- [ ] Testar a coleta com cubos em posições conhecidas.
- [ ] Testar a coleta com posições recebidas pela câmera.

## 5.4. Desenvolvimento do sistema de transporte

Após coletar um cubo, o robô deverá transportá-lo até uma posição determinada pelo computador central.

A definição de como o destino será informado ainda está pendente.

Possibilidades que deverão ser avaliadas:

- Recebimento de coordenadas X, Y e Z.
- Recebimento de um identificador de estação ou local.
- Recebimento de um identificador que será convertido em coordenadas por outro componente do robô.

Atividades:

- [ ] Definir como o computador central representa o destino.
- [ ] Definir o referencial das posições de destino.
- [ ] Implementar o recebimento da solicitação.
- [ ] Planejar a trajetória até o destino.
- [ ] Transportar o cubo mantendo uma configuração segura.
- [ ] Abrir J4 para liberar o objeto.
- [ ] Confirmar o resultado da operação.
- [ ] Retornar o estado ao computador central.

## 5.5. Coordenador de manipulação

Será desenvolvido um componente responsável por coordenar as etapas de coleta e transporte.

Sua função será receber solicitações do computador central e organizar a execução das operações.

Responsabilidades:

- Receber o ID do cubo.
- Receber sua posição espacial.
- Receber o destino, conforme a interface definida.
- Solicitar o planejamento dos movimentos.
- Coordenar a aproximação, coleta e transporte.
- Acompanhar a execução.
- Tratar falhas.
- Informar o resultado ao computador central.

Exemplo conceitual de operação:

```text
Solicitação do computador central
              |
              v
       Identificar cubo
              |
              v
      Validar coordenadas
              |
              v
      Aproximar a garra
              |
              v
       Fechar J4
              |
              v
      Confirmar a coleta
              |
              v
       Receber destino
              |
              v
      Planejar transporte
              |
              v
       Levar até destino
              |
              v
        Abrir J4
              |
              v
       Retornar resultado
```

A operação deverá possuir estados bem definidos, como:

- `IDLE`: aguardando solicitação.
- `RECEIVING_TARGET`: recebendo dados do cubo.
- `PLANNING_APPROACH`: planejando aproximação.
- `APPROACHING`: aproximando a garra.
- `GRASPING`: realizando a coleta.
- `VERIFYING_GRASP`: verificando a coleta.
- `PLANNING_TRANSPORT`: planejando transporte.
- `TRANSPORTING`: transportando o objeto.
- `RELEASING`: liberando o cubo.
- `SUCCEEDED`: operação concluída.
- `FAILED`: operação interrompida por falha.

Esses estados são uma proposta inicial e poderão ser ajustados conforme a implementação.

---

# 6. Segurança e confiabilidade

A segurança deverá ser considerada em todas as fases.

- [ ] Definir limites mecânicos e operacionais para cada junta.
- [ ] Evitar comandos que possam provocar colisões.
- [ ] Configurar limites de velocidade e aceleração.
- [ ] Avaliar limites de corrente e alimentação dos servos.
- [ ] Implementar tratamento de perda de comunicação.
- [ ] Evitar movimentos quando a posição do cubo for inválida ou desatualizada.
- [ ] Verificar colisões com o ambiente e com o próprio robô.
- [ ] Implementar tratamento de falhas de planejamento.
- [ ] Avaliar como interromper uma operação em andamento.
- [ ] Definir condições seguras para recuperação de falhas.

A confirmação de que um cubo foi efetivamente agarrado deverá ser tratada separadamente da confirmação de que o comando de fechamento foi enviado. Sem sensores apropriados, não será possível garantir que o objeto está preso apenas observando o comando do servo.

---

# 7. Organização sugerida do software

A organização definitiva dependerá da implementação atual do repositório, mas a separação lógica sugerida é:

```text
garra/
|
+-- firmware/
|   +-- controle dos servomotores
|
+-- garra_final_description/
|   +-- URDF e Xacro
|   +-- meshes
|   +-- descrição mecânica
|
+-- garra_moveit_config/
|   +-- planejamento
|   +-- controladores
|   +-- configurações de hardware
|
+-- garra_hardware/
|   +-- interface com Arduino
|
+-- garra_bringup/
|   +-- inicialização dos componentes
|
+-- garra_interfaces/
|   +-- mensagens
|   +-- serviços
|   +-- ações
|
+-- garra_manipulation/
|   +-- coordenador de manipulação
|   +-- coleta
|   +-- transporte
|
+-- scripts/
    +-- testes e ferramentas auxiliares
```

Essa estrutura é uma proposta de organização futura, não uma descrição obrigatória do estado atual do repositório.

---

# 8. Critérios de conclusão do projeto

O projeto será considerado concluído quando os seguintes objetivos forem atingidos:

### Garra independente
- Os quatro servos funcionam corretamente.
- Os movimentos possuem limites calibrados.
- A mecânica está finalizada.
- O URDF representa corretamente a montagem física.
- O MoveIt 2 planeja e executa movimentos.
- A garra pode ser controlada sem depender do computador central.

### Integração ROS 2
- A garra inicia como um subsistema ROS 2.
- O hardware é controlado por meio da arquitetura ROS 2.
- O MoveIt 2 executa os movimentos solicitados.
- O computador central consegue solicitar operações.
- O estado e os resultados são retornados corretamente.

### Manipulação autônoma
- O robô identifica cubos por QR Code.
- A posição espacial recebida pela câmera é convertida corretamente.
- A garra consegue alcançar e coletar cubos.
- O computador central consegue definir um destino.
- A garra consegue transportar e liberar os objetos.
- O sistema informa sucesso ou falha de cada operação.

---

# 9. Pendências técnicas para as próximas etapas

| Pendência | Responsável lógico | Prioridade |
|---|---|---|
| Finalizar a mecânica atual | Desenvolvimento mecânico | Alta |
| Testar J1, J2 e J3 pelo ROS 2 (`garra_real.launch.py`) | Hardware e controle | Alta |
| Medir suportes do 1º andar e chassi sob a placa da base | Descrição mecânica | Média |
| Identificar o servo J4 | Hardware | Alta |
| Integrar J4 ao firmware | Hardware e firmware | Média |
| Atualizar CAD e URDF | Descrição mecânica | Após finalização mecânica |
| Validar MoveIt 2 com quatro juntas | Planejamento e controle | Após atualização do URDF |
| Investigar tópicos da câmera | Percepção | Alta na fase de integração |
| Identificar referencial das coordenadas XYZ | Percepção e integração | Alta na fase de integração |
| Verificar transformações TF2 | Integração ROS 2 | Alta na fase de integração |
| Definir representação do destino | Arquitetura do robô | Pendente de decisão |
| Desenvolver coordenador de manipulação | Software ROS 2 | Após interfaces definidas |
| Validar coleta e transporte | Integração geral | Etapa final |

---

# 10. Estratégia de desenvolvimento

O desenvolvimento deverá seguir uma sequência que reduza retrabalho e permita validar cada componente isoladamente.

**Ordem recomendada:**

1. Validar o hardware e os servos atuais (J1–J3).
2. Validar a garra no RViz e MoveIt 2.
3. Validar o acionamento real de J1–J3 por ROS 2.
4. Integrar e testar o J4.
5. Finalizar a montagem mecânica.
6. Atualizar o CAD e o URDF.
7. Investigar a interface do sistema de visão.
8. Validar a transformação das coordenadas da câmera.
9. Desenvolver o coordenador de manipulação.
10. Definir a interface de destino com o computador central.
11. Testar coleta e transporte isoladamente.
12. Integrar a operação ao fluxo de tarefas do robô completo.

**Princípio do projeto:** cada camada deverá funcionar e ser validada antes de depender da camada seguinte. A comunicação entre os componentes deverá utilizar interfaces bem definidas, permitindo que a garra seja desenvolvida e testada independentemente do restante do robô.

---

**Documento de planejamento — versão 1.1 (10/11/2026)**

*As decisões ainda não confirmadas deverão permanecer identificadas como pendências, evitando que hipóteses sejam tratadas como requisitos definitivos.*
