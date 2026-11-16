# Integração com o robô (Robo-WORK)

`executor_prova_garra.patch` muda o `executor_prova.py` do repositório
[OttavioSilva/Robo-WORK](https://github.com/OttavioSilva/Robo-WORK)
(`workspaces/robo_jazzy_ws/src/robo_navigation/scripts/executor_prova.py`, gerado sobre o commit 3e329b7).

Por padrão o passo PEGAR deixa de ser simulado (`--sem-garra` volta ao simulado):

1. o robô chega na mesa (alinhador) e fica parado;
2. o executor publica `std_msgs/String` em `/coleta/pedido` (JSON com `area` e `objetos`);
3. o `garra_coleta/garra_coleta/alcance_cubo.py` deste repositório olha a câmera:
   - cubo fora do alcance → `/robo/mover` → `garra_coleta/garra_coleta/ajuste_base.py` move a base até 15 cm pelo canal
     `especial` do árbitro e responde em `/robo/mover/resultado`;
   - cubo ao alcance → `/pegar_cubo` → `garra_coleta/garra_coleta/pegar_cubo.py` pega e responde em `/pegar_cubo/resultado`;
4. o `alcance_cubo.py` responde em `/coleta/resultado`: `ok` ou `falhou: motivo`;
5. o executor segue a prova mesmo se a coleta falhar (o SOLTAR continua simulado).

Fora de uma coleta, o `alcance_cubo.py` ignora a câmera: a base não se mexe sozinha durante a navegação.

Aplicar na raiz do Robo-WORK:

```bash
git apply /caminho/para/garra/integracao_robo/executor_prova_garra.patch
```

Processos do lado da garra (além do launch da garra e da câmera):

```bash
ros2 run garra_coleta pegar_cubo
ros2 run garra_coleta ajuste_base
ros2 run garra_coleta alcance_cubo          # ou --vision com o cube_vision
python3 executor_prova.py tarefa.yaml
```

Ainda não testado no robô: o fluxo foi conferido só com mensagens publicadas à mão.
Os nomes dos objetos da tarefa ainda não são ligados ao ID do AprilTag: a garra pega o cubo mais
perto da base.
