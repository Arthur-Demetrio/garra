# Calibração da garra e da câmera

Como calibrar os servos da garra e a câmera, e como conferir se a calibração ficou certa.

| Calibração | Script | Grava em | Quando refazer |
|---|---|---|---|
| Servos J1–J3 (zero, limites, sentido) | `scripts/calibrar_servos.py` | `garra_moveit_config/config/calibracao_servos.yaml` | depois de tirar/girar um braço no eixo do servo, trocar um servo ou remontar |
| Escala dos servos (µs por rad) | `scripts/medir_escala.py` | anotar e pôr no mesmo YAML | quando o braço real anda mais ou menos que o RViz |
| Câmera (inclinação, altura, x/y) | `scripts/calibrar_camera.py` | `~/.config/garra/camera_montagem.json` | depois de mexer na câmera ou no suporte |

Em todo terminal que usa ROS:

```bash
source /opt/ros/jazzy/setup.bash && source ~/work/garra/install/setup.bash
cd ~/work/garra
```

> **A porta serial só tem um dono.** `calibrar_servos.py`, `medir_escala.py`, `ver_fins_de_curso.py` e o
> Monitor Serial falam direto com o Arduino: feche antes o `garra_real.launch.py`.

---

## 1. Servos da garra

### 1.1 O que é cada valor

```yaml
Revolucionar_46: {zero_us: 1678, us_por_rad: -580.0, min_us: 504, max_us: 2091}
```

| Campo | Significado |
|---|---|
| `zero_us` | pulso em que a junta está em **q = 0**: a **pose do modelo (RViz) em q = 0**. Atenção: nela os elos **não** ficam todos verticais (1º ~5° para trás, 2º ~2° para a frente, **garra ~23,5° para a frente**) |
| `us_por_rad` | quanto o pulso muda por radiano. O sinal é o sentido (negativo = pulso maior gira para o lado negativo do modelo) |
| `min_us`, `max_us` | pulsos limite: até onde o braço vai sem bater |

Juntas: `Revolucionar_46` = J1 (base do braço, pino 9), `Revolucionar_49` = J2 (meio, pino 10),
`Revolucionar_72` = J3 (ponta, pino 11).

### 1.2 Calibrar zero e limites

Antes: braço livre, nada por perto, **mão na fonte dos servos**. Ao abrir a porta o Arduino reinicia
e os três servos vão para 1472 µs (90°) na hora.

```bash
python3 scripts/calibrar_servos.py --versao 180
```

Comandos:

| Comando | O que faz |
|---|---|
| `j 1` / `j 2` / `j 3` | seleciona a junta (ela vai ao zero já medido, se houver) |
| `+` / `-` | ajuste fino do pulso (10 µs ≈ 1°). `+ 30` anda 30 µs (máx. 50 por comando) |
| `1,90` | leva o motor 1 direto a 90° (salta: confira o caminho antes) |
| `zero` | o pulso atual vira q = 0 |
| `min` / `max` | o pulso atual vira o limite inferior / superior |
| `sentido +` / `sentido -` | `+` se o ângulo no modelo cresce quando o pulso cresce |
| `estado` | mostra o que já foi medido |
| `salvar` | grava o YAML (exige as três juntas completas) |
| `sair` | sai |

Para cada junta:

1. `j <n>`.
2. **Zero:** com `+`/`-`, deixe o elo **no mesmo ângulo que ele tem no RViz em q = 0** (não na
   vertical!): 1º elo ~5° para trás, 2º elo ~2° para a frente, garra ~23,5° para a frente. Confira
   com o celular (app de nível). Digite `zero`. Em 2026-11-20 os zeros foram feitos com os elos
   verticais e a ponta real ia ~8 cm mais longe que o modelo.
3. **Limites:** `min` e `max` são **pulsos**, não lados do braço: `min` = o **menor** pulso,
   `max` = o **maior**, e o `zero` tem que ficar entre os dois (senão o `salvar` recusa).
   - com `-` (pulso diminuindo) devagar, vá até perto de bater (estrutura, cabos, chão) e **pare
     antes**: `min`;
   - com `+` (pulso aumentando), até o outro lado: `max`.

   Meça onde o braço **bate de verdade**. Não use o fim do servo (544 / 2400 µs) se o braço bater
   antes: o software confia nesses limites. Confira em `estado` antes de salvar: `min` < `zero` < `max`.
4. `sentido` igual ao de antes (`-` para J1–J3), a não ser que o teste do item 1.4 mostre o contrário.

Depois das três: `salvar`.

### 1.3 Atualizar os limites em radianos

O calibrador mede em µs; o MoveIt usa rad. Converta:

```
q = (pulso − zero_us) / us_por_rad
```

Com `us_por_rad` negativo, o `min_us` dá o q **maior** e o `max_us` o q **menor**. Ponha os valores
(arredondados um pouco para dentro) nos **dois** lugares, que precisam bater:

- `scripts/ajustar_urdf.py` → `LIMITES` (depois rode `python3 scripts/ajustar_urdf.py`);
- `garra_coleta/garra_coleta/cliente_garra.py` → `LIMITES`.

E recompile: `colcon build --symlink-install`.

### 1.4 Conferir

```bash
ros2 launch garra_moveit_config garra_real.launch.py        # terminal 1
python3 scripts/mover_junta.py home                          # terminal 2: igual ao RViz em q=0 (garra ~23,5° para a frente)
python3 scripts/mover_junta.py J1 0.5                        # uma junta de cada vez
python3 scripts/mover_junta.py J2 -0.5
python3 scripts/mover_junta.py J3 0.5
```

Em cada pose, compare o ângulo do elo no celular com o do RViz.

> **Sentido: compare com o RViz, não com a intuição.** Os ângulos do modelo não se somam como "cada
> junta desfaz a anterior": com `J1 0.5` e `J2 -0.5` o 2º elo fica a ~59° para a frente no modelo (não na
> vertical). Em 2026-11-20 o J2 foi invertido por engano por causa disso e a garra passou a ir para cima
> em vez de para baixo. Para conferir o sentido, olhe o RViz e o braço lado a lado na mesma pose.

> **Onde encostar o celular no 1º elo (J1):** o primeiro elo é **curvo**. Meça na parte reta **perto
> do J2**, não perto do J1. Perto do J1 a curva soma ~11° (em 2026-11-20: J1 = 28,6° deu 40° perto do
> J1 e 29° perto do J2; o certo é o 29°).

Resultado:

- **bate (±3°)** → pronto;
- **errado sempre pela mesma diferença** → zero errado: refaça o 1.2;
- **errado cada vez mais quanto maior o ângulo** → escala errada: item 1.5;
- **andou para o lado contrário** → sentido errado.

### 1.5 Escala (só se o 1.4 apontar)

```bash
python3 scripts/medir_escala.py
```

Para cada junta, ele leva a junta ao zero, você lê o ângulo do elo no celular, ele gira um ângulo
conhecido, você lê de novo e digita quantos graus o elo girou. Ponha o `us_por_rad` resultante no
YAML e confira de novo com o 1.4.

---

## 2. Câmera

### 2.1 O que é calibrado

A posição da câmera no robô: inclinação (graus para baixo), altura da lente e x/y. Com isso o
`camera_cubo.py` converte o pixel de cada tag em posição no chão. A calibração interna da lente
(`garra_coleta/config/camera_calib_webcam.yaml`) já existe e não precisa ser refeita.

**Medidas com a trena: sempre a partir da borda dianteira da base.** O programa soma 27,5 cm
(centro do robô até a borda). Y: positivo para a esquerda da linha central.

### 2.2 Calibrar

Feche antes o `camera_cubo.py`, o Cheese e o navegador (a webcam só abre em um programa).

```bash
python3 scripts/calibrar_camera.py                 # ignora o tag 11 por padrão
python3 scripts/calibrar_camera.py --ignorar 256   # tags que NÃO estão em cubos no chão (ex.: plataforma)
```

1. Espalhe alguns cubos no chão, tag para cima, sem nada na frente. Espere a **inclinação** e a
   **altura** estabilizarem no topo da janela (lidas pelos próprios tags).
2. Deixe **um** cubo visível num ponto medido — bom ponto: **19 cm à frente da borda, y = −3 cm**.
3. Aperte **K** e digite no terminal `19 -3` (x da borda e y, em cm). Ele corrige o x/y da câmera.
4. Aperte **S** para salvar. O `camera_cubo.py` passa a usar na hora, sem recompilar.

Teclas: **S** salva, **K** corrige x/y com cubo medido, **R** zera as medidas, **Q** sai.

### 2.3 Conferir

```bash
ros2 run garra_coleta camera_cubo --sem-ros
```

Ponha o cubo em 2 ou 3 pontos medidos (ex.: 13, 17 e 21 cm da borda). O texto
`da borda: x=… y=…` ao lado do tag deve bater com a trena em ~1 cm. Se só o ponto usado no K bater,
refaça com o cubo no meio da área de coleta.

### 2.4 Tag da plataforma Y

Opcional, para a plataforma lateral. A câmera lê o tag da plataforma como **ID 256**. Meça com a trena
a altura do centro do tag até o chão e passe na câmera:

```bash
ros2 run garra_coleta camera_cubo --tag-plataforma 256 --altura-plataforma <altura em m>
```

---

## 3. Fins de curso da plataforma Y

Para testar as chaves de fim de curso (pinos 17 (esquerda) e 14 (direita) do Mega). Grave antes o firmware
`firmware/teste_fins_de_curso` e feche o driver ROS.

```bash
python3 scripts/ver_fins_de_curso.py /dev/ttyUSB0
```

Aperte uma chave de cada vez e veja o estado mudar na tela. Depois, **regrave o firmware
`firmware/garra_servos`**, senão o ROS não conecta.

---

## 4. Onde fica cada arquivo

| Arquivo | Conteúdo | No git? |
|---|---|---|
| `garra_moveit_config/config/calibracao_servos.yaml` | zero, escala e limites dos servos | sim |
| `scripts/ajustar_urdf.py` e `garra_coleta/garra_coleta/cliente_garra.py` | limites em rad | sim |
| `~/.config/garra/camera_montagem.json` | pose calibrada da câmera | não (fica no computador) |
| `garra_coleta/config/camera_calib_webcam.yaml` | lente da webcam | sim |
| `~/.config/garra/pose_inicial.json` | pose inicial do braço (`pose_inicial_web.py`) | não |
