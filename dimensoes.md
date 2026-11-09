# Dimensões do Robô — Referência para o URDF

Este documento reúne as dimensões físicas do robô móvel e as posições dos componentes estruturais. Seu objetivo é servir como referência para a criação do URDF integrado, permitindo representar o robô e sua garra no ROS 2 e realizar análises de colisão utilizando o MoveIt 2.

As dimensões estão em metros, salvo quando indicado o contrário.

## 1. Sistema de referência

Será utilizado o centro da base do robô como referência horizontal.

- **X positivo:** frente do robô.
- **X negativo:** traseira do robô.
- **Y positivo:** lado esquerdo.
- **Y negativo:** lado direito.
- **Z positivo:** para cima.
- **Origem horizontal:** centro da base do robô.
- **Referência vertical:** chão, com Z = 0.

## 2. Estrutura principal

### 2.1 Base do robô

| Propriedade | Dimensão |
|---|---:|
| Comprimento (X) | 0,55 m |
| Largura (Y) | 0,50 m |
| Altura do topo em relação ao chão | 0,10 m |
| Espessura | Pendente de confirmação |

A base está centralizada na origem horizontal.

Seus limites são:

- X: −0,275 m a +0,275 m.
- Y: −0,250 m a +0,250 m.
- Topo: Z = 0,100 m.

A espessura da base ainda precisa ser medida para determinar a posição vertical de seu centro geométrico.

### 2.2 Primeiro andar

| Propriedade | Dimensão |
|---|---:|
| Comprimento (X) | 0,39 m |
| Largura (Y) | 0,50 m |
| Espessura | 0,01 m |
| Altura da superfície superior | 0,27 m |

O primeiro andar está alinhado com a borda traseira da base.

Posicionamento:

- X: −0,275 m a +0,115 m.
- Y: −0,250 m a +0,250 m.
- Centro X: −0,080 m.
- Centro Y: 0,000 m.
- Superfície inferior: Z = 0,260 m.
- Superfície superior: Z = 0,270 m.
- Centro Z: 0,265 m.

### 2.3 Segundo andar

| Propriedade | Dimensão |
|---|---:|
| Comprimento (X) | 0,21 m |
| Largura (Y) | 0,50 m |
| Espessura | 0,01 m |
| Altura da superfície superior | 0,37 m |

O segundo andar também está alinhado com a borda traseira da base.

Posicionamento:

- X: −0,275 m a −0,065 m.
- Y: −0,250 m a +0,250 m.
- Centro X: −0,170 m.
- Centro Y: 0,000 m.
- Superfície inferior: Z = 0,360 m.
- Superfície superior: Z = 0,370 m.
- Centro Z: 0,365 m.

## 3. Suportes estruturais

Os suportes serão representados no URDF como geometrias de colisão simples, preferencialmente caixas, respeitando suas dimensões e posições físicas.

### 3.1 Suportes do primeiro andar

O primeiro andar possui quatro suportes, distribuídos em dois pares: dois dianteiros e dois traseiros.

Dimensões de cada suporte:

| Propriedade | Dimensão |
|---|---:|
| Comprimento (X) | 0,02 m |
| Largura (Y) | 0,05 m |

Posicionamento longitudinal calculado:

- Par dianteiro: centros em X = +0,050 m.
- Par traseiro: centros em X = −0,205 m.

Os valores foram calculados considerando as distâncias informadas em relação às bordas do primeiro andar e o comprimento de 2 cm dos suportes na direção X.

Posicionamento lateral:

- Os dois suportes dianteiros ficam em lados opostos do robô.
- Os dois suportes traseiros ficam em lados opostos do robô.
- Coordenadas Y dos centros: pendentes de confirmação.

### 3.2 Suportes do segundo andar

O segundo andar possui quatro suportes, posicionados nos quatro cantos da estrutura.

Dimensões de cada suporte:

| Propriedade | Dimensão |
|---|---:|
| Comprimento (X) | 0,05 m |
| Largura (Y) | 0,02 m |

Posicionamento dos centros:

| Posição | X | Y |
|---|---:|---:|
| Dianteiro esquerdo | −0,040 m | +0,240 m |
| Dianteiro direito | −0,040 m | −0,240 m |
| Traseiro esquerdo | −0,250 m | +0,240 m |
| Traseiro direito | −0,250 m | −0,240 m |

Essas posições consideram os suportes alinhados às bordas dianteira, traseira e laterais do segundo andar, com suas dimensões orientadas conforme indicado na tabela.

### 3.3 Altura dos suportes

As alturas ainda precisam ser confirmadas fisicamente.

Como referência geométrica, os espaços entre as superfícies são:

- Entre o topo da base e a parte inferior do primeiro andar: 0,160 m.
- Entre o topo do primeiro andar e a parte inferior do segundo andar: 0,090 m.

Esses valores representam os vãos disponíveis, não necessariamente as alturas definitivas dos suportes.

## 4. Instalação da garra

A garra será instalada diretamente sobre a superfície superior da base do robô.

Posicionamento informado:

- Distância da borda dianteira da base até a referência da garra: 0,05 m.
- Alinhamento lateral: centro do robô (Y = 0).
- Altura de instalação: superfície superior da base, Z = 0,100 m.

Coordenada horizontal calculada:

\[
X = 0,275 - 0,050 = 0,225\text{ m}
\]

Transformação inicialmente prevista para a montagem:

```xml
<origin xyz="0.225 0.0 0.100" rpy="0 0 0"/>
```

**Atenção:** essa transformação considera que a origem do `base_link` da garra coincide com o centro de sua base física e que sua orientação está alinhada aos eixos do robô. Isso precisa ser verificado no modelo e na montagem real antes de definir o URDF integrado.

## 5. Considerações para o URDF integrado

A estrutura integrada deverá representar:

1. Base do robô.
2. Primeiro andar.
3. Segundo andar.
4. Suportes estruturais.
5. Garra robótica.
6. Juntas fixas entre as estruturas.
7. Juntas articuladas da garra.

As estruturas fixas deverão ser conectadas por juntas `fixed`. As juntas articuladas da garra deverão preservar seus nomes, eixos e limites definidos no modelo original.

Para a análise de colisões no MoveIt 2:

- Utilizar geometrias simplificadas para colisão, como caixas e cilindros.
- Incluir os andares e suportes na geometria de colisão.
- Verificar colisões entre os elos móveis da garra e a estrutura do robô.
- Evitar desabilitar indiscriminadamente pares de colisão na matriz SRDF.
- Validar as configurações de movimento com o robô físico, inicialmente em baixa velocidade e sob supervisão.

## 6. Informações pendentes de confirmação

| Item | Informação pendente |
|---|---|
| Base | Espessura física |
| Suportes do primeiro andar | Posições laterais (Y) |
| Suportes do primeiro andar | Altura física |
| Suportes do segundo andar | Altura física |
| Garra | Confirmação da origem e orientação do `base_link` em relação à base física |

As posições calculadas dos suportes são referências geométricas para o modelo e devem ser conferidas com a estrutura física antes da validação final.

## 7. Objetivo deste documento

Este arquivo será utilizado como referência para a integração entre o modelo do robô móvel e o modelo da garra no ROS 2 Jazzy.

A partir dessas dimensões, será possível construir o URDF integrado, visualizar a montagem no RViz 2 e configurar o MoveIt 2 para planejar movimentos considerando também as possíveis colisões entre a garra e o próprio robô.