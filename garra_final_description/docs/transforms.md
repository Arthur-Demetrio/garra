# Transformation Matrices - garra_final

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | shoulder_motor |
| $L_{2}$ | garra_final_showder_servo_horn |
| $L_{3}$ | M_dulo_1 |
| $L_{4}$ | MOTOR_1 |
| $L_{5}$ | garra_final_showder_servo_horn_2 |
| $L_{6}$ | M_dulo_2 |
| $L_{7}$ | MOTOR_1_1 |
| $L_{8}$ | Caixa_motor_pequeno |
| $L_{9}$ | motor_pequeno |
| $L_{10}$ | Componente4_Espelhar |
| $L_{11}$ | Healical_Gear_20R_0_00_m_1_0 |
| $L_{12}$ | Healical_Gear_20R_0_00_m_1_0_1 |
| $L_{13}$ | Healical_Gear_8R_0_00_m_1_0 |
| $L_{14}$ | Componente5_Espelhar_1 |
| $L_{15}$ | Componente5_Espelhar_Espelhar |
| $L_{16}$ | Componente5 |
| $L_{17}$ | Componente5_Espelhar |
| $L_{18}$ | Componente4 |
| $L_{19}$ | Componente6_Espelhar |
| $L_{20}$ | Componente6 |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | Revolucionar_46 | continuous (rad) | $L_{1}$ | $L_{2}$ |
| $q_{2}$ | Revolucionar_49 | continuous (rad) | $L_{4}$ | $L_{5}$ |
| $q_{3}$ | Revolucionar_52 | continuous (rad) | $L_{7}$ | $L_{8}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  +-- [fixed] R_gido
      L1: shoulder_motor
        +-- [continuous] Revolucionar_46 (q1)
            L2: garra_final_showder_servo_horn
              +-- [fixed] R_gido_47
                  L3: M_dulo_1
                    +-- [fixed] R_gido_48
                        L4: MOTOR_1
                          +-- [continuous] Revolucionar_49 (q2)
                              L5: garra_final_showder_servo_horn_2
                                +-- [fixed] R_gido_50
                                    L6: M_dulo_2
                                      +-- [fixed] R_gido_51
                                          L7: MOTOR_1_1
                                            +-- [continuous] Revolucionar_52 (q3)
                                                L8: Caixa_motor_pequeno
                                                  +-- [fixed] R_gido_53
                                                      L9: motor_pequeno
                                                        +-- [fixed] R_gido_54
                                                            L10: Componente4_Espelhar
                                                              |-- [fixed] R_gido_55
                                                              |   L11: Healical_Gear_20R_0_00_m_1_0
                                                              |     |-- [fixed] R_gido_59
                                                              |     |   L14: Componente5_Espelhar_1
                                                              |     +-- [fixed] R_gido_60
                                                              |         L15: Componente5_Espelhar_Espelhar
                                                              |           +-- [fixed] R_gido_61
                                                              |               L19: Componente6_Espelhar
                                                              |-- [fixed] R_gido_56
                                                              |   L12: Healical_Gear_20R_0_00_m_1_0_1
                                                              |     |-- [fixed] R_gido_57
                                                              |     |   L16: Componente5
                                                              |     |     +-- [fixed] R_gido_62
                                                              |     |         L20: Componente6
                                                              |     |-- [fixed] R_gido_58
                                                              |     |   L17: Componente5_Espelhar
                                                              |     +-- [fixed] R_gido_64
                                                              |         L18: Componente4
                                                              +-- [fixed] R_gido_63
                                                                  L13: Healical_Gear_8R_0_00_m_1_0
```

## Transforms

## R_gido

$L_{0}$ **base_link** -> $L_{1}$ **shoulder_motor** (fixed)

- **origin xyz**: (-0.002399, 0.000644, 0.000258) m
- **origin rpy**: (0, 0.011578, 0) rad

### Local Transform

$$
T^{0}_{1} = \begin{bmatrix}
0.999933 & 0 & 0.011578 & -0.002399 \\
0 & 1 & 0 & 0.000644 \\
-0.011578 & 0 & 0.999933 & 0.000258 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolucionar_46

$L_{1}$ **shoulder_motor** -> $L_{2}$ **garra_final_showder_servo_horn** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (-0.023838, -0.025144, 0.086064) m
- **origin rpy**: (-1.570796, 1.550014, 3.141593) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{1}_{2}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
-0.020781 & 0.999784 & 0 & -0.023838 \\
0 & 0 & -1 & -0.025144 \\
-0.999784 & -0.020781 & 0 & 0.086064 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{1}) = \begin{bmatrix}
c_{1} & -s_{1} & 0 & 0 \\
s_{1} & c_{1} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_47

$L_{2}$ **garra_final_showder_servo_horn** -> $L_{3}$ **M_dulo_1** (fixed)

- **origin xyz**: (0.074914, 0.050317, -0.03) m
- **origin rpy**: (-1.570796, 0, 1.832596) rad

### Local Transform

$$
T^{2}_{3} = \begin{bmatrix}
-0.258819 & 0 & -0.965926 & 0.074914 \\
0.965926 & 0 & -0.258819 & 0.050317 \\
0 & -1 & 0 & -0.03 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_48

$L_{3}$ **M_dulo_1** -> $L_{4}$ **MOTOR_1** (fixed)

- **origin xyz**: (0.002592, -0.000644, 0.004103) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{3}_{4} = \begin{bmatrix}
1 & 0 & 0 & 0.002592 \\
0 & 1 & 0 & -0.000644 \\
0 & 0 & 1 & 0.004103 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolucionar_49

$L_{4}$ **MOTOR_1** -> $L_{5}$ **garra_final_showder_servo_horn_2** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (0.059838, -0.008856, 0.173222) m
- **origin rpy**: (-1.570796, 0, 3.141593) rad
- **axis**: (0, 0, -1)

### Local Transform

$T^{4}_{5}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
-1 & 0 & 0 & 0.059838 \\
0 & 0 & -1 & -0.008856 \\
0 & -1 & 0 & 0.173222 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{2}) = \begin{bmatrix}
c_{2} & s_{2} & 0 & 0 \\
-s_{2} & c_{2} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_50

$L_{5}$ **garra_final_showder_servo_horn_2** -> $L_{6}$ **M_dulo_2** (fixed)

- **origin xyz**: (0.163486, 0.09503, -0.005666) m
- **origin rpy**: (-1.570796, 0, 2.443461) rad

### Local Transform

$$
T^{5}_{6} = \begin{bmatrix}
-0.766044 & 0 & -0.642788 & 0.163486 \\
0.642788 & 0 & -0.766044 & 0.09503 \\
0 & -1 & 0 & -0.005666 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_51

$L_{6}$ **M_dulo_2** -> $L_{7}$ **MOTOR_1_1** (fixed)

- **origin xyz**: (0.001615, 0.000015, 0.004041) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{6}_{7} = \begin{bmatrix}
1 & 0 & 0 & 0.001615 \\
0 & 1 & 0 & 0.000015 \\
0 & 0 & 1 & 0.004041 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolucionar_52

$L_{7}$ **MOTOR_1_1** -> $L_{8}$ **Caixa_motor_pequeno** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (0.125796, 0.000362, 0.149459) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, -1)

### Local Transform

$$
T^{7}_{8}(q_{3}) = \begin{bmatrix}
c_{3} & s_{3} & 0 & 0.125796 \\
-s_{3} & c_{3} & 0 & 0.000362 \\
0 & 0 & 1 & 0.149459 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_53

$L_{8}$ **Caixa_motor_pequeno** -> $L_{9}$ **motor_pequeno** (fixed)

- **origin xyz**: (0.170546, 0.0575, -0.099228) m
- **origin rpy**: (-1.623428, 1.570796, -0.174046) rad

### Local Transform

$$
T^{8}_{9} = \begin{bmatrix}
0 & -0.992638 & 0.121117 & 0.170546 \\
0 & 0.121117 & 0.992638 & 0.0575 \\
-1 & 0 & 0 & -0.099228 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_54

$L_{9}$ **motor_pequeno** -> $L_{10}$ **Componente4_Espelhar** (fixed)

- **origin xyz**: (-0.085228, 0.152296, -0.0575) m
- **origin rpy**: (-1.570796, 1.570796, 0) rad

### Local Transform

$$
T^{9}_{10} = \begin{bmatrix}
0 & -1 & 0 & -0.085228 \\
0 & 0 & 1 & 0.152296 \\
-1 & 0 & 0 & -0.0575 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_55

$L_{10}$ **Componente4_Espelhar** -> $L_{11}$ **Healical_Gear_20R_0_00_m_1_0** (fixed)

- **origin xyz**: (-0.014, 0, -0.005) m
- **origin rpy**: (-3.141593, 0, 0) rad

### Local Transform

$$
T^{10}_{11} = \begin{bmatrix}
1 & 0 & 0 & -0.014 \\
0 & -1 & 0 & 0 \\
0 & 0 & -1 & -0.005 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_56

$L_{10}$ **Componente4_Espelhar** -> $L_{12}$ **Healical_Gear_20R_0_00_m_1_0_1** (fixed)

- **origin xyz**: (0.014, 0, -0.005) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{10}_{12} = \begin{bmatrix}
1 & 0 & 0 & 0.014 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & -0.005 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_63

$L_{10}$ **Componente4_Espelhar** -> $L_{13}$ **Healical_Gear_8R_0_00_m_1_0** (fixed)

- **origin xyz**: (0, 0, -0.005) m
- **origin rpy**: (3.141593, 0, 0.401426) rad

### Local Transform

$$
T^{10}_{13} = \begin{bmatrix}
0.920505 & 0.390731 & 0 & 0 \\
0.390731 & -0.920505 & 0 & 0 \\
0 & 0 & -1 & -0.005 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_59

$L_{11}$ **Healical_Gear_20R_0_00_m_1_0** -> $L_{14}$ **Componente5_Espelhar_1** (fixed)

- **origin xyz**: (0.016666, 0.00388, 0.005) m
- **origin rpy**: (0, 0, -3.141593) rad

### Local Transform

$$
T^{11}_{14} = \begin{bmatrix}
-1 & 0 & 0 & 0.016666 \\
0 & -1 & 0 & 0.00388 \\
0 & 0 & 1 & 0.005 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_60

$L_{11}$ **Healical_Gear_20R_0_00_m_1_0** -> $L_{15}$ **Componente5_Espelhar_Espelhar** (fixed)

- **origin xyz**: (0.016666, 0.00388, -0.005) m
- **origin rpy**: (0, 0, -3.141593) rad

### Local Transform

$$
T^{11}_{15} = \begin{bmatrix}
-1 & 0 & 0 & 0.016666 \\
0 & -1 & 0 & 0.00388 \\
0 & 0 & 1 & -0.005 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_57

$L_{12}$ **Healical_Gear_20R_0_00_m_1_0_1** -> $L_{16}$ **Componente5** (fixed)

- **origin xyz**: (-0.016666, -0.00388, -0.005) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{12}_{16} = \begin{bmatrix}
1 & 0 & 0 & -0.016666 \\
0 & 1 & 0 & -0.00388 \\
0 & 0 & 1 & -0.005 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_58

$L_{12}$ **Healical_Gear_20R_0_00_m_1_0_1** -> $L_{17}$ **Componente5_Espelhar** (fixed)

- **origin xyz**: (-0.016666, -0.00388, 0.005) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{12}_{17} = \begin{bmatrix}
1 & 0 & 0 & -0.016666 \\
0 & 1 & 0 & -0.00388 \\
0 & 0 & 1 & 0.005 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_64

$L_{12}$ **Healical_Gear_20R_0_00_m_1_0_1** -> $L_{18}$ **Componente4** (fixed)

- **origin xyz**: (-0.014, 0, -0.005) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{12}_{18} = \begin{bmatrix}
1 & 0 & 0 & -0.014 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & -0.005 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_61

$L_{15}$ **Componente5_Espelhar_Espelhar** -> $L_{19}$ **Componente6_Espelhar** (fixed)

- **origin xyz**: (0, 0, 0.01) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{15}_{19} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0.01 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_62

$L_{16}$ **Componente5** -> $L_{20}$ **Componente6** (fixed)

- **origin xyz**: (0, 0, 0) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{16}_{20} = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{2} = T^{0}_{1} \cdot T^{1}_{2}(q_{1})\quad (L_0 \to L_{2}: \text{garra_final_showder_servo_horn})$$

$$T^{0}_{3} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3}\quad (L_0 \to L_{3}: \text{M_dulo_1})$$

$$T^{0}_{4} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4}\quad (L_0 \to L_{4}: \text{MOTOR_1})$$

$$T^{0}_{5} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2})\quad (L_0 \to L_{5}: \text{garra_final_showder_servo_horn_2})$$

$$T^{0}_{6} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6}\quad (L_0 \to L_{6}: \text{M_dulo_2})$$

$$T^{0}_{7} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7}\quad (L_0 \to L_{7}: \text{MOTOR_1_1})$$

$$T^{0}_{8} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3})\quad (L_0 \to L_{8}: \text{Caixa_motor_pequeno})$$

$$T^{0}_{9} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9}\quad (L_0 \to L_{9}: \text{motor_pequeno})$$

$$T^{0}_{10} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{10}\quad (L_0 \to L_{10}: \text{Componente4_Espelhar})$$

$$T^{0}_{11} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{10} \cdot T^{10}_{11}\quad (L_0 \to L_{11}: \text{Healical_Gear_20R_0_00_m_1_0})$$

$$T^{0}_{12} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{10} \cdot T^{10}_{12}\quad (L_0 \to L_{12}: \text{Healical_Gear_20R_0_00_m_1_0_1})$$

$$T^{0}_{13} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{10} \cdot T^{10}_{13}\quad (L_0 \to L_{13}: \text{Healical_Gear_8R_0_00_m_1_0})$$

$$T^{0}_{14} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{10} \cdot T^{10}_{11} \cdot T^{11}_{14}\quad (L_0 \to L_{14}: \text{Componente5_Espelhar_1})$$

$$T^{0}_{15} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{10} \cdot T^{10}_{11} \cdot T^{11}_{15}\quad (L_0 \to L_{15}: \text{Componente5_Espelhar_Espelhar})$$

$$T^{0}_{16} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{10} \cdot T^{10}_{12} \cdot T^{12}_{16}\quad (L_0 \to L_{16}: \text{Componente5})$$

$$T^{0}_{17} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{10} \cdot T^{10}_{12} \cdot T^{12}_{17}\quad (L_0 \to L_{17}: \text{Componente5_Espelhar})$$

$$T^{0}_{18} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{10} \cdot T^{10}_{12} \cdot T^{12}_{18}\quad (L_0 \to L_{18}: \text{Componente4})$$

$$T^{0}_{19} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{10} \cdot T^{10}_{11} \cdot T^{11}_{15} \cdot T^{15}_{19}\quad (L_0 \to L_{19}: \text{Componente6_Espelhar})$$

$$T^{0}_{20} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{10} \cdot T^{10}_{12} \cdot T^{12}_{16} \cdot T^{16}_{20}\quad (L_0 \to L_{20}: \text{Componente6})$$

