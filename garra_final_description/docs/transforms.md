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
| $L_{8}$ | garra_final_showder_servo_horn_3 |
| $L_{9}$ | Base |
| $L_{10}$ | Motor |
| $L_{11}$ | Engr_2 |
| $L_{12}$ | Junta_1 |
| $L_{13}$ | Junta_1_1 |
| $L_{14}$ | Engr_1 |
| $L_{15}$ | Junta_2 |
| $L_{16}$ | Junta |
| $L_{17}$ | P_1 |
| $L_{18}$ | P_1_1 |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | Revolucionar_46 | continuous (rad) | $L_{1}$ | $L_{2}$ |
| $q_{2}$ | Revolucionar_49 | continuous (rad) | $L_{4}$ | $L_{5}$ |
| $q_{3}$ | Revolucionar_72 | continuous (rad) | $L_{7}$ | $L_{8}$ |
| $q_{4}$ | Revolucionar_85 | continuous (rad) | $L_{9}$ | $L_{11}$ |
| $q_{5}$ | Revolucionar_87 | continuous (rad) | $L_{9}$ | $L_{12}$ |
| $q_{6}$ | Revolucionar_89 | continuous (rad) | $L_{9}$ | $L_{13}$ |
| $q_{7}$ | Revolucionar_91 | continuous (rad) | $L_{9}$ | $L_{14}$ |
| $q_{8}$ | Revolucionar_93 | continuous (rad) | $L_{9}$ | $L_{15}$ |
| $q_{9}$ | Revolucionar_95 | continuous (rad) | $L_{9}$ | $L_{16}$ |
| $q_{10}$ | Revolucionar_86 | continuous (rad) | $L_{11}$ | $L_{17}$ |
| $q_{11}$ | Revolucionar_92 | continuous (rad) | $L_{14}$ | $L_{18}$ |

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
                                      +-- [fixed] R_gido_65
                                          L7: MOTOR_1_1
                                            +-- [continuous] Revolucionar_72 (q3)
                                                L8: garra_final_showder_servo_horn_3
                                                  +-- [fixed] R_gido_73
                                                      L9: Base
                                                        |-- [fixed] R_gido_77
                                                        |   L10: Motor
                                                        |-- [continuous] Revolucionar_85 (q4)
                                                        |   L11: Engr_2
                                                        |     +-- [continuous] Revolucionar_86 (q10)
                                                        |         L17: P_1
                                                        |-- [continuous] Revolucionar_87 (q5)
                                                        |   L12: Junta_1
                                                        |-- [continuous] Revolucionar_89 (q6)
                                                        |   L13: Junta_1_1
                                                        |-- [continuous] Revolucionar_91 (q7)
                                                        |   L14: Engr_1
                                                        |     +-- [continuous] Revolucionar_92 (q11)
                                                        |         L18: P_1_1
                                                        |-- [continuous] Revolucionar_93 (q8)
                                                        |   L15: Junta_2
                                                        +-- [continuous] Revolucionar_95 (q9)
                                                            L16: Junta
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
- **origin rpy**: (1.570796, 1.197477, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{1}_{2}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.364708 & 0.931122 & 0 & -0.023838 \\
0 & 0 & -1 & -0.025144 \\
-0.931122 & 0.364708 & 0 & 0.086064 \\
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
- **origin rpy**: (-1.570796, 1.475992, 3.141593) rad
- **axis**: (0, 0, -1)

### Local Transform

$T^{4}_{5}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
-0.094662 & 0.995509 & 0 & 0.059838 \\
0 & 0 & -1 & -0.008856 \\
-0.995509 & -0.094662 & 0 & 0.173222 \\
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

- **origin xyz**: (0.160854, 0.091479, -0.006666) m
- **origin rpy**: (-1.570796, 0, 2.443461) rad

### Local Transform

$$
T^{5}_{6} = \begin{bmatrix}
-0.766044 & 0 & -0.642788 & 0.160854 \\
0.642788 & 0 & -0.766044 & 0.091479 \\
0 & -1 & 0 & -0.006666 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_65

$L_{6}$ **M_dulo_2** -> $L_{7}$ **MOTOR_1_1** (fixed)

- **origin xyz**: (-0.0003, -0.000428, 0.000662) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{6}_{7} = \begin{bmatrix}
1 & 0 & 0 & -0.0003 \\
0 & 1 & 0 & -0.000428 \\
0 & 0 & 1 & 0.000662 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolucionar_72

$L_{7}$ **MOTOR_1_1** -> $L_{8}$ **garra_final_showder_servo_horn_3** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (0.146796, 0.022261, 0.173559) m
- **origin rpy**: (-1.570796, 1.4014, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{7}_{8}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.168587 & -0.985687 & 0 & 0.146796 \\
0 & 0 & 1 & 0.022261 \\
-0.985687 & -0.168587 & 0 & 0.173559 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{3}) = \begin{bmatrix}
c_{3} & -s_{3} & 0 & 0 \\
s_{3} & c_{3} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_73

$L_{8}$ **garra_final_showder_servo_horn_3** -> $L_{9}$ **Base** (fixed)

- **origin xyz**: (0.004924, 0.001353, 0.0625) m
- **origin rpy**: (1.55077, -1.570796, 0) rad

### Local Transform

$$
T^{8}_{9} = \begin{bmatrix}
0 & -0.999799 & -0.020025 & 0.004924 \\
0 & 0.020025 & -0.999799 & 0.001353 \\
1 & 0 & 0 & 0.0625 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## R_gido_77

$L_{9}$ **Base** -> $L_{10}$ **Motor** (fixed)

- **origin xyz**: (-0.113471, -0.081721, -0.0235) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{9}_{10} = \begin{bmatrix}
1 & 0 & 0 & -0.113471 \\
0 & 1 & 0 & -0.081721 \\
0 & 0 & 1 & -0.0235 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolucionar_85

$L_{9}$ **Base** -> $L_{11}$ **Engr_2** (continuous)
  Variable: $q_{4}$

- **origin xyz**: (-0.06925, -0.055, 0.004) m
- **origin rpy**: (3.141593, 0, -1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{9}_{11}(q_{4}) = T_{fixed} \cdot R_{axis}(q_{4})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -1 & 0 & -0.06925 \\
-1 & 0 & 0 & -0.055 \\
0 & 0 & -1 & 0.004 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{4}) = \begin{bmatrix}
c_{4} & -s_{4} & 0 & 0 \\
s_{4} & c_{4} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolucionar_87

$L_{9}$ **Base** -> $L_{12}$ **Junta_1** (continuous)
  Variable: $q_{5}$

- **origin xyz**: (-0.06714, -0.079, 0.004) m
- **origin rpy**: (3.141593, 0, -1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{9}_{12}(q_{5}) = T_{fixed} \cdot R_{axis}(q_{5})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -1 & 0 & -0.06714 \\
-1 & 0 & 0 & -0.079 \\
0 & 0 & -1 & 0.004 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{5}) = \begin{bmatrix}
c_{5} & -s_{5} & 0 & 0 \\
s_{5} & c_{5} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolucionar_89

$L_{9}$ **Base** -> $L_{13}$ **Junta_1_1** (continuous)
  Variable: $q_{6}$

- **origin xyz**: (-0.06714, -0.079, 0) m
- **origin rpy**: (3.141593, 0, -1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{9}_{13}(q_{6}) = T_{fixed} \cdot R_{axis}(q_{6})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -1 & 0 & -0.06714 \\
-1 & 0 & 0 & -0.079 \\
0 & 0 & -1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{6}) = \begin{bmatrix}
c_{6} & -s_{6} & 0 & 0 \\
s_{6} & c_{6} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolucionar_91

$L_{9}$ **Base** -> $L_{14}$ **Engr_1** (continuous)
  Variable: $q_{7}$

- **origin xyz**: (-0.04675, -0.055, 0.004) m
- **origin rpy**: (3.141593, 0, -1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{9}_{14}(q_{7}) = T_{fixed} \cdot R_{axis}(q_{7})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -1 & 0 & -0.04675 \\
-1 & 0 & 0 & -0.055 \\
0 & 0 & -1 & 0.004 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{7}) = \begin{bmatrix}
c_{7} & -s_{7} & 0 & 0 \\
s_{7} & c_{7} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolucionar_93

$L_{9}$ **Base** -> $L_{15}$ **Junta_2** (continuous)
  Variable: $q_{8}$

- **origin xyz**: (-0.04886, -0.079, 0.004) m
- **origin rpy**: (3.141593, 0, -1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{9}_{15}(q_{8}) = T_{fixed} \cdot R_{axis}(q_{8})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -1 & 0 & -0.04886 \\
-1 & 0 & 0 & -0.079 \\
0 & 0 & -1 & 0.004 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{8}) = \begin{bmatrix}
c_{8} & -s_{8} & 0 & 0 \\
s_{8} & c_{8} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolucionar_95

$L_{9}$ **Base** -> $L_{16}$ **Junta** (continuous)
  Variable: $q_{9}$

- **origin xyz**: (-0.04886, -0.079, 0) m
- **origin rpy**: (3.141593, 0, -1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{9}_{16}(q_{9}) = T_{fixed} \cdot R_{axis}(q_{9})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -1 & 0 & -0.04886 \\
-1 & 0 & 0 & -0.079 \\
0 & 0 & -1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{9}) = \begin{bmatrix}
c_{9} & -s_{9} & 0 & 0 \\
s_{9} & c_{9} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolucionar_86

$L_{11}$ **Engr_2** -> $L_{17}$ **P_1** (continuous)
  Variable: $q_{10}$

- **origin xyz**: (0.034279, 0.029154, 0) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$$
T^{11}_{17}(q_{10}) = \begin{bmatrix}
c_{10} & -s_{10} & 0 & 0.034279 \\
s_{10} & c_{10} & 0 & 0.029154 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolucionar_92

$L_{14}$ **Engr_1** -> $L_{18}$ **P_1_1** (continuous)
  Variable: $q_{11}$

- **origin xyz**: (0.000359, -0.044999, 0) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$$
T^{14}_{18}(q_{11}) = \begin{bmatrix}
c_{11} & -s_{11} & 0 & 0.000359 \\
s_{11} & c_{11} & 0 & -0.044999 \\
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

$$T^{0}_{8} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3})\quad (L_0 \to L_{8}: \text{garra_final_showder_servo_horn_3})$$

$$T^{0}_{9} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9}\quad (L_0 \to L_{9}: \text{Base})$$

$$T^{0}_{10} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{10}\quad (L_0 \to L_{10}: \text{Motor})$$

$$T^{0}_{11} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{11}(q_{4})\quad (L_0 \to L_{11}: \text{Engr_2})$$

$$T^{0}_{12} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{12}(q_{5})\quad (L_0 \to L_{12}: \text{Junta_1})$$

$$T^{0}_{13} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{13}(q_{6})\quad (L_0 \to L_{13}: \text{Junta_1_1})$$

$$T^{0}_{14} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{14}(q_{7})\quad (L_0 \to L_{14}: \text{Engr_1})$$

$$T^{0}_{15} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{15}(q_{8})\quad (L_0 \to L_{15}: \text{Junta_2})$$

$$T^{0}_{16} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{16}(q_{9})\quad (L_0 \to L_{16}: \text{Junta})$$

$$T^{0}_{17} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{11}(q_{4}) \cdot T^{11}_{17}(q_{10})\quad (L_0 \to L_{17}: \text{P_1})$$

$$T^{0}_{18} = T^{0}_{1} \cdot T^{1}_{2}(q_{1}) \cdot T^{2}_{3} \cdot T^{3}_{4} \cdot T^{4}_{5}(q_{2}) \cdot T^{5}_{6} \cdot T^{6}_{7} \cdot T^{7}_{8}(q_{3}) \cdot T^{8}_{9} \cdot T^{9}_{14}(q_{7}) \cdot T^{14}_{18}(q_{11})\quad (L_0 \to L_{18}: \text{P_1_1})$$

