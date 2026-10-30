# Validation Report: garra_final

## Status: PASS

## Summary

| Metric | Value |
|--------|-------|
| Links | 21 |
| Joints | 20 |
| Assemblies | 2 |
| Root | `base_link` |
| Errors | 0 |
| Warnings | 0 |

## Kinematic Tree

```
base_link [primitive]
  └─ R_gido [fixed]
    shoulder_motor [primitive]
      └─ Revolucionar_46 [continuous]
        garra_final_showder_servo_horn [BAKE] [primitive]
          └─ R_gido_47 [fixed]
            M_dulo_1 [primitive]
              └─ R_gido_48 [fixed]
                MOTOR_1 [primitive]
                  └─ Revolucionar_49 [continuous]
                    garra_final_showder_servo_horn_2 [BAKE] [primitive]
                      └─ R_gido_50 [fixed]
                        M_dulo_2 [primitive]
                          └─ R_gido_51 [fixed]
                            MOTOR_1_1 [primitive]
                              └─ Revolucionar_52 [continuous]
                                Caixa_motor_pequeno [BAKE] [primitive]
                                  └─ R_gido_53 [fixed]
                                    motor_pequeno [primitive]
                                      └─ R_gido_54 [fixed]
                                        Componente4_Espelhar [primitive]
                                          └─ R_gido_55 [fixed]
                                            Healical_Gear_20R_0_00_m_1_0 [primitive]
                                              └─ R_gido_59 [fixed]
                                                Componente5_Espelhar_1 [primitive]
                                              └─ R_gido_60 [fixed]
                                                Componente5_Espelhar_Espelhar [primitive]
                                                  └─ R_gido_61 [fixed]
                                                    Componente6_Espelhar [primitive]
                                          └─ R_gido_56 [fixed]
                                            Healical_Gear_20R_0_00_m_1_0_1 [primitive]
                                              └─ R_gido_57 [fixed]
                                                Componente5 [primitive]
                                                  └─ R_gido_62 [fixed]
                                                    Componente6 [primitive]
                                              └─ R_gido_58 [fixed]
                                                Componente5_Espelhar [primitive]
                                              └─ R_gido_64 [fixed]
                                                Componente4 [primitive]
                                          └─ R_gido_63 [fixed]
                                            Healical_Gear_8R_0_00_m_1_0 [primitive]
```

## Collision Geometry

| Link | Source | Shape/File |
|------|--------|------------|
| `Caixa_motor_pequeno` | primitive STL | cylinder |
| `Componente4` | primitive STL | box |
| `Componente4_Espelhar` | primitive STL | box |
| `Componente5` | primitive STL | box |
| `Componente5_Espelhar` | primitive STL | box |
| `Componente5_Espelhar_1` | primitive STL | box |
| `Componente5_Espelhar_Espelhar` | primitive STL | box |
| `Componente6` | primitive STL | box |
| `Componente6_Espelhar` | primitive STL | box |
| `Healical_Gear_20R_0_00_m_1_0` | primitive STL | box |
| `Healical_Gear_20R_0_00_m_1_0_1` | primitive STL | box |
| `Healical_Gear_8R_0_00_m_1_0` | primitive STL | cylinder |
| `MOTOR_1` | primitive STL | cylinder |
| `MOTOR_1_1` | primitive STL | cylinder |
| `M_dulo_1` | primitive STL | box |
| `M_dulo_2` | primitive STL | box |
| `base_link` | primitive STL | cylinder |
| `garra_final_showder_servo_horn` | primitive STL | box |
| `garra_final_showder_servo_horn_2` | primitive STL | box |
| `motor_pequeno` | primitive STL | box |
| `shoulder_motor` | primitive STL | cylinder |

## Mesh Bake Offsets

Links where joint frame ≠ component origin. Visual/inertial/collision origins shifted.

| Link | Offset (mm) |
|------|-------------|
| `garra_final_showder_servo_horn` | (15.8, -85.4, -3.0) |
| `Caixa_motor_pequeno` | (-131.1, 48.6, -157.0) |
| `garra_final_showder_servo_horn_2` | (15.8, -85.4, 0.0) |
