# Validation Report: garra_final

## Status: PASS (with warnings)

## Summary

| Metric | Value |
|--------|-------|
| Links | 21 |
| Joints | 20 |
| Assemblies | 2 |
| Root | `base_link` |
| Errors | 0 |
| Warnings | 2 |

## Warnings

- Link 'base_link': visual mesh export failed — exception: 2 : InternalValidationError : Xl::Utils::findObjectPath(this, objPath)
- Link 'Caixa_motor_pequeno': visual mesh export failed — exception: 2 : InternalValidationError : Xl::Utils::findObjectPath(this, objPath)

## Kinematic Tree

```
base_link [primitive]
  └─ R_gido [fixed]
    shoulder_motor [primitive]
      └─ R_gido_46 [fixed]
        garra_final_showder_servo_horn [primitive]
          └─ R_gido_47 [fixed]
            M_dulo_1 [primitive]
              └─ R_gido_48 [fixed]
                MOTOR_1 [primitive]
                  └─ R_gido_49 [fixed]
                    garra_final_showder_servo_horn_2 [primitive]
                      └─ R_gido_50 [fixed]
                        M_dulo_2 [primitive]
                          └─ R_gido_51 [fixed]
                            MOTOR_1_1 [primitive]
                              └─ R_gido_52 [fixed]
                                Caixa_motor_pequeno [primitive]
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
| `Caixa_motor_pequeno` | primitive STL | box |
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
