# garra_final — Robot Description

![garra_final](images/robot.png)

## Overview

| Property | Value |
|----------|-------|
| Total mass | 1.026 kg |
| Links | 19 |
| Joints | 18 (11 movable) |
| Assemblies | 2 |
| Root link | `base_link` |

## Table of Contents

- [Kinematic Tree](#kinematic-tree)
- [Link Properties](#link-properties)
- [Joint Properties](#joint-properties)
- [Assembly Breakdown](#assembly-breakdown)
- [Quick Start (ROS 2)](#quick-start-ros-2)
- [Files](#files)

## Kinematic Tree

```
base_link
  └─ R_gido [fixed]
    shoulder_motor
      └─ Revolucionar_46 [continuous]
        garra_final_showder_servo_horn [BAKE]
          └─ R_gido_47 [fixed]
            M_dulo_1
              └─ R_gido_48 [fixed]
                MOTOR_1
                  └─ Revolucionar_49 [continuous]
                    garra_final_showder_servo_horn_2 [BAKE]
                      └─ R_gido_50 [fixed]
                        M_dulo_2
                          └─ R_gido_65 [fixed]
                            MOTOR_1_1
                              └─ Revolucionar_72 [continuous]
                                garra_final_showder_servo_horn_3 [BAKE]
                                  └─ R_gido_73 [fixed]
                                    Base
                                      └─ R_gido_77 [fixed]
                                        Motor
                                      └─ Revolucionar_85 [continuous]
                                        Engr_2 [BAKE]
                                          └─ Revolucionar_86 [continuous]
                                            P_1 [BAKE]
                                      └─ Revolucionar_87 [continuous]
                                        Junta_1 [BAKE]
                                      └─ Revolucionar_89 [continuous]
                                        Junta_1_1 [BAKE]
                                      └─ Revolucionar_91 [continuous]
                                        Engr_1 [BAKE]
                                          └─ Revolucionar_92 [continuous]
                                            P_1_1 [BAKE]
                                      └─ Revolucionar_93 [continuous]
                                        Junta_2 [BAKE]
                                      └─ Revolucionar_95 [continuous]
                                        Junta [BAKE]
```

## Link Properties

| Link | Mass (kg) | Material | Collision | Bodies |
|------|-----------|----------|-----------|--------|
| `Base` | 0.1915 | A_o | visual_reuse | 1 |
| `Engr_1` | 0.0231 | A_o | visual_reuse | 1 |
| `Engr_2` | 0.0202 | A_o | visual_reuse | 1 |
| `Junta` | 0.0068 | A_o | visual_reuse | 1 |
| `Junta_1` | 0.0068 | A_o | visual_reuse | 1 |
| `Junta_1_1` | 0.0068 | A_o | visual_reuse | 1 |
| `Junta_2` | 0.0068 | A_o | visual_reuse | 1 |
| `MOTOR_1` | 0.0892 | Alum_nio_prata_anodizada | visual_reuse | 1 |
| `MOTOR_1_1` | 0.0892 | Alum_nio_prata_anodizada | visual_reuse | 1 |
| `M_dulo_1` | 0.0961 | Laminado_vermelho_fosco | visual_reuse | 1 |
| `M_dulo_2` | 0.0707 | Laminado_vermelho_fosco | visual_reuse | 1 |
| `Motor` | 0.0509 | A_o | visual_reuse | 1 |
| `P_1` | 0.0674 | A_o | visual_reuse | 1 |
| `P_1_1` | 0.0674 | A_o | visual_reuse | 1 |
| `base_link` | 0.1225 | Laminado_vermelho_fosco | visual_reuse | 1 |
| `garra_final_showder_servo_horn` | 0.0072 | A_o | visual_reuse | 1 |
| `garra_final_showder_servo_horn_2` | 0.0072 | A_o | visual_reuse | 1 |
| `garra_final_showder_servo_horn_3` | 0.0072 | A_o | visual_reuse | 1 |
| `shoulder_motor` | 0.0892 | Alum_nio_prata_anodizada | visual_reuse | 1 |

## Joint Properties

| Joint | Type | Parent → Child | Axis | Limits |
|-------|------|---------------|------|--------|
| `R_gido` | fixed | `base_link` → `shoulder_motor` | (0,0,1) | — |
| `R_gido_47` | fixed | `garra_final_showder_servo_horn` → `M_dulo_1` | (0,0,1) | — |
| `R_gido_48` | fixed | `M_dulo_1` → `MOTOR_1` | (0,0,1) | — |
| `R_gido_50` | fixed | `garra_final_showder_servo_horn_2` → `M_dulo_2` | (0,0,1) | — |
| `R_gido_65` | fixed | `M_dulo_2` → `MOTOR_1_1` | (0,0,1) | — |
| `R_gido_73` | fixed | `garra_final_showder_servo_horn_3` → `Base` | (0,0,1) | — |
| `R_gido_77` | fixed | `Base` → `Motor` | (0,0,1) | — |
| `Revolucionar_46` | continuous | `shoulder_motor` → `garra_final_showder_servo_horn` | (0,0,1) | — |
| `Revolucionar_49` | continuous | `MOTOR_1` → `garra_final_showder_servo_horn_2` | (0,0,-1) | — |
| `Revolucionar_72` | continuous | `MOTOR_1_1` → `garra_final_showder_servo_horn_3` | (0,0,1) | — |
| `Revolucionar_85` | continuous | `Base` → `Engr_2` | (0,0,1) | — |
| `Revolucionar_86` | continuous | `Engr_2` → `P_1` | (0,0,1) | — |
| `Revolucionar_87` | continuous | `Base` → `Junta_1` | (0,0,1) | — |
| `Revolucionar_89` | continuous | `Base` → `Junta_1_1` | (0,0,1) | — |
| `Revolucionar_91` | continuous | `Base` → `Engr_1` | (0,0,1) | — |
| `Revolucionar_92` | continuous | `Engr_1` → `P_1_1` | (0,0,1) | — |
| `Revolucionar_93` | continuous | `Base` → `Junta_2` | (0,0,1) | — |
| `Revolucionar_95` | continuous | `Base` → `Junta` | (0,0,1) | — |

## Assembly Breakdown

### Configura_o_1

- **Links**: Engr_1, Motor, Engr_2, Base, P_1, Junta, P_1_1, Junta_1, Junta_1_1, Junta_2
- **Total mass**: 0.448 kg

### garra_final

- **Links**: base_link, shoulder_motor, M_dulo_1, MOTOR_1, M_dulo_2, MOTOR_1_1, garra_final_showder_servo_horn, garra_final_showder_servo_horn_2, garra_final_showder_servo_horn_3
- **Total mass**: 0.579 kg

## Quick Start (ROS 2)

```bash
# 1. Copy package to your ROS 2 workspace
cp -r garra_final_description ~/ros2_ws/src/

# 2. Build
cd ~/ros2_ws
colcon build --packages-select garra_final_description
source install/setup.bash

# 3. Visualize in RViz2
ros2 launch garra_final_description display.launch.py

# 4. Validate URDF structure
check_urdf install/garra_final_description/share/garra_final_description/urdf/garra_final.urdf

# 5. Print kinematic tree
urdf_to_graphviz install/garra_final_description/share/garra_final_description/urdf/garra_final.urdf
```

**Joint control**: The launch file includes `joint_state_publisher_gui` —
use the sliders to move revolute/prismatic joints in RViz2.

**Topic inspection**:
```bash
# See published joint states
ros2 topic echo /joint_states

# See robot description parameter
ros2 param get /robot_state_publisher robot_description
```

## Files

| Path | Description |
|------|-------------|
| `urdf/garra_final.urdf.xacro` | Top-level xacro (entry point) |
| `urdf/garra_final.urdf` | Flat URDF (for validation) |
| `urdf/assemblies/` | Per-assembly xacro macros |
| `meshes/` | Visual (OBJ) and collision (STL) meshes |
| `launch/display.launch.py` | Launch robot_state_publisher, RViz, and generated controllers |
| `config/joint_state.yaml` | Joint state publisher config |
| `config/ros2_controllers.yaml` | Generated ros2_control controller manager config |
| `robot_data.yaml` | Supplementary data (beyond URDF) |
| `docs/transforms.md` | Transformation matrices (KaTeX) |

## Customizing

Assemblies tagged `!dummy_` are designed to be swapped out. To replace one:

1. Create your replacement as a xacro macro with the same interface
2. Place it in `urdf/assemblies/`
3. Update the `<xacro:include>` in `urdf/garra_final.urdf.xacro`
4. Update meshes in `meshes/<your_assembly>/`

The xacro prefix system (`${prefix}`) ensures link names stay unique
when multiple instances of the same assembly are used.

---
*Generated by Fusion URDF/XACRO Exporter v3.1.1*