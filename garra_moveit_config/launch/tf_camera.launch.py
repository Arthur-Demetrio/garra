"""TF estatica robo_base -> camera_optical_frame (para o cube_vision e o alcance_cubo.py --vision).

A pose vem do camera_cubo.py (calibrar_camera.py > robot_config_webcam.yaml do -WORK > trena),
entao acompanha a calibracao sozinha.

    ros2 launch garra_moveit_config tf_camera.launch.py
"""

import os
import sys

from launch import LaunchDescription

sys.path.insert(0, os.path.dirname(__file__))
from coleta_comum import tf_camera  # noqa: E402


def generate_launch_description():
    return LaunchDescription([tf_camera()])
