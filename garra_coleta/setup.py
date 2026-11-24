from glob import glob

from setuptools import setup

PACOTE = 'garra_coleta'

setup(
    name=PACOTE,
    version='0.1.0',
    packages=[PACOTE],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACOTE]),
        ('share/' + PACOTE, ['package.xml']),
        ('share/' + PACOTE + '/launch', glob('launch/*.py')),
        ('share/' + PACOTE + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arthur Demetrio',
    maintainer_email='arthurbdemetrio@gmail.com',
    description='Coleta de cubos da garra.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'pegar_cubo = garra_coleta.pegar_cubo:main',
            'alcance_cubo = garra_coleta.alcance_cubo:main',
            'ajuste_base = garra_coleta.ajuste_base:main',
            'camera_cubo = garra_coleta.camera_cubo:main',
            'plataforma_y = garra_coleta.plataforma_y:main',
            'gerenciador_slots = garra_coleta.gerenciador_slots:main',
        ],
    },
)
