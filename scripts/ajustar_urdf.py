"""Ajusta o URDF exportado do Fusion (fusion2URDF) para o MoveIt. Rode depois de cada exportacao.

    python3 scripts/ajustar_urdf.py [garra_final_description/urdf/garra_final.urdf]

O exportador gera todas as juntas como continuous, sem limites, e com um bloco <ros2_control>
proprio (hardware simulado, 11 juntas). Este script, sem alterar malhas nem origens:
  - torna J1, J2 e J3 revolute com os limites de LIMITES e corrige os eixos de EIXOS;
  - prende as juntas dos dedos (fixed) na pose do CAD, ate o J4 ser integrado;
  - remove o <ros2_control> exportado: o sistema real fica em garra_moveit_config.
E idempotente: rodar duas vezes nao muda nada. O TCP (tool0) fica em garra_moveit_config,
para sobreviver a uma nova exportacao.
"""

import os
import sys
import xml.etree.ElementTree as ET

PADRAO = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                      'garra_final_description', 'urdf', 'garra_final.urdf')

# J1, J2, J3. PROVISORIO: +-90 graus em torno da pose do CAD, ate a calibracao.
# effort: torque do DS3218MG (~1.86 N.m). A velocidade real fica em joint_limits.yaml.
LIMITES = {
    'Revolucionar_46': (-1.5708, 1.5708),
    'Revolucionar_49': (-1.5708, 1.5708),
    'Revolucionar_72': (-1.5708, 1.5708),
}
# Eixo de cada junta no frame dela, quando a exportacao erra o sentido. Fixo (nao inverte), para o
# script continuar idempotente. J3: positivo dobra a garra para a frente (-X do base_link), como J1.
# Confira depois de cada exportacao: o validar_garra.py testa esse sentido.
EIXOS = {
    'Revolucionar_72': '0 0 -1',
}
EFFORT = '1.86'
VELOCIDADE = '1.00'


def main():
    caminho = sys.argv[1] if len(sys.argv) > 1 else PADRAO
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    arvore = ET.parse(caminho, parser)
    robo = arvore.getroot()

    faltam = set(LIMITES)
    presos = []
    for junta in robo.findall('joint'):
        nome = junta.get('name')
        if nome in LIMITES:
            faltam.discard(nome)
            junta.set('type', 'revolute')
            limite = junta.find('limit')
            if limite is None:
                limite = ET.SubElement(junta, 'limit')
            limite.attrib.clear()
            baixo, alto = LIMITES[nome]
            limite.set('lower', str(baixo))
            limite.set('upper', str(alto))
            limite.set('effort', EFFORT)
            limite.set('velocity', VELOCIDADE)
            if nome in EIXOS:
                eixo = junta.find('axis')
                if eixo is None:
                    eixo = ET.SubElement(junta, 'axis')
                eixo.set('xyz', EIXOS[nome])
        elif junta.get('type') in ('continuous', 'revolute', 'prismatic'):
            junta.set('type', 'fixed')
            for tag in ('axis', 'limit', 'dynamics', 'mimic'):
                for filho in junta.findall(tag):
                    junta.remove(filho)
            presos.append(nome)

    if faltam:
        print(f'ERRO: juntas do braco ausentes no URDF: {sorted(faltam)}. '
              'A exportacao mudou os nomes? Atualize LIMITES e as configs do MoveIt.')
        return 1

    removidos = robo.findall('ros2_control')
    for bloco in removidos:
        robo.remove(bloco)

    ET.indent(arvore, space='  ')
    arvore.write(caminho, encoding='unicode', xml_declaration=True)
    print(f'{caminho}: braco revolute {sorted(LIMITES)}; dedos fixos {presos or "(ja estavam)"}; '
          f'{len(removidos)} bloco(s) ros2_control removido(s).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
