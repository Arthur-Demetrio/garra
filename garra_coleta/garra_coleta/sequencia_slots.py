"""Sequencias de guardar e retirar cubos dos slots (ainda NAO integrado ao ROS).

Os slots ficam na borda do primeiro andar, fora da vista da camera. Por isso o ID de cada cubo
e lido com o cubo preso nos dedos, na pose 'mostrar_apriltag', e o estoque (slots.py) e a
unica memoria de onde cada cubo esta.

Este modulo so descreve a ordem dos passos e o que fazer em cada falha. O hardware entra por
duas interfaces, que serao implementadas na integracao (MoveIt/ROS) e aqui nos testes por fakes:

  Braco.ir_para(nome, q)             juntas, planejado pelo MoveIt (planejar_juntas + executar)
  Braco.entrar_slot(q_slot, subida)  juntas ate 'subida' acima de q_slot e desce reto ate ele
  Braco.sair_slot(subida)            sobe reto 'subida' a partir da pose atual
  Braco.abrir_dedos() / fechar_dedos()
  Braco.ir_y(y)                      plataforma Y (/garra/plataforma_y/alvo); so se configurado
  LeitorTag.ler(timeout_s) -> list[int]   IDs que a camera ve agora

Guardar (o cubo ja esta nos dedos, depois do pegar_cubo):
  mostrar_apriltag (+ busca em J3) -> le ID -> reserva o menor slot -> acima_slots -> entra no slot -> abre
  -> sai do slot -> acima_slots -> pronta
Retirar (de um slot ou de um cubo pelo ID):
  abre -> acima_slots -> entra no slot -> fecha -> sai do slot -> acima_slots
  -> mostrar_apriltag -> le ID (confirma qual cubo saiu) -> devolve o ID para a missao decidir
  Sem leitura: volta ao slot, abre e sai; o registro nao muda (ErroSlots, cubo_na_garra=None).

A pose mostrar_apriltag so serve para CONFIRMAR o cubo nos dedos e saber o ID. J1 e J2 ficam fixos;
se o tag nao for lido, a junta 3 tenta os desvios de 'busca_j3_graus' (o cubo nunca fica preso
igual, e J3 corrige a inclinacao do tag sem tirar o cubo da imagem).

A chave dos dedos (/garra/cubo_preso) NAO e usada: esta presa na posicao fechada e sempre diria
"preso". A unica confirmacao de que o cubo esta nos dedos e a camera ler o tag.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional, Protocol

import math

import yaml

from garra_coleta.slots import CAPACIDADE, GerenciadorSlots

PONTOS = ("pronta", "mostrar_apriltag", "acima_slots")
BUSCA_J3_PADRAO_GRAUS = (8.0, -8.0, 15.0, -15.0)
BUSCA_J3_MAX_GRAUS = 30.0


class Braco(Protocol):
    def ir_para(self, nome: str, q: List[float]) -> None: ...
    def entrar_slot(self, q_slot: List[float], subida: float) -> None: ...
    def sair_slot(self, subida: float) -> None: ...
    def abrir_dedos(self) -> None: ...
    def fechar_dedos(self) -> None: ...
    def ir_y(self, y: float) -> None: ...


class LeitorTag(Protocol):
    def ler(self, timeout_s: float) -> List[int]: ...


class ErroSlots(RuntimeError):
    """Falha numa sequencia. 'cubo_na_garra' diz o que ficou nos dedos (ID, -1 = desconhecido,
    None = nada), para quem chamou decidir o proximo passo."""

    def __init__(self, mensagem, cubo_na_garra=None):
        super().__init__(mensagem)
        self.cubo_na_garra = cubo_na_garra


@dataclass(frozen=True)
class PoseSlot:
    juntas: List[float]
    y_plataforma: Optional[float] = None   # m, robo_base (no yaml: y_cm, em cm)


@dataclass(frozen=True)
class ConfigSlots:
    pontos: Dict[str, List[float]]
    slots: Dict[int, PoseSlot]
    subida_m: float
    busca_j3: tuple = ()      # desvios (rad) somados a J3 de mostrar_apriltag, em ordem


@dataclass(frozen=True)
class ResultadoRetirada:
    cubo_id: Optional[int]   # ID lido pela camera
    slot: int
    confirmado: bool          # a camera leu o tag
    esperado: Optional[int]   # o que o estoque dizia estar no slot


def _juntas(nome, valor):
    if valor is None:
        return None
    if not isinstance(valor, (list, tuple)) or len(valor) != 3:
        raise ValueError(f"{nome}: precisa ser uma lista com 3 juntas, veio {valor!r}")
    return [float(v) for v in valor]


def carregar_config(caminho, capacidade=CAPACIDADE, exigir_completa=True) -> ConfigSlots:
    """Le config/slots.yaml. Com exigir_completa, falha listando todas as poses ainda null."""
    dados = yaml.safe_load(Path(caminho).read_text(encoding="utf-8")) or {}
    faltando = []
    pontos = {}
    for nome in PONTOS:
        q = _juntas(nome, (dados.get("pontos") or {}).get(nome))
        if q is None:
            faltando.append(nome)
        pontos[nome] = q
    slots = {}
    brutos = {int(k): v for k, v in (dados.get("slots") or {}).items()}
    for s in range(capacidade):
        item = brutos.get(s) or {}
        q = _juntas(f"slot {s}", item.get("juntas"))
        if q is None:
            faltando.append(f"slot {s}")
        y_cm = item.get("y_cm")
        slots[s] = PoseSlot(q, None if y_cm is None else float(y_cm) / 100.0)
    subida = float(dados.get("subida_m", 0.04))
    if not 0.0 < subida <= 0.15:
        raise ValueError(f"subida_m fora de (0, 0.15]: {subida}")
    graus = dados.get("busca_j3_graus", BUSCA_J3_PADRAO_GRAUS) or ()
    if any(abs(float(g)) > BUSCA_J3_MAX_GRAUS for g in graus):
        raise ValueError(f"busca_j3_graus passa de +-{BUSCA_J3_MAX_GRAUS}: {graus}")
    busca = tuple(math.radians(float(g)) for g in graus)
    if exigir_completa and faltando:
        raise ValueError("poses nao calibradas em slots.yaml: " + ", ".join(faltando))
    return ConfigSlots(pontos, slots, subida, busca)


class SequenciaSlots:

    def __init__(self, braco: Braco, leitor: LeitorTag, config: ConfigSlots,
                 estoque: Optional[GerenciadorSlots] = None,
                 salvar: Optional[Callable[[GerenciadorSlots], None]] = None,
                 timeout_tag_s: float = 2.0, tentativas_tag: int = 2,
                 log: Callable[[str], None] = print):
        self.b, self.leitor, self.cfg = braco, leitor, config
        self.estoque = estoque or GerenciadorSlots(len(config.slots))
        self._salvar = salvar or (lambda _e: None)
        self.timeout_tag_s = timeout_tag_s
        self.tentativas_tag = tentativas_tag
        self.log = log

    # ---- passos ----------------------------------------------------------------------------
    def _ler_uma(self, rotulo) -> Optional[int]:
        ids = sorted(set(self.leitor.ler(self.timeout_tag_s)))
        if len(ids) == 1:
            return ids[0]
        self.log(f"leitura do tag ({rotulo}): viu {ids or 'nada'}")
        return None

    def ler_tag(self) -> Optional[int]:
        """Vai para mostrar_apriltag e devolve o unico ID visto; None se nenhum ou mais de um.

        Primeiro 'tentativas_tag' leituras na pose; depois uma leitura em cada desvio de J3.
        Se ler, a garra fica onde leu; se nao, volta para a pose original.
        """
        q = self.cfg.pontos["mostrar_apriltag"]
        self.b.ir_para("mostrar_apriltag", q)
        for tentativa in range(self.tentativas_tag):
            lido = self._ler_uma(f"{tentativa + 1}/{self.tentativas_tag}")
            if lido is not None:
                return lido
        for desvio in self.cfg.busca_j3:
            g = math.degrees(desvio)
            self.b.ir_para(f"mostrar_apriltag j3{g:+.0f}", [q[0], q[1], q[2] + desvio])
            lido = self._ler_uma(f"j3 {g:+.0f} graus")
            if lido is not None:
                return lido
        if self.cfg.busca_j3:
            self.b.ir_para("mostrar_apriltag", q)
        return None

    def _ir(self, nome):
        self.b.ir_para(nome, self.cfg.pontos[nome])

    def _ir_slot(self, slot):
        pose = self.cfg.slots[slot]
        self._ir("acima_slots")
        if pose.y_plataforma is not None:
            self.b.ir_y(pose.y_plataforma)
        self.b.entrar_slot(pose.juntas, self.cfg.subida_m)

    def _sair_slot(self):
        self.b.sair_slot(self.cfg.subida_m)
        self._ir("acima_slots")

    # ---- guardar ---------------------------------------------------------------------------
    def guardar(self, cubo_id_esperado: Optional[int] = None) -> int:
        """Confirma o cubo nos dedos, le o ID e guarda no menor slot livre. Devolve o slot.

        cubo_id_esperado (opcional) e o ID que a camera viu antes da pegada; se a leitura na
        pose mostrar_apriltag falhar, ele e usado. Se as duas faltarem, nao guarda.
        Estoque cheio: o ID e lido mesmo assim e vai em ErroSlots.cubo_na_garra.
        """
        lido = self.ler_tag()
        if lido is None and cubo_id_esperado is None:
            raise ErroSlots("tag nao lido e sem ID esperado: cubo continua na garra",
                            cubo_na_garra=-1)
        if lido is not None and cubo_id_esperado is not None and lido != cubo_id_esperado:
            self.log(f"AVISO: esperava o cubo {cubo_id_esperado}, a camera leu {lido}; vale o lido")
        cubo_id = lido if lido is not None else cubo_id_esperado
        if self.estoque.primeiro_disponivel() is None:
            raise ErroSlots(f"estoque cheio: cubo {cubo_id} continua na garra",
                            cubo_na_garra=cubo_id)

        slot = self.estoque.reservar(cubo_id)
        self.log(f"guardando cubo {cubo_id} no slot {slot}")
        soltou = False
        try:
            self._ir_slot(slot)
            self.b.abrir_dedos()
            soltou = True
            self.estoque.confirmar(slot)
            self._salvar(self.estoque)
            self._sair_slot()
            self._ir("pronta")
        except Exception as exc:
            if not soltou:
                self.estoque.cancelar(slot)
                raise ErroSlots(f"falha ao guardar no slot {slot}: {exc}",
                                cubo_na_garra=cubo_id) from exc
            raise ErroSlots(f"cubo {cubo_id} ficou no slot {slot}, mas a saida falhou: {exc}",
                            cubo_na_garra=None) from exc
        return slot

    # ---- retirar ---------------------------------------------------------------------------
    def retirar(self, slot: Optional[int] = None, cubo_id: Optional[int] = None) -> ResultadoRetirada:
        """Tira um cubo de um slot (pelo numero ou pelo ID) e mostra o tag para a camera.

        A garra termina na pose mostrar_apriltag, com o cubo nos dedos: quem chama (as missoes)
        decide o que fazer com ele a partir de ResultadoRetirada.cubo_id.
        """
        if (slot is None) == (cubo_id is None):
            raise ValueError("passe slot OU cubo_id")
        if slot is None:
            slot = self.estoque.slot_do_cubo(cubo_id)
            if slot is None:
                raise ErroSlots(f"cubo {cubo_id} nao esta em nenhum slot")
        esperado = self.estoque.cubo_no_slot(slot)
        if esperado is None:
            raise ErroSlots(f"slot {slot} esta vazio")

        try:
            self.b.abrir_dedos()
            self._ir_slot(slot)
            self.b.fechar_dedos()
        except Exception as exc:
            raise ErroSlots(f"falha ao chegar no slot {slot}: {exc}") from exc

        # Sem sensor nos dedos: o registro so muda depois que a camera ler o tag.
        try:
            self._sair_slot()
        except Exception as exc:
            raise ErroSlots(f"movimento falhou saindo do slot {slot} (registro mantido): {exc}",
                            cubo_na_garra=-1) from exc

        lido = self.ler_tag()
        if lido is None:
            self.log(f"AVISO: tag nao lido; devolvendo ao slot {slot} (registro mantido)")
            try:
                self._ir_slot(slot)
                self.b.abrir_dedos()
                self._sair_slot()
            except Exception as exc:
                raise ErroSlots(f"tag nao lido e a devolucao ao slot {slot} falhou: {exc}",
                                cubo_na_garra=-1) from exc
            raise ErroSlots(f"tag nao lido: soltou de novo no slot {slot} (registro mantido)",
                            cubo_na_garra=None)

        self.estoque.retirar_por_slot(slot)
        if lido != esperado:
            self.log(f"AVISO: slot {slot} tinha {lido}, nao {esperado}")
            outro = self.estoque.slot_do_cubo(lido)
            if outro is not None:
                # O registro tinha o cubo lido em outro slot: provavelmente estao trocados.
                self.estoque.corrigir(outro, esperado)
                self.log(f"AVISO: slot {outro} corrigido para {esperado}")
        self._salvar(self.estoque)
        return ResultadoRetirada(lido, slot, True, esperado)
