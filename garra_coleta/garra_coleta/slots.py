"""Estoque dos tres slots da garra (sem ROS).

Slots numerados de 0 a 2 (0 = limite da esquerda, 1 = meio, 2 = limite da direita). Cada slot guarda o ID do AprilTag do cubo (int) ou None.
Ao guardar, sempre se usa o MENOR slot livre.

Guardar e feito em duas fases, porque o braco leva segundos ate soltar o cubo:
  slot = estoque.reservar(cubo_id)    # escolhe o slot antes de mover
  ... braco vai ate o slot e solta ...
  estoque.confirmar(slot)             # ou estoque.cancelar(slot) se falhou antes de soltar
Um slot reservado nao e oferecido a outro cubo.

O estado pode ser salvo em JSON (salvar/carregar), para nao perder o estoque se o no reiniciar
no meio da partida.
"""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

CAPACIDADE = 3


@dataclass(frozen=True)
class EstadoSlots:
    slots: Dict[int, Optional[int]]
    reservados: frozenset = frozenset()

    @property
    def ocupados(self) -> int:
        return sum(cubo is not None for cubo in self.slots.values())

    @property
    def livres(self) -> int:
        return len(self.slots) - self.ocupados

    def para_dict(self) -> dict:
        return {
            "slots": {str(s): c for s, c in sorted(self.slots.items())},
            "reservados": sorted(self.reservados),
            "ocupados": self.ocupados,
        }


def _id(cubo_id) -> int:
    if isinstance(cubo_id, bool):
        raise ValueError("cubo_id invalido")
    try:
        valor = int(str(cubo_id).strip())
    except ValueError:
        raise ValueError(f"cubo_id invalido: {cubo_id!r} (use o ID do AprilTag)") from None
    if valor < 0:
        raise ValueError(f"cubo_id invalido: {valor}")
    return valor


class GerenciadorSlots:

    def __init__(self, capacidade: int = CAPACIDADE):
        if capacidade < 1:
            raise ValueError("capacidade deve ser pelo menos 1")
        self.capacidade = capacidade
        self.reset()

    # ---- consulta -------------------------------------------------------------------------
    def estado(self) -> EstadoSlots:
        return EstadoSlots(dict(self.slots), frozenset(self.reservados))

    def primeiro_disponivel(self) -> Optional[int]:
        return next((s for s in range(self.capacidade)
                     if self.slots[s] is None and s not in self.reservados), None)

    def slot_do_cubo(self, cubo_id) -> Optional[int]:
        cubo_id = _id(cubo_id)
        return next((s for s, c in self.slots.items() if c == cubo_id), None)

    def cubo_no_slot(self, slot: int) -> Optional[int]:
        self._validar_slot(slot)
        return self.slots[slot]

    # ---- alteracao ------------------------------------------------------------------------
    def reset(self) -> EstadoSlots:
        self.slots: Dict[int, Optional[int]] = {s: None for s in range(self.capacidade)}
        self.reservados = set()
        return self.estado()

    def reservar(self, cubo_id) -> int:
        cubo_id = _id(cubo_id)
        if self.slot_do_cubo(cubo_id) is not None:
            raise ValueError(f"cubo {cubo_id} ja esta no slot {self.slot_do_cubo(cubo_id)}")
        slot = self.primeiro_disponivel()
        if slot is None:
            raise RuntimeError("estoque cheio: todos os slots estao ocupados")
        self.slots[slot] = cubo_id
        self.reservados.add(slot)
        return slot

    def confirmar(self, slot: int) -> None:
        if slot not in self.reservados:
            raise RuntimeError(f"slot {slot} nao estava reservado")
        self.reservados.discard(slot)

    def cancelar(self, slot: int) -> None:
        if slot not in self.reservados:
            raise RuntimeError(f"slot {slot} nao estava reservado")
        self.reservados.discard(slot)
        self.slots[slot] = None

    def guardar(self, cubo_id) -> int:
        """Reserva e confirma de uma vez (para testes ou correcao manual)."""
        slot = self.reservar(cubo_id)
        self.confirmar(slot)
        return slot

    def retirar_por_slot(self, slot: int) -> int:
        self._validar_slot(slot)
        if slot in self.reservados:
            raise RuntimeError(f"slot {slot} esta reservado (guardando agora)")
        cubo_id = self.slots[slot]
        if cubo_id is None:
            raise RuntimeError(f"slot {slot} esta vazio")
        self.slots[slot] = None
        return cubo_id

    def retirar_por_cubo(self, cubo_id) -> int:
        slot = self.slot_do_cubo(cubo_id)
        if slot is None:
            raise RuntimeError(f"cubo {_id(cubo_id)} nao esta em nenhum slot")
        self.retirar_por_slot(slot)
        return slot

    def corrigir(self, slot: int, cubo_id) -> None:
        """Escreve o que a camera leu (ou None) no slot, sem as regras de ordem."""
        self._validar_slot(slot)
        self.reservados.discard(slot)
        if cubo_id is None:
            self.slots[slot] = None
            return
        cubo_id = _id(cubo_id)
        outro = self.slot_do_cubo(cubo_id)
        if outro is not None and outro != slot:
            self.slots[outro] = None
        self.slots[slot] = cubo_id

    # ---- persistencia ---------------------------------------------------------------------
    def salvar(self, caminho) -> None:
        """Grava so os slots confirmados; reservas em andamento contam como vazias."""
        caminho = Path(caminho)
        caminho.parent.mkdir(parents=True, exist_ok=True)
        dados = {str(s): (None if s in self.reservados else c) for s, c in self.slots.items()}
        temp = caminho.with_suffix(caminho.suffix + ".tmp")
        temp.write_text(json.dumps({"slots": dados}, indent=2), encoding="utf-8")
        temp.replace(caminho)

    @classmethod
    def carregar(cls, caminho, capacidade: int = CAPACIDADE) -> "GerenciadorSlots":
        g = cls(capacidade)
        caminho = Path(caminho)
        if caminho.exists():
            dados = json.loads(caminho.read_text(encoding="utf-8")).get("slots", {})
            for s, c in dados.items():
                if int(s) in g.slots and c is not None:
                    g.corrigir(int(s), c)
        return g

    def _validar_slot(self, slot) -> None:
        if slot not in self.slots:
            raise ValueError(f"slot {slot} invalido (0 a {self.capacidade - 1})")
