import math
import tempfile
import unittest
from pathlib import Path

from garra_coleta.sequencia_slots import (
    ConfigSlots, ErroSlots, PoseSlot, SequenciaSlots, carregar_config)
from garra_coleta.slots import GerenciadorSlots

CONFIG = Path(__file__).resolve().parents[1] / "config" / "slots.yaml"


class TestGerenciadorSlots(unittest.TestCase):
    def test_comeca_totalmente_vazio(self):
        self.assertEqual(GerenciadorSlots().estado().para_dict()["slots"],
                         {"0": None, "1": None, "2": None})

    def test_guarda_no_menor_slot_disponivel(self):
        g = GerenciadorSlots()
        self.assertEqual(g.guardar(12), 0)
        self.assertEqual(g.guardar(7), 1)
        g.retirar_por_slot(0)
        self.assertEqual(g.guardar(3), 0)
        self.assertEqual(g.guardar(5), 2)

    def test_retirada_por_slot_e_por_cubo(self):
        g = GerenciadorSlots()
        g.guardar(12)
        g.guardar(7)
        self.assertEqual(g.retirar_por_slot(1), 7)
        self.assertEqual(g.guardar(9), 1)
        self.assertEqual(g.retirar_por_cubo(12), 0)
        self.assertEqual(g.estado().slots, {0: None, 1: 9, 2: None})

    def test_reserva_nao_e_oferecida_e_cancelar_libera(self):
        g = GerenciadorSlots()
        s = g.reservar(4)
        self.assertEqual(g.primeiro_disponivel(), 1)
        with self.assertRaises(RuntimeError):
            g.retirar_por_slot(s)
        g.cancelar(s)
        self.assertEqual(g.primeiro_disponivel(), 0)

    def test_cubo_repetido_e_id_invalido(self):
        g = GerenciadorSlots()
        g.guardar(1)
        with self.assertRaises(ValueError):
            g.guardar("1")
        with self.assertRaises(ValueError):
            g.guardar("abc")

    def test_estoque_cheio(self):
        g = GerenciadorSlots()
        for c in (1, 2, 3):
            g.guardar(c)
        with self.assertRaises(RuntimeError):
            g.guardar(5)

    def test_salvar_e_carregar_ignora_reserva(self):
        g = GerenciadorSlots()
        g.guardar(8)
        g.reservar(9)
        with tempfile.TemporaryDirectory() as d:
            arq = Path(d) / "estado.json"
            g.salvar(arq)
            self.assertEqual(GerenciadorSlots.carregar(arq).estado().slots,
                             {0: 8, 1: None, 2: None})


class BracoFake:
    def __init__(self, falhar_em=None):
        self.passos, self.falhar_em = [], falhar_em

    def _p(self, nome):
        self.passos.append(nome)
        if nome == self.falhar_em:
            raise RuntimeError("falha simulada")

    def ir_para(self, nome, q):
        self.q = list(q)
        self._p(nome)
    def entrar_slot(self, q, subida): self._p(f"entrar {q[0]:.0f}")
    def sair_slot(self, subida): self._p("sair")
    def abrir_dedos(self): self._p("abrir")
    def fechar_dedos(self): self._p("fechar")
    def ir_y(self, y): self._p(f"y {y}")


class LeitorFake:
    def __init__(self, *leituras):
        self.leituras = list(leituras)

    def ler(self, timeout_s):
        return self.leituras.pop(0) if self.leituras else []


def config(busca=()):
    pontos = {n: [0.0, 0.0, 0.0] for n in ("pronta", "mostrar_apriltag", "acima_slots")}
    slots = {s: PoseSlot([float(s), 0.0, 0.0], -0.10 if s == 2 else None) for s in range(3)}
    return ConfigSlots(pontos, slots, 0.04, tuple(math.radians(g) for g in busca))


class TestSequenciaSlots(unittest.TestCase):
    def seq(self, braco, leitor, **kw):
        return SequenciaSlots(braco, leitor, config(), log=lambda _m: None, **kw)

    def test_guardar_ordem_dos_passos(self):
        b = BracoFake()
        s = self.seq(b, LeitorFake([5]))
        self.assertEqual(s.guardar(), 0)
        self.assertEqual(b.passos, ["mostrar_apriltag", "acima_slots", "entrar 0", "abrir",
                                    "sair", "acima_slots", "pronta"])
        self.assertEqual(s.estoque.estado().slots[0], 5)

    def test_guardar_vale_o_tag_lido_e_usa_esperado_se_nao_ler(self):
        s = self.seq(BracoFake(), LeitorFake([6], [], []))
        s.guardar(cubo_id_esperado=2)
        s.guardar(cubo_id_esperado=2)
        self.assertEqual(s.estoque.estado().slots, {0: 6, 1: 2, 2: None})

    def test_guardar_sem_tag_nem_esperado_nao_mexe(self):
        b = BracoFake()
        with self.assertRaises(ErroSlots) as e:
            self.seq(b, LeitorFake([], [])).guardar()
        self.assertEqual(e.exception.cubo_na_garra, -1)
        self.assertEqual(b.passos, ["mostrar_apriltag"])

    def test_dois_tags_na_imagem_nao_conta(self):
        s = self.seq(BracoFake(), LeitorFake([1, 2], [3]))
        s.guardar()
        self.assertEqual(s.estoque.estado().slots[0], 3)

    def test_falha_antes_de_soltar_libera_reserva(self):
        s = self.seq(BracoFake(falhar_em="entrar 0"), LeitorFake([5]))
        with self.assertRaises(ErroSlots) as e:
            s.guardar()
        self.assertEqual(e.exception.cubo_na_garra, 5)
        self.assertIsNone(s.estoque.estado().slots[0])

    def test_falha_depois_de_soltar_mantem_cubo_no_slot(self):
        s = self.seq(BracoFake(falhar_em="sair"), LeitorFake([5]))
        with self.assertRaises(ErroSlots) as e:
            s.guardar()
        self.assertIsNone(e.exception.cubo_na_garra)
        self.assertEqual(s.estoque.estado().slots[0], 5)

    def test_slot_com_plataforma_y(self):
        b = BracoFake()
        s = self.seq(b, LeitorFake())
        for c in (1, 2):
            s.estoque.guardar(c)
        s.leitor = LeitorFake([4])
        self.assertEqual(s.guardar(), 2)
        self.assertIn("y -0.1", b.passos)

    def test_retirar_por_cubo(self):
        b = BracoFake()
        s = self.seq(b, LeitorFake([7]))
        s.estoque.guardar(3)
        s.estoque.guardar(7)
        r = s.retirar(cubo_id=7)
        self.assertEqual((r.cubo_id, r.slot, r.confirmado), (7, 1, True))
        self.assertEqual(b.passos, ["abrir", "acima_slots", "entrar 1", "fechar", "sair",
                                    "acima_slots", "mostrar_apriltag"])
        self.assertEqual(s.estoque.estado().slots, {0: 3, 1: None, 2: None})

    def test_retirar_sem_leitura_devolve_ao_slot_e_mantem_registro(self):
        b = BracoFake()
        s = self.seq(b, LeitorFake())
        s.estoque.guardar(3)
        with self.assertRaises(ErroSlots) as e:
            s.retirar(cubo_id=3)
        self.assertIsNone(e.exception.cubo_na_garra)
        self.assertEqual(b.passos, ["abrir", "acima_slots", "entrar 0", "fechar", "sair",
                                    "acima_slots", "mostrar_apriltag",
                                    "acima_slots", "entrar 0", "abrir", "sair", "acima_slots"])
        self.assertEqual(s.estoque.estado().slots[0], 3)

    def test_retirar_corrige_cubos_trocados(self):
        s = self.seq(BracoFake(), LeitorFake([9]))
        s.estoque.guardar(3)
        s.estoque.guardar(9)
        r = s.retirar(slot=0)
        self.assertEqual((r.cubo_id, r.esperado), (9, 3))
        self.assertEqual(s.estoque.estado().slots, {0: None, 1: 3, 2: None})

    def test_busca_em_j3_ate_ler(self):
        b = BracoFake()
        s = SequenciaSlots(b, LeitorFake([], [], [], [4]), config((8, -8, 15)),
                           log=lambda _m: None)
        self.assertEqual(s.guardar(), 0)
        self.assertEqual(b.passos[:3], ["mostrar_apriltag", "mostrar_apriltag j3+8",
                                        "mostrar_apriltag j3-8"])
        self.assertEqual(s.estoque.estado().slots[0], 4)

    def test_busca_em_j3_so_mexe_j3_e_volta_se_falhar(self):
        b = BracoFake()
        s = SequenciaSlots(b, LeitorFake(), config((10,)), log=lambda _m: None)
        s.cfg.pontos["mostrar_apriltag"] = [0.1, 0.2, 0.3]
        self.assertIsNone(s.ler_tag())
        self.assertEqual(b.passos, ["mostrar_apriltag", "mostrar_apriltag j3+10",
                                    "mostrar_apriltag"])
        self.assertEqual(b.q, [0.1, 0.2, 0.3])

    def test_estoque_cheio_le_o_id_e_nao_guarda(self):
        b = BracoFake()
        s = self.seq(b, LeitorFake([7]))
        for c in (1, 2, 3):
            s.estoque.guardar(c)
        with self.assertRaises(ErroSlots) as e:
            s.guardar()
        self.assertEqual(e.exception.cubo_na_garra, 7)
        self.assertEqual(b.passos, ["mostrar_apriltag"])

    def test_retirar_slot_vazio(self):
        with self.assertRaises(ErroSlots):
            self.seq(BracoFake(), LeitorFake()).retirar(slot=2)


class TestConfig(unittest.TestCase):
    def test_lista_o_que_falta(self):
        with tempfile.TemporaryDirectory() as d:
            arq = Path(d) / "slots.yaml"
            arq.write_text("pontos: {pronta: [0, 0, 0], mostrar_apriltag: null}\n"
                           "slots: {0: {juntas: [0, 0, 0], y_cm: 11.0}}\n", encoding="utf-8")
            with self.assertRaises(ValueError) as e:
                carregar_config(arq)
            msg = str(e.exception)
            for falta in ("mostrar_apriltag", "acima_slots", "slot 1", "slot 2"):
                self.assertIn(falta, msg)
            self.assertNotIn("slot 0", msg)
            self.assertNotIn("slot 3", msg)
            cfg = carregar_config(arq, exigir_completa=False)
            self.assertAlmostEqual(cfg.slots[0].y_plataforma, 0.11)

    def test_config_do_pacote_carrega(self):
        cfg = carregar_config(CONFIG, exigir_completa=False)
        self.assertEqual(cfg.pontos["pronta"], [0.12, -0.95, 0.87])
        self.assertEqual(len(cfg.slots), 3)
        self.assertEqual(len(cfg.busca_j3), 4)


class TestGravarPose(unittest.TestCase):
    def test_grava_ponto_e_slot_mantendo_comentarios(self):
        from garra_coleta.ajustar_poses import gravar_no_yaml
        texto = CONFIG.read_text(encoding="utf-8")
        novo = gravar_no_yaml(texto, "mostrar_apriltag", [0.1, -0.2, 0.3])
        novo = gravar_no_yaml(novo, "slot2", [1.0, 0.5, -0.25])
        self.assertIn("  mostrar_apriltag: [0.100, -0.200, 0.300]\n", novo)
        self.assertIn("  2: {juntas: [1.000, 0.500, -0.250], y_cm: null}   # limite da direita", novo)
        self.assertEqual(texto.count("#"), novo.count("#"))
        import yaml
        dados = yaml.safe_load(novo)
        self.assertEqual(dados["slots"][2]["juntas"], [1.0, 0.5, -0.25])
        with self.assertRaises(ValueError):
            gravar_no_yaml(texto, "slot3", [0, 0, 0])


if __name__ == "__main__":
    unittest.main()
