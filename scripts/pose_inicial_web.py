#!/usr/bin/env python3
"""Pagina web para escolher e salvar a posicao inicial do braco (J1-J3).

Com o MoveIt rodando (garra_real.launch.py ou demo.launch.py):

    python3 scripts/pose_inicial_web.py          # abre http://localhost:8090

Na pagina: um controle por junta (em graus), posicao da ponta e checagem de colisao ao vivo,
e os botoes Mover (planeja com o MoveIt, sem colisao), Salvar como inicial e Ir para inicial.
A pose salva fica em ~/.config/garra/pose_inicial.json; `mover_junta.py inicial` vai ate ela.
"""

import argparse
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'garra_coleta'))
import rclpy  # noqa: E402
from garra_coleta.cliente_garra import ClienteGarra, LIMITES, JUNTAS, SUCESSO  # noqa: E402

ARQ = Path(os.environ.get('GARRA_POSE_INICIAL', Path.home() / '.config' / 'garra' / 'pose_inicial.json'))
NOMES = ['J1 (base)', 'J2 (meio)', 'J3 (ponta)']

PAGINA = """<!doctype html><html lang="pt-BR"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Posicao inicial</title>
<style>
body{font-family:system-ui,sans-serif;max-width:560px;margin:24px auto;padding:0 16px;background:#f6f6f4;color:#222}
h1{font-size:20px}.j{background:#fff;border-radius:8px;padding:12px;margin:10px 0}
.j label{display:flex;justify-content:space-between;font-weight:600}
input[type=range]{width:100%}input[type=number]{width:80px}
button{font-size:15px;padding:9px 14px;margin:4px 4px 4px 0;border-radius:6px;border:1px solid #888;background:#fff;cursor:pointer}
button.p{background:#1f6feb;color:#fff;border-color:#1f6feb}button:disabled{opacity:.5}
#info{background:#fff;border-radius:8px;padding:12px;font-family:monospace;white-space:pre-line}
.ok{color:#16793a}.ruim{color:#c62828;font-weight:700}
</style></head><body>
<h1>Posição inicial da garra</h1>
<div id="juntas"></div>
<div id="info">carregando...</div>
<p><button onclick="lerAtual()">Ler posição atual</button>
<button class="p" id="bm" onclick="acao('mover')">Mover</button>
<button onclick="acao('salvar')">Salvar como inicial</button>
<button onclick="acao('ir_inicial')">Ir para inicial</button></p>
<p id="msg"></p>
<script>
let lim=[],q=[0,0,0],t=null;
const g=r=>(r*180/Math.PI).toFixed(1), rad=d=>d*Math.PI/180;
async function api(c,d){const r=await fetch(c,{method:d?'POST':'GET',body:d?JSON.stringify(d):undefined});return r.json()}
function montar(nomes){let h='';nomes.forEach((n,i)=>{h+=`<div class="j"><label>${n}<span><input type="number" step="1" id="n${i}"> graus</span></label>
<input type="range" step="0.5" id="r${i}" min="${g(lim[i][0])}" max="${g(lim[i][1])}"></div>`});
document.getElementById('juntas').innerHTML=h;
nomes.forEach((n,i)=>{for(const p of ['r','n'])document.getElementById(p+i).oninput=e=>{q[i]=rad(+e.target.value);mostrar();checar()}})}
function mostrar(){q.forEach((v,i)=>{document.getElementById('r'+i).value=g(v);document.getElementById('n'+i).value=g(v)})}
function checar(){clearTimeout(t);t=setTimeout(async()=>{const r=await api('/fk',{q});
document.getElementById('info').innerHTML=`Ponta: x=${(r.x*100).toFixed(1)} cm do centro (${((r.x-0.275)*100).toFixed(1)} da borda), z=${(r.z*100).toFixed(1)} cm do chao\\n`+
(r.valido?'<span class="ok">Sem colisao no modelo</span>':`<span class="ruim">COLISAO: ${r.contatos}</span>`)+
`\\nAtual: ${r.atual.map(g).join(' / ')} graus\\nInicial salva: ${r.inicial?r.inicial.map(g).join(' / ')+' graus':'nenhuma'}`;
document.getElementById('bm').disabled=!r.valido},150)}
async function lerAtual(){const r=await api('/estado');q=r.atual;mostrar();checar()}
async function acao(a){const m=document.getElementById('msg');m.textContent='...';
document.querySelectorAll('button').forEach(b=>b.disabled=true);
try{const r=await api('/'+a,{q});m.textContent=r.msg;if(r.atual){q=r.atual;mostrar()}}catch(e){m.textContent='erro: '+e}
document.querySelectorAll('button').forEach(b=>b.disabled=false);checar()}
(async()=>{const r=await api('/estado');lim=r.limites;q=r.atual;montar(r.nomes);mostrar();checar()})();
</script></body></html>"""


def ler_inicial():
    try:
        return json.loads(ARQ.read_text())['q']
    except (OSError, ValueError, KeyError):
        return None


class Servidor:
    def __init__(self, cliente, velocidade):
        self.c, self.v = cliente, velocidade
        self.trava = threading.Lock()   # uma chamada ao ROS por vez

    def estado(self, _):
        return dict(atual=self.c.juntas_atuais(), inicial=ler_inicial(), nomes=NOMES,
                    limites=[list(l) for l in LIMITES])

    def fk(self, d):
        q = d['q']
        p = self.c.fk(q, 'robo_base').position
        ok, contatos = self.c.valido(q)
        return dict(x=p.x, z=p.z, valido=ok, contatos=', '.join('-'.join(c) for c in contatos),
                    atual=self.c.juntas_atuais(), inicial=ler_inicial())

    def mover(self, d, q=None):
        q = [float(v) for v in (q or d['q'])]
        for v, j, (lo, hi) in zip(q, JUNTAS, LIMITES):
            if not lo <= v <= hi:
                return dict(msg=f'{j} fora do limite [{lo}, {hi}]')
        ok, contatos = self.c.valido(q)
        if not ok:
            return dict(msg=f'Nao movi: colisao no modelo ({contatos})')
        traj, codigo = self.c.planejar_juntas(self.c.juntas_atuais(), q, self.v)
        if codigo != SUCESSO:
            return dict(msg=f'Planejamento falhou (codigo {codigo}). Tente um alvo intermediario.')
        self.c.executar(traj)
        atual = self.esperar_parar()
        erro = max(abs(a - b) for a, b in zip(atual, q))
        texto = 'Chegou.' if erro < 0.02 else f'Parou a {math.degrees(erro):.1f} graus do alvo.'
        return dict(msg=texto, atual=atual)

    def esperar_parar(self, maximo_s=60.0):
        """O resultado do MoveIt pode chegar antes do braco terminar: espera as juntas pararem
        (menos de 0,002 rad de mudanca em 0,5 s) antes de comparar com o alvo."""
        fim = time.monotonic() + maximo_s
        anterior = self.c.juntas_atuais()
        while time.monotonic() < fim:
            time.sleep(0.5)
            atual = self.c.juntas_atuais()
            if max(abs(a - b) for a, b in zip(atual, anterior)) < 0.002:
                return atual
            anterior = atual
        return anterior

    def salvar(self, d):
        q = [round(float(v), 4) for v in d['q']]
        ARQ.parent.mkdir(parents=True, exist_ok=True)
        ARQ.write_text(json.dumps({'q': q, 'juntas': JUNTAS}, indent=2))
        return dict(msg=f'Salvo em {ARQ}: ' + ' / '.join(f'{math.degrees(v):.1f}' for v in q) + ' graus')

    def ir_inicial(self, d):
        q = ler_inicial()
        if q is None:
            return dict(msg='Nenhuma posicao inicial salva ainda.')
        return self.mover(d, q)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--porta', type=int, default=8090)
    ap.add_argument('--velocidade', type=float, default=0.1, help='escala de velocidade (0-1)')
    args = ap.parse_args()

    rclpy.init()
    c = ClienteGarra('pose_inicial_web')
    c.aguardar()
    s = Servidor(c, args.velocidade)
    rotas = {'/estado': s.estado, '/fk': s.fk, '/mover': s.mover, '/salvar': s.salvar,
             '/ir_inicial': s.ir_inicial}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _responder(self, corpo, tipo='application/json'):
            dados = corpo.encode()
            self.send_response(200)
            self.send_header('Content-Type', tipo + '; charset=utf-8')
            self.send_header('Content-Length', str(len(dados)))
            self.end_headers()
            self.wfile.write(dados)

        def _rota(self, d):
            f = rotas.get(self.path)
            if f is None:
                self.send_error(404)
                return
            try:
                with s.trava:
                    r = f(d)
            except Exception as e:   # mostra o erro na pagina em vez de derrubar o servidor
                r = dict(msg=f'erro: {e}')
            self._responder(json.dumps(r))

        def do_GET(self):
            if self.path == '/':
                self._responder(PAGINA, 'text/html')
            else:
                self._rota({})

        def do_POST(self):
            n = int(self.headers.get('Content-Length', 0))
            self._rota(json.loads(self.rfile.read(n) or b'{}'))

    srv = HTTPServer(('127.0.0.1', args.porta), Handler)
    print(f'Abra http://localhost:{args.porta}  (Ctrl+C para sair)', flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        c.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
