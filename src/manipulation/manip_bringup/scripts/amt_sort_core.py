#!/usr/bin/env python3
"""AMT refeita (2026-11-16, branch mudanca_pp): ordenar os blocos da mesa de precisao. Sem ROS.

Pedido do operador: "monte a task da AMT de novo, ela precisa organizar as
tags que vem do arquivo da competicao em ordem crescente". Os blocos comecam
nos slots em ordem desconhecida (o arquivo nao diz) e terminam com os ids em
ordem crescente da esquerda para a direita (ou no alvo explicito do arquivo).

Duas contas, as duas so' com IDS de tag (nada de coordenada guardada, entao a
base e o braco podem se mexer no meio da rotina sem invalidar nada):
  Fileira   quem esta' em qual slot, pela ORDEM dos blocos na fila vista pela
            camera. Com os blocos dentro dos furos a tag do slot NAO aparece (o
            bloco a tapa, visto do lado do robo: 1a tentativa no robo, 16/11
            21h47, "nao vi bloco em slot nenhum"); o que se ve e' a fila de tags
            dos blocos, com passo regular, e a tag de slot VAZIO.
  planejar  sequencia de pick/place usando os potes de bordo como espera:
            tira um bloco do ciclo, roda os outros e fecha (2 potes bastam).
"""

from collections import Counter

import numpy as np

# Bloco "na fileira": centro da tag dele a ate' isto da curva das tags dos slots,
# em LADOS da tag do slot (31,8 mm). O bloco fica ~46 mm antes da tag; visto de
# longe e de lado, o topo dele (4 cm acima do tampo) aparece ate' ~40 mm
# deslocado. 3 lados = 95 mm; o slot vizinho fica a 107 mm AO LONGO da fileira.
DIST_MAX_LADOS = 3.0


def _centro(cantos):
    return np.asarray(cantos, float).reshape(4, 2).mean(axis=0)


def _lado_px(cantos):
    c = np.asarray(cantos, float).reshape(4, 2)
    return float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))


def arranjo_do_quadro(tags, slot_ids, cubo_ids=None, tamanho=(640, 480)):
    """Quem esta' em qual slot NESTE quadro.

    tags: {id: 4 cantos px}; slot_ids: ids das tags dos slots na ordem da
    fileira; cubo_ids: ids aceitos como bloco (None = qualquer tag que nao seja
    de slot). Devolve (cubo_no_slot {cubo: slot}, slots_vazios [slot], soltos
    [bloco visto fora da fileira]).

    Um slot so' sai como VAZIO se o lugar onde o bloco dele apareceria cabe
    inteiro na imagem: na borda, a tag do slot aparece e a do bloco (mais alta
    e mais perto da camera) fica cortada — dizer "vazio" ali e' soltar um bloco
    em cima de outro.

    Tudo em pixels, sem retificar: a posicao (u, v) das tags dos slots em funcao
    do INDICE do slot e' uma curva suave (reta com perspectiva); o bloco fica no
    indice mais proximo dele. Vale tambem para o slot cuja tag o bloco tapou.
    """
    slot_ids = list(slot_ids)
    vistos = [s for s in slot_ids if s in tags]
    if not vistos:
        return {}, [], []
    idx = np.array([slot_ids.index(s) for s in vistos], float)
    cen = np.array([_centro(tags[s]) for s in vistos])
    lado = np.array([_lado_px(tags[s]) for s in vistos])
    if len(vistos) >= 2:
        grau = min(2, len(vistos) - 1)
        grade = np.arange(idx.min() - 1.0, idx.max() + 1.0001, 0.02)   # ate' 1 slot alem dos vistos
        curva = np.column_stack([np.polyval(np.polyfit(idx, cen[:, k], grau), grade) for k in (0, 1)])
        lados = np.polyval(np.polyfit(idx, lado, 1), grade)
    else:
        grade, curva, lados = idx, cen, lado
    cubo_no_slot, soltos = {}, []
    for t, cantos in tags.items():
        if t in slot_ids or (cubo_ids is not None and t not in cubo_ids):
            continue
        d = np.linalg.norm(curva - _centro(cantos), axis=1)
        k = int(np.argmin(d))
        i = int(round(float(grade[k])))
        if d[k] <= DIST_MAX_LADOS * lados[k] and 0 <= i < len(slot_ids):
            cubo_no_slot[t] = slot_ids[i]
        else:
            soltos.append(t)
    ocupados = set(cubo_no_slot.values())
    vazios = []
    for s, c, lp in zip(vistos, cen, lado):
        r = (DIST_MAX_LADOS + 0.7) * lp                  # onde o bloco poderia estar + meia tag
        if s not in ocupados and r <= c[0] <= tamanho[0] - r and r <= c[1] <= tamanho[1] - r:
            vazios.append(s)
    return cubo_no_slot, vazios, soltos


# Passo entre blocos vizinhos na imagem, em LADOS da tag: medido em 16/11 21h47
# (6 blocos nos slots, de home e de pegar_obj): 3,3 a 3,6. Slot vazio no meio = 2 passos.
PASSO_EM_LADOS = 3.45


class Fileira:
    """Junta quadros e diz quem esta' em qual slot pela ordem na fila.

    Cada quadro da' so' relacoes ("o 4 esta' 1 passo a' esquerda do 6"); quadros
    de vistas diferentes (olhando para a esquerda, para a direita, base
    deslocada) se emendam pelos blocos em comum. A posicao absoluta vem de:
      - tag de slot COLADA num bloco (mesmo u): o bloco esta' NESSE slot. E' o
        que se ve das poses laterais, mais altas (17/11 16h12: "3@145 104@151");
        tratada como slot vazio, ela embaralhava a fila e a AMT leu a mesa errada;
      - tag de slot sozinha, longe da borda: slot VAZIO, entra na fila.
    Sem nenhuma das duas vale a hipotese de que a fila comeca no primeiro slot
    (ancorado=False). Leitura contraditoria (dois blocos no mesmo slot) = {}.

    Convencao da imagem nas poses de olhar (pegar_obj, tag_esquerda,
    tag_direita): o slot de id MAIOR fica a' ESQUERDA (conferido nos quadros de
    16/11: 106..101 e 107..103 da esquerda para a direita).
    """

    def __init__(self, slot_ids, minimo=2, largura=640):
        self.slot_ids, self.minimo, self.largura = list(slot_ids), minimo, largura
        self._passos = {}              # (a, b), a a' esquerda de b na imagem -> Counter(passo)
        self._vistos = Counter()
        self._colado = Counter()       # (bloco, slot): tag do slot vista colada no bloco
        self.quadros = 0

    def juntar(self, tags):
        itens = [(i, _centro(c), _lado_px(c)) for i, c in tags.items()]
        if not itens:
            return
        self.quadros += 1
        # so' o que esta' na fileira: fora da faixa da mediana fica de fora (bloco na garra, no chao)
        v_med, l_med = np.median([c[1] for _, c, _ in itens]), np.median([lp for _, _, lp in itens])
        itens = [it for it in itens if abs(it[1][1] - v_med) <= 3.0 * l_med]
        fila = [it for it in itens if it[0] not in self.slot_ids]
        blocos = list(fila)
        for s, cs, ls in (it for it in itens if it[0] in self.slot_ids):
            folga = 0.4 * PASSO_EM_LADOS * ls                  # o vizinho fica a 1 passo; colado, a < 0,3
            perto = min(blocos, key=lambda b: abs(b[1][0] - cs[0]), default=None)
            if perto is not None and abs(perto[1][0] - cs[0]) <= folga:
                self._colado[(perto[0], s)] += 1
            elif folga + 0.7 * ls <= cs[0] <= self.largura - folga - 0.7 * ls:
                fila.append((s, cs, ls))                       # vazio: o bloco dele caberia na imagem
        fila.sort(key=lambda it: it[1][0])
        for i, _, _ in fila:
            self._vistos[i] += 1
        for (a, ca, la), (b, cb, lb) in zip(fila, fila[1:]):
            passo = max(1, int(round(float(np.linalg.norm(cb - ca)) / (PASSO_EM_LADOS * 0.5 * (la + lb)))))
            self._passos.setdefault((a, b), Counter())[passo] += 1

    def _vale(self, *ids):
        # bloco: precisa de `minimo` quadros; tag de slot (36h11, id fixo): 1 quadro basta
        return all(self._vistos[i] >= (1 if i in self.slot_ids else self.minimo) for i in ids)

    def estado(self):
        """({slot: bloco | None} do que se sabe, ancorado)."""
        viz = {}
        for (a, b), votos in self._passos.items():
            passo, n = votos.most_common(1)[0]
            if self._vale(a, b) and n >= (1 if (a in self.slot_ids or b in self.slot_ids) else self.minimo):
                viz.setdefault(a, []).append((b, passo))
                viz.setdefault(b, []).append((a, -passo))
        colado = {}
        for (b, s), k in self._colado.items():
            if k >= self.minimo:
                colado.setdefault(b, Counter())[self.slot_ids.index(s)] += k
        n = len(self.slot_ids)
        ancorados, soltos, visto = [], [], set()
        for raiz in list(viz) + list(colado):                  # cada grupo emendado
            if raiz in visto:
                continue
            pos, fila = {raiz: 0}, [raiz]
            while fila:
                a = fila.pop()
                for b, passo in viz.get(a, ()):
                    if b not in pos:
                        pos[b] = pos[a] + passo
                        fila.append(b)
            visto |= set(pos)
            anc = Counter()                                    # base = indice do slot + posicao na fila
            for i, p in pos.items():
                if i in self.slot_ids:
                    anc[self.slot_ids.index(i) + p] += self._vistos[i]
                for k, votos in colado.get(i, {}).items():
                    anc[k + p] += votos
            if anc:
                ancorados.append((pos, anc.most_common(1)[0][0]))
            else:
                soltos.append(pos)
        grupos, ancorado = ancorados, bool(ancorados)
        if not ancorados and soltos:
            # sem ancora: o maior grupo, com o da direita da imagem no primeiro slot (exato se ocupa todos)
            pos = max(soltos, key=len)
            base = max(pos.values())
            grupos, ancorado = [(pos, base)], base - min(pos.values()) + 1 == n
        est = {}
        for pos, base in grupos:                               # indice do slot diminui para a direita
            ocupados = []
            for i, p in pos.items():
                if i in self.slot_ids or not 0 <= base - p < n:
                    continue
                if est.setdefault(self.slot_ids[base - p], i) != i:
                    return {}, False                           # dois blocos no mesmo slot
                ocupados.append(base - p)
            vazios = [base - p for s, p in pos.items() if s in self.slot_ids]
            if ocupados:
                vazios += [k for k in range(min(ocupados), max(ocupados) + 1) if k not in ocupados]   # buraco na fila
            for k in vazios:
                if 0 <= k < n and est.setdefault(self.slot_ids[k], None) is not None:
                    return {}, False                           # vazio numa vista, ocupado em outra
        return est, ancorado


def completar(estado, esperados, slot_ids):
    """Todos os blocos do arquivo localizados => os slots que sobraram estao vazios
    (so' ha' esses blocos na mesa). Se faltar algum, devolve o estado como veio."""
    if not esperados or set(esperados) - {c for c in estado.values() if c is not None}:
        return dict(estado)
    return {s: estado.get(s) for s in slot_ids}


def alvo_por_ordem(slot_ids, ordem, direcao="left_to_right"):
    """{slot: cubo}: ordem[i] no i-esimo slot, da esquerda para a direita (ou ao contrario)."""
    fila = list(slot_ids) if direcao != "right_to_left" else list(reversed(slot_ids))
    return {s: c for s, c in zip(fila, ordem)}


def planejar(estado, alvo, a_bordo=(), capacidade=3):
    """Sequencia de ("pick"|"place", cubo, slot) que leva `estado` ao `alvo`.

    estado: {slot: cubo|None} (slots vistos); alvo: {slot: cubo}. So' move bloco
    visto e so' solta em slot visto vazio. Devolve (ops, sobra): sobra = blocos
    que ficaram fora do alvo (slot-alvo nao visto, ou potes insuficientes).
    """
    est = dict(estado)
    bordo = list(a_bordo)
    destino = {c: s for s, c in alvo.items()}
    ops = []

    def fecha(c):
        """Tirar `c` para abrir vaga resolve? Segue quem ocupa o lugar de quem ate'
        voltar ao slot de `c` ou achar um vazio; para em bloco que nao se pode mover."""
        origem = next(s for s, x in est.items() if x == c)
        s, vistos = destino[c], set()
        while s not in vistos:
            vistos.add(s)
            o = est.get(s, 0)
            if o is None or s == origem:
                return True
            if o == 0 or o not in destino or destino[o] not in est:
                return False
            s = destino[o]
        return False

    def fora_do_lugar():
        return sorted(c for s, c in est.items() if c is not None and c in destino and destino[c] != s
                      and destino[c] in est)

    for _ in range(4 * (len(est) + len(bordo)) + 4):
        solta = next((c for c in sorted(bordo) if c in destino and est.get(destino[c], 0) is None), None)
        if solta is not None:
            bordo.remove(solta)
            est[destino[solta]] = solta
            ops.append(("place", solta, destino[solta]))
            continue
        pend = fora_do_lugar()
        if not pend:
            break
        # bloco cujo slot-alvo ja' esta' vazio: pega e solta em seguida
        direto = next((c for c in pend if est[destino[c]] is None), None)
        if direto is not None and len(bordo) < capacidade:
            pega = direto
        else:
            # ciclo fechado: tira um bloco para abrir vaga (deixa 1 pote livre para
            # rodar o ciclo). So' se o ciclo fecha: senao o bloco ficaria a bordo.
            pega = next((c for c in pend if fecha(c)), None) if len(bordo) <= capacidade - 2 else None
            if pega is None:
                break
        origem = next(s for s, c in est.items() if c == pega)
        est[origem] = None
        bordo.append(pega)
        ops.append(("pick", pega, origem))
    sobra = sorted(set(fora_do_lugar()) | {c for c in bordo if c in destino})
    return ops, sobra


def simular(estado, ops, a_bordo=(), capacidade=3):
    """Aplica ops conferindo as regras (slot vazio ao soltar, pote livre ao pegar). Devolve (estado, bordo)."""
    est, bordo = dict(estado), list(a_bordo)
    for op, c, s in ops:
        if op == "pick":
            assert est.get(s) == c and len(bordo) < capacidade, (op, c, s, est, bordo)
            est[s] = None
            bordo.append(c)
        else:
            assert c in bordo and est.get(s, 0) is None, (op, c, s, est, bordo)
            bordo.remove(c)
            est[s] = c
    return est, bordo


def ordem(estado, slot_ids, direcao="left_to_right"):
    """ids dos blocos na fileira, da esquerda para a direita (so' slots ocupados)."""
    fila = list(slot_ids) if direcao != "right_to_left" else list(reversed(slot_ids))
    return [estado[s] for s in fila if estado.get(s) is not None]
