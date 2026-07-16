# Keepout Filter — Paredes Virtuais (RK-04)

**Problema (RK-04):** a fita zebrada de piso (e qualquer parede virtual de altura ~0)
é **invisível ao LiDAR** — não aparece no `/scan` nem no costmap por obstáculo. Sem
tratamento, o robô passa por cima → colisão (que no rulebook custa metade da pontuação).

**Solução (mecanismo oficial do Nav2):** o **Keepout Filter**. Uma máscara (imagem)
marca as zonas proibidas; dois servidores publicam essa máscara e sua metainformação;
um plugin `KeepoutFilter` nos costmaps global e local transforma as zonas pretas da
máscara em células **letais**, como se fossem parede.

Este pacote já traz todo o mecanismo pronto e **desligado por padrão** (`use_keepout:=false`).

---

## Como está montado (para referência)

- **Máscara por mapa:** `caramelo_mapping/maps/<mapa>/keepout_mask.{pgm,yaml}` — mesma
  geometria (tamanho, resolução, origem) do `map.pgm`. `mode: scale` → pixel **preto (0)**
  = keepout (letal), **branco (255)** = livre.
- **Servidores** (sobem só com `use_keepout:=true`, em `global_localization.launch.py`):
  - `filter_mask_server` (nav2_map_server) → publica a máscara em `/keepout_filter_mask`.
  - `costmap_filter_info_server` → publica `/costmap_filter_info`.
  - `lifecycle_manager_costmap_filters` gerencia os dois.
- **Plugin nos costmaps:** injetado via override do launch (`navigation.launch.py`) **só
  quando** `use_keepout:=true`, nos costmaps global e local (`filters: ["keepout_filter"]`).
  Config dos servidores: `caramelo_localization/config/keepout_filter.yaml`.

---

## Passo a passo para usar

### 1. Gerar o canvas da máscara (uma vez por mapa)

Já existe canvas todo-livre para `sala_520` e `arena_teste`. Para um mapa novo:
```bash
ros2 run caramelo_navigation init_keepout_mask.py --map-name <mapa>
# ou, no fonte:
python3 src/caramelo_navigation/scripts/init_keepout_mask.py \
    --map-name <mapa> --map-dir src/caramelo_mapping/maps
```
Isso cria `keepout_mask.pgm` (todo branco = nenhum bloqueio) e `keepout_mask.yaml`.

### 2. Pintar as paredes virtuais

Abra `maps/<mapa>/keepout_mask.pgm` num editor de imagem (GIMP, Krita) e pinte de
**PRETO (0)** exatamente onde estão as faixas/zonas proibidas, alinhando com o `map.pgm`
(mesma escala e origem). Salve mantendo o formato PGM. Deixe branco o resto.

> Dica: abra `map.pgm` e `keepout_mask.pgm` como camadas no mesmo arquivo para pintar
> por cima das referências reais, depois exporte só a camada da máscara.

### 3. Subir a navegação com o keepout ligado

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=<mapa> use_keepout:=true
```
No RViz, adicione um `Map` display no tópico `/keepout_filter_mask` para conferir que a
máscara aparece alinhada ao mapa. As zonas pretas devem virar obstáculo no costmap.

---

## Notas

- **Desligado por padrão:** sem `use_keepout:=true`, nada do keepout sobe e os costmaps
  ficam idênticos ao de antes (sem avisos de "esperando CostmapFilterInfo").
- **Máscara todo-livre = sem efeito:** é seguro subir com `use_keepout:=true` mesmo antes
  de pintar — não bloqueia nada até você pintar de preto.
- **Vale para os dois costmaps:** global (planejamento) e local (execução), então a parede
  virtual é respeitada tanto no plano quanto perto do robô.
- **Não confundir com obstáculo real:** o keepout é estático, vem do mapa; obstáculos
  dinâmicos continuam vindo do LiDAR pela `obstacle_layer`.
