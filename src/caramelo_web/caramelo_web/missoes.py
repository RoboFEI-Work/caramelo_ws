"""Tasks de competição que o painel pode disparar.

Mesma descoberta que a GUI Qt faz (competicao_module.cpp): varre a pasta
`missions/` na raiz do workspace (fonte única das tarefas) e a subpasta
`missions/tasks geradas/` (arquivos OFICIAIS do @Work Commander, formato 2026),
e só aceita YAML que tenha `task_id`. O filtro por `task_id` não é firula: a
pasta guarda também arquivos que NÃO são task (README, `exemplos/` com YAML de
ações), e oferecer um deles ao operador produziria uma missão que falha no
planejador sem dizer por quê.

Não há fallback para share de pacote (regra do repositório): as tasks vivem em
missions/. A raiz do workspace é deduzida da árvore fonte dos mapas
(<ws>/src/caramelo_mapping/maps -> <ws>), como a GUI faz; sem ela, ~/caramelo_ws.

O painel expõe LISTA, nunca campo de texto: caminho de arquivo digitado à mão é
exatamente o tipo de erro que só aparece quando o cronômetro da prova já está
correndo.
"""
from pathlib import Path

import yaml

from caramelo_web import arenas

# Subpastas de missions/ varridas, nesta ordem. "" é a própria raiz.
SUBPASTAS = ("tasks geradas", "")


def raiz_do_workspace() -> Path:
    mapas = arenas.pasta_dos_mapas()
    ws = mapas.parent.parent.parent      # <ws>/src/caramelo_mapping/maps -> <ws>
    if (ws / "missions").is_dir():
        return ws
    return Path.home() / "caramelo_ws"


def _pastas() -> list:
    raiz = raiz_do_workspace() / "missions"
    return [raiz / sub if sub else raiz for sub in SUBPASTAS]


def listar() -> list:
    raiz = raiz_do_workspace() / "missions"
    achadas = []
    for pasta in _pastas():
        if not pasta.is_dir():
            continue
        for arquivo in sorted(pasta.glob("*.yaml")):
            try:
                with arquivo.open("r", encoding="utf-8") as f:
                    dados = yaml.safe_load(f) or {}
            except Exception:
                continue
            if not isinstance(dados, dict):
                continue
            task_id = str(dados.get("task_id") or "").strip()
            if not task_id:
                continue
            nome = str(dados.get("name") or "").strip()
            # O arquivo entra no rótulo: a task oficial (tasks geradas/BMT_...)
            # e a de teste à mão (task_BMT.yaml) podem ter o mesmo task_id.
            relativo = str(arquivo.relative_to(raiz))
            achadas.append({
                "caminho": str(arquivo),
                "arquivo": relativo,
                "task_id": task_id,
                "nome": nome,
                "rotulo": f"{task_id} — {nome}  ({relativo})" if nome
                          else f"{task_id}  ({relativo})",
                "duracao_min": dados.get("duration_min"),
                # active_service_areas aparece nas duas formas nos YAMLs que
                # existem hoje: lista de nomes (BMT) e lista de dicionarios com
                # id (ATT1). Normalizar aqui evita "[object Object]" na tela.
                "areas": [
                    str(a.get("id", "")) if isinstance(a, dict) else str(a)
                    for a in (dados.get("active_service_areas") or [])
                ],
                "objetos": len(dados.get("objects") or []),
            })
    return achadas


def valida(caminho: str) -> bool:
    """O caminho veio mesmo da nossa lista?

    O painel manda o caminho de volta ao servidor, e caminho vindo do navegador
    é entrada de fora. Sem esta conferência, um pedido forjado faria o servidor
    de missão tentar abrir qualquer arquivo do disco.
    """
    return any(m["caminho"] == caminho for m in listar())
