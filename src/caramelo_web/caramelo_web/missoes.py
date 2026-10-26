"""Tasks de competição que o painel pode disparar.

Mesma descoberta que a GUI Qt faz (competicao_module.cpp): varre o share do
manip_bt e a pasta ~/caramelo_ws/missions, e só aceita YAML que tenha `task_id`.
O filtro por `task_id` não é firula: as duas pastas guardam também YAMLs
vizinhos que NÃO são task (ws_table_mapping.yaml, por exemplo), e oferecer um
deles ao operador produziria uma missão que falha no planejador sem dizer
por quê.

O painel expõe LISTA, nunca campo de texto: caminho de arquivo digitado à mão é
exatamente o tipo de erro que só aparece quando o cronômetro da prova já está
correndo.
"""
from pathlib import Path

import yaml


def _pastas() -> list:
    pastas = []
    try:
        from ament_index_python.packages import get_package_share_directory
        pastas.append(Path(get_package_share_directory("manip_bt")) / "behavior_tree_manip")
    except Exception:
        # manip_bt pode não estar sourceado (perfil só de navegação); a pasta
        # missions/ ainda vale.
        pass
    pastas.append(Path.home() / "caramelo_ws/missions")
    return pastas


def listar() -> list:
    achadas = []
    vistos = set()
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
            if not task_id or task_id in vistos:
                continue
            vistos.add(task_id)
            nome = str(dados.get("name") or "").strip()
            achadas.append({
                "caminho": str(arquivo),
                "task_id": task_id,
                "nome": nome,
                "rotulo": f"{task_id} — {nome}" if nome else task_id,
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
