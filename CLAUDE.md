# CLAUDE.md — caramelo_ws

Instruções do projeto para agentes (Claude Code e outros). As convenções de código, build,
testes e caminhos estão em `README.md` (§5, §8); docs em `docs/`.

<!-- jarvis:inicio -->
## Memória compartilhada (.jarvis/)

- Ao iniciar a sessão ou assumir uma tarefa, leia `.jarvis/README.md` e siga a ordem de
  leitura (`contexto.md` → `estado.md` → decisões/histórico relevantes).
- Consulte a memória antes de trabalhar e confira os arquivos reais antes de agir.
- Atualize `.jarvis/estado.md`, `decisoes.md` e `historico.md` conforme o protocolo do
  README (marcos, decisões, correções do operador, bloqueios, antes de encerrar).
- Ao delegar a subagentes, passe estas mesmas orientações no prompt.
- A atualização depende do agente seguir o protocolo: não há hook nem automação.
<!-- jarvis:fim -->
