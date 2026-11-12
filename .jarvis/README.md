# .jarvis/ — memória compartilhada do workspace caramelo_ws

Memória persistente, em Markdown, para que uma sessão nova ou outro agente
entenda o projeto, recupere decisões, saiba onde o trabalho parou e continue
sem depender da conversa anterior. Não substitui o código nem os docs: o que
vale é o que está nos arquivos reais e nos resultados verificáveis.

## Ordem de leitura

1. Este README (protocolo).
2. `contexto.md` — o que é estável: objetivo, organização, restrições, preferências, comandos.
3. `estado.md` — tarefas ativas, último checkpoint, bloqueios, próximo passo.
4. `decisoes.md` e `historico.md` — só as entradas ligadas à tarefa que vai assumir.
5. `skills/retomar-trabalho.md` antes de começar; `skills/atualizar-memoria.md` antes de gravar.

## O que vai em cada arquivo

| Arquivo | Conteúdo | Atualizar quando |
|---|---|---|
| `contexto.md` | Objetivo e escopo, mapa do workspace, restrições técnicas, preferências explícitas do operador, comandos e caminhos verificados, referências | Algo estável mudar ou uma preferência nova for dita |
| `estado.md` | Objetivo ativo; tarefas com id, status (pendente / em andamento / bloqueada / concluída), responsável, checkpoint, arquivos, verificações, próximo passo | Ao assumir tarefa, em cada marco, antes de operação demorada, ao encerrar |
| `decisoes.md` | Data, decisão, motivo, consequências, origem, status (vigente / substituída por …) | Decisão tomada, revertida ou substituída |
| `historico.md` | Resumo datado: pedido, ação, resultado, pendências, referências | Fim de cada bloco de trabalho; correções do operador |
| `skills/` | Procedimentos reutilizáveis (nome, quando usar, pré-requisitos, passos, verificação, cuidados) | Procedimento repetido ≥ 2 vezes e validado |

## Protocolo dos agentes

**Ao iniciar ou assumir uma tarefa**: ler este README → `contexto.md` → `estado.md` →
decisões/histórico relevantes → conferir os arquivos reais (`git status`, `git log`,
código citado) → registrar em `estado.md` a tarefa assumida, o responsável e o próximo passo.

**Durante**: atualizar após marcos, decisões, correções do operador, bloqueios e mudanças
de escopo. Checkpoint entre etapas, sem esperar o fim. Antes de algo demorado (build
completo, calibração, teste no robô), anotar o que será feito e como verificar. Não
registrar cada comando.

**Antes de encerrar ou transferir**: atualizar o estado real; dizer o que foi alterado e o
que foi verificado; separar planejado / executado / validado; deixar o próximo passo
executável por outro agente.

**Interrupção abrupta**: a próxima sessão retoma do último checkpoint e confere o estado
real. Nada depois do checkpoint pode ser presumido como feito.

## Vários agentes

- Consultar a memória antes de agir e de novo antes de gravar em arquivo compartilhado.
- Registrar responsável por tarefa; dois agentes não editam a mesma tarefa ou arquivo ao mesmo tempo.
- Com coordenador, ele consolida o que os demais mandam.
- Antes de gravar: reler o arquivo e fazer edição pontual, preservando o que outros escreveram. Conflito → parar a escrita e reconciliar.

## Limitações (sem mecanismo extra nesta versão)

- Só Markdown: sem sincronização em tempo real, sem transação, sem lock. Agentes em cópias
  diferentes do workspace (PC de desenvolvimento × PC do robô) só se veem via git ou outro
  meio externo; nada disso está configurado aqui.
- Nenhum hook, watcher ou script: a atualização depende de os agentes seguirem o protocolo.
- A pasta não é backup. Se o workspace for perdido, a memória vai junto, salvo o que estiver no git.
- Existe outra memória, **não compartilhada**: a memória automática do Claude Code deste
  usuário em `~/.claude/projects/-home-robofei-work-caramelo-ws/memory/` (fora do repo, por
  máquina). `.jarvis/` é a fonte compartilhada; se divergirem, investigar e corrigir.
- As transcrições das conversas ficam em `~/.claude/projects/-home-robofei-work-caramelo-ws/*.jsonl`
  (gravação automática do Claude Code, só nesta máquina). Servem para recuperar pedidos
  antigos; o que importa deve ser resumido aqui, não copiado. Tabela completa de onde cada
  coisa fica: `contexto.md`, seção "Onde a memória e os históricos ficam".
- O `CLAUDE.md` da raiz aponta para as seções §5 e §8 do `README.md` **do projeto**
  (build/testes e convenções), não deste README.
- Os arquivos em `skills/` são procedimentos locais lidos por quem seguir este README; não
  são skills descobertas automaticamente por nenhuma ferramenta (não verificado nem configurado).

## Privacidade

Não gravar senhas, tokens, chaves, conteúdo de `.env`, IPs privados sem necessidade nem
dados pessoais. Indicar onde a configuração é obtida, nunca o valor.
