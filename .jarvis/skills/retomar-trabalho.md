# Skill: retomar-trabalho

**Finalidade**: assumir uma tarefa (sessão nova, outro agente ou após interrupção) sem
depender da conversa anterior.

**Quando usar**: no início de toda sessão neste workspace e ao receber uma tarefa delegada.

**Pré-requisitos**: acesso ao workspace `~/caramelo_ws` e ao git.

## Passos
1. Ler `.jarvis/README.md`, `.jarvis/contexto.md` e `.jarvis/estado.md`.
2. Em `estado.md`, localizar a tarefa (id) e o último checkpoint. Se não houver tarefa, ler o
   objetivo ativo e o pedido do operador.
3. Ler em `decisoes.md` as decisões ligadas à área (PP, nav, docking…) e em `historico.md`
   as entradas das últimas datas.
4. Conferir o estado real: `git status`, `git log --oneline -10`, `git stash list`; abrir os
   arquivos citados no checkpoint e confirmar que o que a memória diz existe.
5. Se a memória contradizer os arquivos, corrigir a memória (anotar a origem) antes de agir.
6. Registrar em `estado.md`: tarefa assumida, responsável, data e próximo passo concreto.
7. Só então trabalhar. Se for delegar a subagentes, passar a eles este protocolo.

## Verificação
Deve ser possível responder, só com a memória e os arquivos referenciados: qual o objetivo,
onde a tarefa parou e qual o próximo comando/edição.

## Cuidados
- Nada após o último checkpoint pode ser presumido como feito.
- Este PC é o do robô: nunca `pkill -f` por nome, nunca mover o robô, nunca commitar sem pedido.
- Instruções atuais e explícitas do operador vencem registros antigos.
