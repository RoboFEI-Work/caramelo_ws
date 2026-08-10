# Estado de uma missao do Caramelo (navegacao + manipulacao), publicado em
# /caramelo/mission/status.
#
# Existe porque, ate' agora, uma missao so' podia ser acompanhada lendo o
# terminal: o run_mission e' um orquestrador de subprocessos e o bt_yaml_executor
# so' escreve em stdout/rosout. Nenhuma interface (GUI Qt ou painel web) conseguia
# saber em que passo a missao estava sem parsear terminal.
#
# Publicado com QoS transient_local (depth 1): quem conecta no meio da missao
# recebe imediatamente o ultimo estado.

# --- state ---
uint8 STATE_IDLE=0
uint8 STATE_PLANNING=1     # gerando o actions.yaml / montando a arvore
uint8 STATE_PREFLIGHT=2    # esperando os action servers no grafo
uint8 STATE_RUNNING=3      # arvore em execucao
uint8 STATE_DONE=4
uint8 STATE_FAILED=5
uint8 STATE_ABORTED=6      # Ctrl-C ou cancelamento

std_msgs/Header header

string mission_id          # identificador da execucao (nome do actions.yaml)
string task_id             # tarefa da competicao: BMT, ATT1, BTT1, ...

# Passo atual dentro da arvore gerada. Conta TODOS os passos, inclusive o
# prologo (braco em home) e o epilogo (home + ida ao FINISH) que o executor
# acrescenta sozinho -- e' isso que uma barra de progresso precisa. Vale -1
# quando nenhum passo esta' em execucao (planejamento, pre-flight, fim).
int32 action_index
int32 action_total

string action_kind         # goto | pick | place | home
string action_target       # WS1 | tag_01 | Mesa15 | home
string stage               # subestagio: iniciado, concluido, falhou, staging,
                           # align, undock, nav, ...

uint8 state
string message             # texto legivel para o operador
float64 elapsed            # segundos desde o inicio da missao
