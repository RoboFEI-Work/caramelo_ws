# caramelo_web — painel do robô no navegador

Abre o Caramelo inteiro numa página: acompanhar, comandar, dirigir na mão,
disparar prova, olhar a tela do computador do robô e abrir um terminal — de
outro computador da rede, sem instalar nada no cliente.

```
ros2 launch caramelo_web web_panel.launch.py
```

O servidor imprime, no terminal em que subiu, o endereço completo já com a
chave de acesso:

```
============================================================
 PAINEL DO CARAMELO no ar.
 Abra um destes endereços no navegador (o link já leva a chave):

   http://192.168.0.42:8080/?t=Yc3Q...
   http://127.0.0.1:8080/?t=Yc3Q...
============================================================
```

## A chave de acesso

O painel escuta em `0.0.0.0` porque o operador não fica sentado no robô. Desde
que ele ganhou terminal e direção manual, isso passaria a significar "shell e
volante para qualquer um no mesmo Wi-Fi". A tranca é assimétrica:

- **Livre**: mapa, posição, sensores, saúde, andamento da prova. O painel abre
  sem atrito para quem só quer olhar.
- **Com chave**: tudo que MUDA o estado do robô, mais o terminal e o espelho de
  tela.

A chave é sorteada a cada arranque (`secrets.token_urlsafe`) e nunca é gravada
em disco. Para fixá-la — deixar um favorito, colar um QR code no robô:

```
ros2 launch caramelo_web web_panel.launch.py token:=umaChaveQualquer
ros2 run caramelo_web caramelo_web_server --port 8080 --token umaChaveQualquer
```

Ela vai na URL (`?t=...`) ou no cabeçalho `X-Caramelo-Chave`, para quem chamar a
API por script. Isto **não é autenticação**: não há usuários, sessão nem HTTPS.
É uma tranca contra o vizinho de bancada e contra o clique curioso — quem tiver
o link tem o robô.

## O que tem em cada aba

| aba | o que faz |
|-----|-----------|
| **Operação** | mapa da arena com o robô, o sensor de distância, mesas e pontos salvos. Clique manda o robô até ali; arrastando, a seta define para que lado ele deve ficar virado. O mesmo clique, no outro modo, avisa ao robô onde ele está quando a localização se perde. Parar, esquecer obstáculos, encostar/sair de uma mesa. |
| **Missão** | escolhe a prova, monta o passo a passo sem mover o robô e executa. Interrompe pelo caminho limpo (o executor faz o halt da árvore; nada é morto à força). |
| **Direção manual** | volante na tela e teclado (W A S D, Q/E, espaço). Três velocidades. |
| **Terminal** | um bash de verdade no computador do robô, já com o ambiente ROS carregado. |
| **Sistema** | saúde dos componentes, últimos avisos, programas no ar e o espelho da tela. |

## Direção manual: por que existe watchdog

O ESC desta base interpreta **ausência de PWM como ré em velocidade máxima**, e
não existe botão de emergência de hardware. Então "parar de mandar comando" não
é "parar" — é o pior comando possível. O nó ROS resolve isso com duas janelas,
contadas do último comando vindo do navegador:

- **300 ms** sem comando novo → publica `Twist` **zero** (o freio).
- **900 ms** → para de publicar de vez.

A segunda janela existe por causa do `twist_mux`: o comando manual tem
prioridade 90 e a navegação, 80. Um fluxo eterno de zeros deixaria a navegação
muda para sempre. O `twist_mux` esquece o teclado depois de 0,5 s, então parar
0,6 s depois do último comando garante que a base recebeu vários zeros **antes**
de o volante voltar para a navegação.

Pelo mesmo motivo, o painel **nunca** manda um "parar" que ninguém pediu: trocar
de aba ou minimizar a janela não freia um robô que estava navegando sozinho.

## Duas regras que não se negociam

1. **Nunca publicar em `/goal_pose`.** O `bt_navigator` do Nav2 Jazzy também
   assina esse tópico: um clique no mapa viraria DOIS goals, um preemptando o
   outro, com o painel exibindo o status do que já morreu. Só a action
   `navigate_to_pose`.
2. **Nunca publicar em `/mecanum_controller/reference`.** Escrever lá pula o
   `twist_mux` e o `collision_monitor`, ou seja, entrega a base sem nenhum
   freio automático. O caminho limpo é `cmd_vel_key`.

## Espelho da tela

Só visualização — mostra a tela, não mexe nela. É um MJPEG
(`multipart/x-mixed-replace`), que qualquer navegador desenha dentro de um
`<img>` sem codec nem plugin. ~10 fps, JPEG de qualidade média, largura
reduzida: serve para acompanhar, não para gravar vídeo.

Depende do `mss`, instalado no usuário e **sem sudo**:

```
python3 -m pip install --user --break-system-packages mss
```

Neste robô o ambiente é WSLg: existe `WAYLAND_DISPLAY`, mas também existe
`DISPLAY=:0` com um XWayland servindo o mesmo desktop, e é por ele que o `mss`
captura (testado: `monitors[0]` = 1920x1200, captura e JPEG OK). Se um dia o
robô subir sem ambiente gráfico, a rota responde 503 com a frase explicando —
nunca com exceção.

## Terminal

`pty` da stdlib + WebSocket do FastAPI. Um bash por conexão, morto junto com
ela (SIGHUP no grupo inteiro, para não deixar `ros2 launch` órfão). Sem `pty` de
verdade não haveria Tab, curses nem Ctrl+C — sinal de teclado só existe onde
existe terminal de controle.

O ambiente vem do próprio processo do painel (mesmo overlay que o resto do
stack) mais um rc que sourceia `/etc/bash.bashrc`, `~/.bashrc` e
`~/.caramelo_env.sh`. O último é repetido de propósito: sem ele, um layout de
Ubuntu em que `~/.profile` não chama o `~/.bashrc` abriria um terminal sem
`ROS_DOMAIN_ID=67`, e o sintoma seria `ros2 topic list` vazio com o robô
inteiro no ar.

O front usa **xterm.js versionado** em `www/vendor/` — sem CDN, sem
`npm install`. Versões e origem exata: `www/vendor/LEIAME.md`.

## Armadilhas medidas nesta rodada (2026-08-18)

Três coisas que funcionavam "na teoria" e não funcionavam na máquina. Ficam
registradas aqui porque todas voltam se alguém mexer no lugar errado:

1. **`--symlink-install` + `StaticFiles`.** Todo arquivo do `www/` instalado é
   um link para a árvore fonte, e o `StaticFiles` recusa por padrão o que
   resolve para fora da pasta servida. Sem `follow_symlink=True`,
   `/vendor/xterm.js` dá 404 e a aba Terminal abre **preta, sem erro nenhum** —
   enquanto a página inicial continua funcionando (ela vai por `FileResponse`,
   que não passa por essa checagem).

2. **Matar o terminal pelo grupo de processos não basta.** O bash interativo
   põe cada job no seu próprio grupo, e o SIGHUP que o kernel manda ao fechar o
   pty só alcança o grupo de primeiro plano. Fechar a aba com um `sleep &`
   rodando deixava o processo vivo, reparentado ao init. O que junta todo mundo
   é a **sessão** (`setsid` do `login_tty`): o encerramento varre `/proc` por
   `sid == pid do bash`.

3. **`finally` de gerador assíncrono não é determinístico.** A vaga do espelho
   de tela era devolvida no `finally` do gerador do corpo MJPEG — que, quando o
   navegador some, fica suspenso no `yield` até o coletor de lixo passar.
   Sintoma: fechar e reabrir a janela do espelho respondia "já há espelhos de
   tela demais". A liberação passou para uma tarefa de fim de resposta
   (`BackgroundTask`), que roda também na desconexão.

E uma medição que mudou o desenho: o pulso do teleop era um `create_timer(0.05)`
do ROS e saía a **10,3 Hz**, com intervalo travado em 100 ms (o executor
coalesce com o timer de pose). Para um heartbeat de tela seria irrelevante; para
o freio deste robô, não. Virou thread própria — medido depois: 20,0 Hz,
intervalo 50–52 ms.

## Rotas

Leitura (livre): `/`, `/api/acesso`, `/api/saude`, `/api/arenas`,
`/api/arenas/{n}/meta`, `/api/arenas/{n}/mapa.png`, `/api/missoes`,
`/api/missoes/plano`, `/api/tela/estado`, `ws://…/ws/estado`.

Com chave: `POST /api/meta`, `/api/navegacao/cancelar`,
`/api/navegacao/costmaps`, `/api/pose_inicial`, `/api/dock`, `/api/undock`,
`/api/missao/plano`, `/api/missao/rodar`, `/api/missao/abortar`, `/api/teleop`;
`GET /api/tela.mjpeg`; `ws://…/ws/terminal`.

## Conferir sem robô

```
ros2 run caramelo_web caramelo_web_server --port 8080 --token teste --sem-ros
curl -s localhost:8080/api/arenas
curl -s -X POST localhost:8080/api/teleop            # 403, sem a chave
curl -s -X POST 'localhost:8080/api/teleop?t=teste'  # 503, sem ROS
```
