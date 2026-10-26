# Bibliotecas de terceiros, versionadas no repositório

Em competição não há internet. Nada aqui pode virar `npm install` nem CDN no
caminho de abrir o painel: se o arquivo não estiver no repositório, o terminal
do painel simplesmente não abre na hora em que o operador precisa dele.

Por isso os arquivos abaixo são **cópias fixas**, commitadas. Não existe passo
de build: o `index.html` os carrega com `<script src>` direto.

| arquivo         | pacote npm          | versão | sha256 (prefixo) |
|-----------------|---------------------|--------|------------------|
| `xterm.js`      | `@xterm/xterm`      | 6.0.0  | `14903579ff5466` |
| `xterm.css`     | `@xterm/xterm`      | 6.0.0  | `854a7c0fb70e8b` |
| `addon-fit.js`  | `@xterm/addon-fit`  | 0.11.0 | `ba3ea256ce0620` |
| `LICENSE-xterm.txt` | `@xterm/xterm`  | 6.0.0  | licença MIT      |

Origem exata (baixado em 2026-08-18, direto do registry, sem `node_modules`):

    curl -L https://registry.npmjs.org/@xterm/xterm/-/xterm-6.0.0.tgz -o xterm.tgz
    tar xzf xterm.tgz package/lib/xterm.js package/css/xterm.css package/LICENSE

    curl -L https://registry.npmjs.org/@xterm/addon-fit/-/addon-fit-0.11.0.tgz -o fit.tgz
    tar xzf fit.tgz package/lib/addon-fit.js

São as builds **UMD** (`lib/`), não as ESM (`lib/*.mjs`): sem bundler, a UMD é a
única que se registra sozinha no `window`. Depois de carregadas existem
`window.Terminal` e `window.FitAddon.FitAddon` — é assim que o `index.html` as
usa, e é o que quebra se alguém trocar por um arquivo `.mjs`.

## Para atualizar

Repita os dois `curl` com a versão nova, substitua os arquivos, atualize esta
tabela (inclusive os sha256, com `sha256sum`) e **abra o terminal do painel uma
vez** antes de dar por feito. O xterm muda API entre versões maiores; o sintoma
de uma troca malfeita é a aba Terminal ficar preta sem mensagem de erro
nenhuma, porque `new Terminal(...)` estourou antes de desenhar.
